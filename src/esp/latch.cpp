#include "estop/estop.hpp"

#include "esp_log.h"
#include "rtps_participant.hpp"

#include "time_util.hpp"

namespace estop {

using core::LatchEvent;
using core::Time;

namespace {

constexpr const char *kTag = "estop";

const char *event_name(LatchEvent::Kind kind) {
  switch (kind) {
  case LatchEvent::Kind::LATCHED:
    return "LATCHED";
  case LatchEvent::Kind::CLEAR_ATTEMPT_STARTED:
    return "clear attempt started";
  case LatchEvent::Kind::CLEAR_ATTEMPT_REFUSED:
    return "clear attempt refused";
  case LatchEvent::Kind::CLEAR_ATTEMPT_EXPIRED:
    return "clear attempt expired";
  case LatchEvent::Kind::RELEASED:
    return "RELEASED";
  }
  return "?";
}

} // namespace

Latch::Latch(const Config &config)
    : config_(config)
    , logic_({.watched = config.watched, .policy = config.policy}) {}

bool Latch::start() {
  if (task_ != nullptr) {
    return true;
  }
  rx_queue_ = xQueueCreateStatic(detail::kRxQueueDepth, sizeof(detail::Rx), rx_queue_storage_,
                                 &rx_queue_buf_);
  command_queue_ = xQueueCreateStatic(detail::kCommandQueueDepth, sizeof(detail::Command),
                                      command_queue_storage_, &command_queue_buf_);
  status_queue_ =
      xQueueCreateStatic(1, sizeof(LatchStatus), status_queue_storage_, &status_queue_buf_);
  if (rx_queue_ == nullptr || command_queue_ == nullptr || status_queue_ == nullptr) {
    return false;
  }
  // Room for the longest cause and target, reserved once, so sending never allocates.
  tx_.header.frame_id = "";
  tx_.source = rammp::kEStopClearer;
  tx_.reason.reserve(sizeof(LatchStatus::cause));
  tx_.clear_target.reserve(sizeof(core::Decoded::source));
  (void)xQueueOverwrite(status_queue_, &status_);
  task_ = xTaskCreateStaticPinnedToCore(task_main, "estop_latch", detail::kStackBytes, this,
                                        config_.task_priority, task_stack_, &task_tcb_,
                                        config_.task_core);
  return task_ != nullptr;
}

bool Latch::attach(espp::RtpsParticipant &participant) {
  if (task_ == nullptr) {
    return false; // start() first: received samples go to its queue
  }
  if (participant_.load() != nullptr) {
    return true;
  }
  if (!participant.add_writer({.topic = rammp::kEStop.name, .type_name = rammp::kEStop.type})) {
    return false;
  }
  participant_.store(&participant);
  return participant.add_reader(
      {.topic = rammp::kEStop.name,
       .type_name = rammp::kEStop.type,
       .on_sample = [this](std::span<const uint8_t> payload) { on_sample(payload); }});
}

void Latch::start_clear_attempt() {
  const detail::Command command{.kind = detail::Command::Kind::CLEAR_ATTEMPT,
                                .gpio = -1,
                                .contact = Contact::NO,
                                .reason = {}};
  if (task_ != nullptr && xQueueSend(command_queue_, &command, 0) == pdTRUE) {
    xTaskNotifyGive(task_);
  }
}

void Latch::engage(const char *reason) {
  detail::Command command{
      .kind = detail::Command::Kind::ENGAGE, .gpio = -1, .contact = Contact::NO, .reason = {}};
  core::copy_text(command.reason, reason != nullptr ? reason : "");
  if (task_ == nullptr) {
    return; // not started: active() is already true
  }
  if (xQueueSend(command_queue_, &command, 0) != pdTRUE) {
    engage_pending_.store(true); // the queue is full: engage anyway, without the reason
  }
  xTaskNotifyGive(task_);
}

LatchStatus Latch::status() const {
  LatchStatus status;
  if (status_queue_ == nullptr || xQueuePeek(status_queue_, &status, 0) != pdTRUE) {
    status.latched = true;
  }
  return status;
}

void Latch::on_sample(std::span<const uint8_t> payload) {
  detail::Rx rx{};
  rx.rx = detail::now(); // receive time on the steady clock: what freshness means
  if (!core::decode(payload, rammp::kEStopClearer, rammp::kEStopClearer, rx.msg)) {
    rx_dropped_.fetch_add(1, std::memory_order_relaxed);
    return;
  }
  if (rx.msg.from_self) {
    return; // our own sample, looped back
  }
  if (xQueueSend(rx_queue_, &rx, 0) != pdTRUE) {
    rx_dropped_.fetch_add(1, std::memory_order_relaxed);
    return;
  }
  xTaskNotifyGive(task_);
}

void Latch::send(Time now) {
  const auto out = logic_.next(now);
  tx_.header.stamp = detail::stamp_now();
  tx_.state = out.state;
  tx_.reason.assign(out.reason);
  tx_.pub_seq = out.pub_seq;
  tx_.clear_target.assign(out.clear_target);
  tx_.clear_pub_seq = out.clear_pub_seq;
  const std::size_t size = core::encode(tx_, tx_buf_);
  espp::RtpsParticipant *participant = participant_.load();
  if (size == 0) {
    status_.tx_errors++;
    return;
  }
  if (participant != nullptr &&
      !participant->publish(rammp::kEStop.name, std::span<const uint8_t>(tx_buf_.data(), size))) {
    status_.tx_errors++;
  }
}

void Latch::handle(const detail::Command &command, Time now) {
  switch (command.kind) {
  case detail::Command::Kind::ENGAGE:
    logic_.engage(command.reason);
    break;
  case detail::Command::Kind::CLEAR_ATTEMPT:
    (void)logic_.start_clear_attempt(now, switch_held());
    break;
  case detail::Command::Kind::SET_INPUT:
    break; // the node's
  }
}

// A configured switch that is not armed counts as held: it cannot be trusted to stop the
// robot, so no clear attempt may start or finish (the latch starts latched).
bool Latch::switch_held() const {
  return config_.gpio >= 0 && (!input_.armed() || input_.pressed());
}

// The clear input starts one attempt per hold of clear_hold; releasing it re-arms.
void Latch::poll_clear_input(Time now, bool switch_held) {
  if (!clear_input_.pressed()) {
    clear_since_.reset();
    clear_used_ = false;
    return;
  }
  if (!clear_since_) {
    clear_since_ = now;
  }
  if (!clear_used_ && now - *clear_since_ >= config_.clear_hold) {
    clear_used_ = true;
    (void)logic_.start_clear_attempt(now, switch_held);
  }
}

void Latch::task_main(void *self) { static_cast<Latch *>(self)->run(); }

void Latch::run() {
  uint32_t edges_seen = 0;
  for (;;) {
    const Time before = detail::now();
    const Time next = logic_.next_send();
    (void)ulTaskNotifyTake(pdTRUE, detail::ticks_until(before, next));
    const Time now = detail::now();

    // Arming retried every period until it works; until then the latch holds (an
    // unarmed switch cannot be trusted to stop it).
    if (config_.gpio >= 0 && !input_.armed() &&
        input_.arm(config_.gpio, config_.contact, detail::EdgeInput::Edges::PRESS, task_) &&
        input_.pressed()) {
      detail::switch_reason(reason_, config_.gpio);
      logic_.engage(reason_);
    }
    if (config_.clear_gpio >= 0 && !clear_input_.armed()) {
      (void)clear_input_.arm(config_.clear_gpio, config_.clear_contact,
                             detail::EdgeInput::Edges::BOTH, task_);
    }
    if (const uint32_t edges = input_.edges(); edges != edges_seen) {
      edges_seen = edges;
      detail::switch_reason(reason_, config_.gpio);
      logic_.engage(reason_);
    }

    detail::Command command;
    while (xQueueReceive(command_queue_, &command, 0) == pdTRUE) {
      handle(command, now);
    }
    if (engage_pending_.exchange(false)) {
      logic_.engage("engage");
    }

    detail::Rx rx;
    for (UBaseType_t i = 0; i < detail::kRxQueueDepth && xQueueReceive(rx_queue_, &rx, 0) == pdTRUE;
         i++) {
      logic_.receive(rx.msg, rx.rx);
    }

    poll_clear_input(now, switch_held());
    logic_.tick(now, switch_held());
    if (logic_.send_due(now)) {
      send(now);
    }

    LatchEvent event;
    while (logic_.take_event(event)) {
      if (config_.log_events) {
        ESP_LOGI(kTag, "%s%s%s", event_name(event.kind), event.text[0] != '\0' ? ": " : "",
                 event.text);
      }
    }

    status_.latched = logic_.latched();
    status_.clear_attempt_running = logic_.clear_attempt_running();
    status_.armed = config_.gpio < 0 || input_.armed();
    status_.clear_armed = config_.clear_gpio < 0 || clear_input_.armed();
    core::copy_text(status_.cause, logic_.cause());
    status_.clear_attempts = logic_.counters().clear_attempts;
    status_.releases = logic_.counters().releases;
    status_.rx_errors = rx_dropped_.load(std::memory_order_relaxed);
    status_.pub_seq = logic_.pub_seq();
    active_.store(status_.latched);
    (void)xQueueOverwrite(status_queue_, &status_);
  }
}

} // namespace estop
