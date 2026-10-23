#include "estop/estop.hpp"

#include <cstring>

#include "esp_timer.h"
#include "rtps_participant.hpp"

#include "time_util.hpp"

namespace estop {

using core::Time;

Node::Node(const Config &config)
    : config_(config)
    , logic_({.name = config.name != nullptr ? config.name : "",
              .policy = config.policy,
              .require_latch = config.require_latch}) {}

bool Node::start() {
  if (task_ != nullptr) {
    return true;
  }
  if (config_.name == nullptr || std::strlen(config_.name) == 0 || std::strlen(config_.name) > 15) {
    return false; // EStop.source must fit std::string's in-place buffer
  }
  rx_queue_ = xQueueCreateStatic(detail::kRxQueueDepth, sizeof(detail::Rx), rx_queue_storage_,
                                 &rx_queue_buf_);
  command_queue_ = xQueueCreateStatic(detail::kCommandQueueDepth, sizeof(detail::Command),
                                      command_queue_storage_, &command_queue_buf_);
  status_queue_ =
      xQueueCreateStatic(1, sizeof(NodeStatus), status_queue_storage_, &status_queue_buf_);
  if (rx_queue_ == nullptr || command_queue_ == nullptr || status_queue_ == nullptr) {
    return false;
  }
  // Every string this node sends, set once here: from now on only the POD fields change
  // and the reason swaps between strings that fit the in-place buffer.
  tx_.header.frame_id = "";
  tx_.source = config_.name;
  tx_.reason = "";
  tx_.clear_target = "";
  tx_.clear_pub_seq = 0;
  status_.input = config_.input;
  (void)xQueueOverwrite(status_queue_, &status_);
  task_ = xTaskCreateStaticPinnedToCore(task_main, "estop", detail::kStackBytes, this,
                                        config_.task_priority, task_stack_, &task_tcb_,
                                        config_.task_core);
  return task_ != nullptr;
}

bool Node::attach(espp::RtpsParticipant &participant) {
  if (task_ == nullptr) {
    return false; // start() first: received samples go to its queue
  }
  if (participant_.load() != nullptr) {
    return true;
  }
  // Best-effort, like every RAMMP topic: the period resend is the reliability.
  if (!participant.add_writer({.topic = rammp::kEStop.name, .type_name = rammp::kEStop.type})) {
    return false;
  }
  participant_.store(&participant);
  return participant.add_reader(
      {.topic = rammp::kEStop.name,
       .type_name = rammp::kEStop.type,
       .on_sample = [this](std::span<const uint8_t> payload) { on_sample(payload); }});
}

void Node::engage(const char *reason) {
  detail::Command command{.kind = detail::Command::Kind::ENGAGE, .input = {}, .reason = {}};
  core::copy_text(command.reason, reason != nullptr ? reason : "");
  if (task_ == nullptr) {
    return; // not started: is_active() is already true
  }
  if (xQueueSend(command_queue_, &command, 0) != pdTRUE) {
    engage_pending_.store(true); // the queue is full: engage anyway, without the reason
  }
  xTaskNotifyGive(task_);
}

void Node::set_input(const Input &input) {
  const detail::Command command{
      .kind = detail::Command::Kind::SET_INPUT, .input = input, .reason = {}};
  if (task_ == nullptr) {
    config_.input = input; // before start(): start() arms this one
    return;
  }
  if (xQueueSend(command_queue_, &command, portMAX_DELAY) == pdTRUE) {
    xTaskNotifyGive(task_);
  }
}

NodeStatus Node::status() const {
  NodeStatus status;
  if (status_queue_ == nullptr || xQueuePeek(status_queue_, &status, 0) != pdTRUE) {
    status.active = true; // not started: say active, as is_active() does
  }
  return status;
}

void Node::on_sample(std::span<const uint8_t> payload) {
  detail::Rx rx{};
  rx.rx = detail::now(); // receive time on the steady clock: what freshness means
  if (!core::decode(payload, config_.name, rammp::kEStopClearer, rx.msg)) {
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

void Node::send(Time now) {
  const auto out = logic_.next(now);
  const bool engaged = out.state == rammp::EStopState::ENGAGED;
  tx_.header.stamp = detail::stamp_now();
  tx_.state = out.state;
  tx_.reason = engaged ? reason_ : "";
  tx_.pub_seq = out.pub_seq;
  const std::size_t size = core::encode(tx_, tx_buf_);
  espp::RtpsParticipant *participant = participant_.load();
  if (size == 0) {
    status_.tx_errors++;
    return;
  }
  // Not attached yet is not an error: the next period resends.
  if (participant != nullptr &&
      !participant->publish(rammp::kEStop.name, std::span<const uint8_t>(tx_buf_.data(), size))) {
    status_.tx_errors++;
  }
}

void Node::handle(const detail::Command &command, Time) {
  switch (command.kind) {
  case detail::Command::Kind::ENGAGE:
    core::copy_text(reason_, command.reason);
    logic_.press();
    break;
  case detail::Command::Kind::SET_INPUT:
    status_.input = command.input;
    break;
  case detail::Command::Kind::CLEAR_ATTEMPT:
    break; // the latch's
  }
}

void Node::task_main(void *self) { static_cast<Node *>(self)->run(); }

void Node::run() {
  uint32_t edges_seen = 0;
  bool rearm = true;
  for (;;) {
    const Time before = detail::now();
    const Time next = logic_.next_send();
    (void)ulTaskNotifyTake(pdTRUE, detail::ticks_until(before, next));
    const Time now = detail::now();

    detail::Command command;
    while (xQueueReceive(command_queue_, &command, 0) == pdTRUE) {
      rearm |= command.kind == detail::Command::Kind::SET_INPUT;
      handle(command, now);
    }
    if (engage_pending_.exchange(false)) {
      core::copy_text(reason_, "engage");
      logic_.press();
    }

    // A new pin or contact, or arming failed last time (retried every period; until it
    // works is_active() holds). A switch already held on the new pin is a press: the edge
    // interrupt alone would never see it.
    const bool has_switch = status_.input.gpio_num != GPIO_NUM_NC;
    if (rearm || (has_switch && !input_.armed())) {
      rearm = false;
      if (!has_switch) {
        input_.disarm();
      } else if (input_.arm(status_.input, detail::EdgeInput::Edges::PRESS, task_) &&
                 input_.pressed()) {
        detail::switch_reason(reason_, status_.input.gpio_num);
        logic_.press();
      }
    }
    if (const uint32_t edges = input_.edges(); edges != edges_seen) {
      edges_seen = edges;
      detail::switch_reason(reason_, status_.input.gpio_num);
      logic_.press(); // engaging does not wait for the period
    }

    detail::Rx rx;
    for (UBaseType_t i = 0; i < detail::kRxQueueDepth && xQueueReceive(rx_queue_, &rx, 0) == pdTRUE;
         i++) {
      logic_.receive(rx.msg, rx.rx, now, input_.pressed());
    }

    if (logic_.send_due(now)) {
      send(now);
    }

    const bool armed = status_.input.gpio_num == GPIO_NUM_NC || input_.armed();
    status_.active = logic_.active(now, armed);
    status_.engaged = logic_.engaged();
    status_.latch_engaged = logic_.latch_engaged(now);
    status_.latch_fresh = logic_.latch_fresh(now);
    status_.armed = armed;
    status_.presses = logic_.counters().presses;
    status_.clear_attempts_accepted = logic_.counters().clear_attempts_accepted;
    status_.clear_attempts_refused = logic_.counters().clear_attempts_refused;
    status_.rx_errors = rx_dropped_.load(std::memory_order_relaxed);
    status_.pub_seq = logic_.pub_seq();
    core::copy_text(status_.latch_source, logic_.latch_source());
    core::copy_text(status_.latch_reason, logic_.latch_reason());
    active_.store(status_.active);
    (void)xQueueOverwrite(status_queue_, &status_);
  }
}

} // namespace estop
