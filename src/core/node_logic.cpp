#include "estop/core/node_logic.hpp"

namespace estop::core {

namespace {

constexpr Time to_time(std::chrono::milliseconds ms) {
  return std::chrono::duration_cast<Time>(ms);
}

} // namespace

NodeLogic::NodeLogic(const NodeConfig &config)
    : config_(config)
    , pub_seq_(config.first_pub_seq) {}

void NodeLogic::press() {
  engaged_ = true;
  press_unsent_ = true; // a new engaged_pub_seq, so a clear attempt already in flight fails
  counters_.presses++;
}

void NodeLogic::receive(const Decoded &msg, Time rx, Time now, bool switch_held) {
  if (msg.from_self || !msg.from_clearer) {
    return; // our own sample, or another node's: the latch speaks for them
  }
  latch_heard_ = true;
  latch_last_rx_ = rx;
  latch_says_engaged_ = msg.engaged;
  if (msg.engaged) {
    copy_text(latch_source_, msg.source);
    copy_text(latch_reason_, msg.reason);
  } else if (msg.clear_for_self) {
    receive_clear_attempt(msg, rx, now, switch_held);
  }
}

void NodeLogic::receive_clear_attempt(const Decoded &msg, Time rx, Time now, bool switch_held) {
  if (!engaged_) {
    return; // the latch resends its attempt until it sees our NOT_ENGAGED
  }
  const bool fresh = now - rx <= to_time(config_.policy.period);
  const bool covers =
      !press_unsent_ && rammp::estop_seq_at_least(msg.clear_pub_seq, engaged_pub_seq_);
  if (!fresh || !covers || switch_held) {
    counters_.clear_attempts_refused++;
    return;
  }
  engaged_ = false;
  counters_.clear_attempts_accepted++;
  next_send_ = now; // NOT_ENGAGED at once: it is what lets the latch release the stop
}

bool NodeLogic::send_due(Time now) const { return press_unsent_ || now >= next_send_; }

NodeOut NodeLogic::next(Time now) {
  pub_seq_++;
  if (press_unsent_) { // only press() sets it, and press() engages
    engaged_pub_seq_ = pub_seq_;
    press_unsent_ = false;
  }
  next_send_ = now + to_time(config_.policy.period);
  return {engaged_ ? rammp::EStopState::ENGAGED : rammp::EStopState::NOT_ENGAGED, pub_seq_};
}

bool NodeLogic::latch_fresh(Time now) const {
  return latch_heard_ && now - latch_last_rx_ <= to_time(config_.policy.timeout);
}

bool NodeLogic::latch_engaged(Time now) const { return latch_fresh(now) && latch_says_engaged_; }

bool NodeLogic::active(Time now, bool input_armed) const {
  return engaged_ || latch_engaged(now) || !input_armed ||
         (config_.require_latch && !latch_fresh(now));
}

} // namespace estop::core
