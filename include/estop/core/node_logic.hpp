#pragma once

/**
 * @file node_logic.hpp
 * @brief The rules of a node that can request the stop (the HMI, other boards), with no
 *        I/O and no clock of its own: the caller passes the time and the switch level.
 *
 * NOT_ENGAGED: sends state=NOT_ENGAGED every period (the heartbeat the latch's deadman
 * watches). ENGAGED: sends state=ENGAGED every period, switch released or not, until it
 * accepts a clear attempt. A press (or engage()) moves it to ENGAGED and sends at once;
 * engaged_pub_seq is the pub_seq of its first ENGAGED after the latest press, so a clear
 * attempt that predates a press is refused.
 *
 * A clear attempt is accepted only if it comes from rammp::kEStopClearer, is addressed to
 * this node, covers the latest press (clear_pub_seq >= engaged_pub_seq), arrived within
 * one period (receive time), and the switch is not held.
 *
 * Single-threaded: the owner (estop::Node's task) is the only caller. No allocation.
 */

#include <chrono>
#include <cstdint>
#include <string_view>

#include "estop/core/codec.hpp"
#include "messages/estop_message.hpp"

namespace estop::core {

/// Steady time since boot, from the caller's clock.
using Time = std::chrono::microseconds;

struct NodeConfig {
  std::string_view name; ///< EStop.source; 15 characters or fewer; must outlive the node
  rammp::EStopPolicy policy = rammp::kEStopPolicy;
  bool require_latch = false; ///< no e-stop message from the latch within the timeout = stop
  uint32_t first_pub_seq = 0; ///< pub_seq before the first message; 0 in firmware, set by tests
};

/// One message to send: the fields that change; the caller fills the rest.
struct NodeOut {
  rammp::EStopState state;
  uint32_t pub_seq;
};

struct NodeCounters {
  uint32_t presses = 0;                 ///< press() calls (edges and engage())
  uint32_t clear_attempts_accepted = 0; ///< clear attempts this node accepted
  uint32_t clear_attempts_refused = 0;  ///< stale, switch held, or older than the press
};

class NodeLogic {
public:
  explicit NodeLogic(const NodeConfig &config);

  /// A switch edge or engage(): ENGAGED, with a new engaged_pub_seq at the next send,
  /// which is due at once.
  void press();

  /// A received e-stop message, already decoded and not our own (from_self is ignored
  /// anyway). `rx` is its receive time, `now` the time it is handled; `switch_held` is
  /// the input's level now, only consulted for a clear attempt.
  void receive(const Decoded &msg, Time rx, Time now, bool switch_held);

  /// Whether a message is due: a press not yet sent, or a period since the last one.
  bool send_due(Time now) const;

  /// The message to send now. Advances pub_seq and the next send time.
  NodeOut next(Time now);

  /// When the next message is due, for the caller's wait.
  Time next_send() const { return next_send_; }

  bool engaged() const { return engaged_; }      ///< this node asks for the stop
  bool latch_fresh(Time now) const;              ///< latch heard within the timeout
  bool latch_engaged(Time now) const;            ///< latch fresh and saying ENGAGED
  bool active(Time now, bool input_armed) const; ///< do not drive (any cause)
  const char *latch_source() const { return latch_source_; }
  const char *latch_reason() const { return latch_reason_; }
  uint32_t pub_seq() const { return pub_seq_; }
  uint32_t engaged_pub_seq() const { return engaged_pub_seq_; }
  const NodeCounters &counters() const { return counters_; }

private:
  void receive_clear_attempt(const Decoded &msg, Time rx, Time now, bool switch_held);

  NodeConfig config_;
  bool engaged_ = false;
  bool press_unsent_ = false; ///< a press whose first ENGAGED has not gone out yet
  uint32_t pub_seq_ = 0;
  uint32_t engaged_pub_seq_ = 0;
  Time next_send_{0};
  bool latch_heard_ = false;
  Time latch_last_rx_{0};
  bool latch_says_engaged_ = false;
  char latch_source_[16] = {};
  char latch_reason_[48] = {};
  NodeCounters counters_;
};

} // namespace estop::core
