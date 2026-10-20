// The protocol end to end without a network: the real NodeLogic (the HMI) against the
// real LatchLogic (the MIB), every message through the real codec, delivery controlled
// by the test so in-flight, lost and reordered messages can be forced. The scenarios of
// pace-hmi-fw test/estop_latch_test.py, one function each.

#include <array>
#include <chrono>
#include <span>
#include <string>

#include "check.hpp"
#include "estop/core/latch_logic.hpp"
#include "estop/core/node_logic.hpp"
#include "wire.hpp"

namespace {

using namespace std::chrono_literals;
using estop::core::LatchEvent;
using estop::core::LatchLogic;
using estop::core::NodeLogic;
using estop::core::Time;
using estop::test::Bytes;

constexpr Time P = 100ms;
constexpr std::string_view kHmi[] = {"hmi"};
constexpr rammp::EStopPolicy kNoDeadman{
    .period = 100ms, .timeout = 300ms, .deadman = false, .clear_window = 1000ms};

/// A node with a switch level, sending through the codec.
struct Hmi {
  explicit Hmi(const char *name = "hmi", uint32_t first_pub_seq = 0)
      : name(name)
      , logic({.name = name, .first_pub_seq = first_pub_seq}) {}

  Bytes send(Time now) {
    const auto out = logic.next(now);
    const bool engaged = out.state == rammp::EStopState::ENGAGED;
    return estop::test::encode(
        estop::test::message(engaged, name, engaged ? "switch gpio2" : "", out.pub_seq));
  }

  /// True if this took a clear attempt.
  bool receive(const Bytes &bytes, Time now, Time age = Time{0}) {
    const uint32_t before = logic.counters().clear_attempts_accepted;
    logic.receive(estop::test::decode(bytes, name), now - age, now, held);
    return logic.counters().clear_attempts_accepted != before;
  }

  const char *name;
  NodeLogic logic;
  bool held = false;
};

Bytes latch_send(LatchLogic &latch, Time now) {
  const auto out = latch.next(now);
  return estop::test::encode(rammp::EStop{.header = {},
                                          .state = out.state,
                                          .source = rammp::kEStopClearer,
                                          .reason = std::string(out.reason),
                                          .pub_seq = out.pub_seq,
                                          .clear_target = std::string(out.clear_target),
                                          .clear_pub_seq = out.clear_pub_seq});
}

void deliver(LatchLogic &latch, const Bytes &bytes, Time now) {
  latch.receive(estop::test::decode(bytes, rammp::kEStopClearer), now);
}

bool saw(LatchLogic &l, LatchEvent::Kind kind, std::string_view text = {}) {
  bool found = false;
  LatchEvent e;
  while (l.take_event(e)) {
    found |= e.kind == kind && std::string_view(e.text).find(text) != std::string_view::npos;
  }
  return found;
}

/// One HMI and the MIB, stepping a period at a time with everything delivered.
struct Bench {
  explicit Bench(rammp::EStopPolicy policy = rammp::kEStopPolicy, uint32_t first_pub_seq = 0)
      : latch({.watched = kHmi, .policy = policy})
      , hmi("hmi", first_pub_seq) {}

  void step(bool hmi_to_mib = true, bool mib_to_hmi = true) {
    now += P;
    const Bytes out = hmi.send(now);
    if (hmi_to_mib) {
      deliver(latch, out, now);
    }
    latch.tick(now);
    const Bytes back = latch_send(latch, now);
    if (mib_to_hmi) {
      (void)hmi.receive(back, now);
    }
  }

  void run(int periods, bool hmi_to_mib = true, bool mib_to_hmi = true) {
    for (int i = 0; i < periods; i++) {
      step(hmi_to_mib, mib_to_hmi);
    }
  }

  void clear() { CHECK(latch.start_clear_attempt(now)); }

  /// Heard from, cleared, released: the starting point of most scenarios.
  void boot() {
    run(1);
    clear();
    run(3);
    CHECK(!latch.latched());
  }

  Time now{1'000'000};
  LatchLogic latch;
  Hmi hmi;
};

constexpr int periods(std::chrono::milliseconds span) { return static_cast<int>(span / 100ms); }

void boot_is_latched_until_cleared() {
  Bench b;
  CHECK(!b.latch.start_clear_attempt(b.now)); // never heard from the HMI
  b.run(5);
  CHECK(b.latch.latched()); // nothing releases by itself
  b.clear();
  b.run(3);
  CHECK(!b.latch.latched());
}

void press_latches_and_holds_after_release_of_the_switch() {
  Bench b;
  b.boot();
  b.hmi.logic.press();
  b.run(1);
  CHECK(b.latch.latched());
  b.run(50); // a momentary switch: up long ago, the HMI keeps sending ENGAGED
  CHECK(b.hmi.logic.engaged() && b.latch.latched());
}

void clear_releases_through_the_handshake() {
  Bench b;
  b.boot();
  b.hmi.logic.press();
  b.run(3);
  b.clear();
  b.run(3);
  CHECK(!b.hmi.logic.engaged() && !b.latch.latched());
  CHECK(saw(b.latch, LatchEvent::Kind::RELEASED));
}

void engaged_in_flight_after_the_clear_attempt_does_not_release_or_confuse() {
  Bench b;
  b.boot();
  b.hmi.logic.press();
  b.run(2);
  b.clear();
  const Bytes attempt = latch_send(b.latch, b.now); // held back
  const Bytes late = b.hmi.send(b.now);             // already in flight, newer than clear_pub_seq
  deliver(b.latch, late, b.now + 10ms);
  b.latch.tick(b.now + 10ms);
  CHECK(b.latch.latched()); // still waiting for a NOT_ENGAGED > clear_pub_seq
  CHECK(b.hmi.receive(attempt, b.now + 10ms, 10ms));
  deliver(b.latch, b.hmi.send(b.now + 20ms), b.now + 20ms);
  b.latch.tick(b.now + 20ms);
  CHECK(!b.latch.latched());
  // The late ENGAGED again (a duplicate) after release: engaging is never seq-checked,
  // so it latches. Safe, if a nuisance.
  deliver(b.latch, late, b.now + 30ms);
  CHECK(b.latch.latched());
}

void press_again_while_the_clear_attempt_is_in_flight_is_refused() {
  Bench b;
  b.boot();
  b.hmi.logic.press();
  b.run(2);
  b.clear();
  const Bytes attempt = latch_send(b.latch, b.now);
  b.hmi.logic.press(); // pressed again before the attempt arrives
  deliver(b.latch, b.hmi.send(b.now), b.now);
  CHECK(!b.hmi.receive(attempt, b.now));
  // The MIB keeps resending its attempt (old clear_pub_seq); the HMI refuses; it expires.
  b.run(periods(rammp::kEStopPolicy.clear_window) + 3);
  CHECK(b.hmi.logic.engaged() && b.latch.latched());
  CHECK(saw(b.latch, LatchEvent::Kind::CLEAR_ATTEMPT_EXPIRED, "clear window"));
  b.clear(); // a fresh attempt covers the new press
  b.run(3);
  CHECK(!b.latch.latched());
}

void old_clear_attempt_arriving_later_is_refused() {
  Bench b;
  b.boot();
  b.hmi.logic.press();
  b.run(2);
  b.clear();
  const Bytes old_attempt = latch_send(b.latch, b.now);
  b.run(3); // accepted and released normally
  CHECK(!b.latch.latched());
  b.hmi.logic.press();
  b.run(2);
  CHECK(!b.hmi.receive(old_attempt, b.now)); // clear_pub_seq below the new engaged_pub_seq
  CHECK(b.hmi.logic.engaged());
}

void stale_clear_attempt_is_refused_by_receive_age() {
  Hmi hmi;
  const Time now{1'000'000};
  hmi.logic.press();
  (void)hmi.send(now);
  const Bytes attempt = estop::test::encode(
      estop::test::message(false, "mib", "clear attempt", 9, "hmi", hmi.logic.pub_seq()));
  CHECK(!hmi.receive(attempt, now, 2 * P));
  CHECK(hmi.receive(attempt, now));
}

void held_switch_refuses_the_clear_attempt() {
  Hmi hmi;
  const Time now{1'000'000};
  hmi.logic.press();
  (void)hmi.send(now);
  hmi.held = true;
  const Bytes attempt = estop::test::encode(
      estop::test::message(false, "mib", "clear attempt", 9, "hmi", hmi.logic.pub_seq()));
  CHECK(!hmi.receive(attempt, now));
}

void another_nodes_heartbeat_is_not_a_clear_attempt() {
  Bench b;
  b.boot();
  b.hmi.logic.press();
  b.run(2);
  Hmi other("pendant");
  for (int i = 0; i < 5; i++) {
    deliver(b.latch, other.send(b.now), b.now);
    CHECK(!b.hmi.receive(other.send(b.now), b.now)); // not from the clearer
  }
  b.latch.tick(b.now);
  CHECK(b.latch.latched() && b.hmi.logic.engaged());
}

void clear_attempt_from_a_non_clearer_is_ignored() {
  Hmi hmi;
  const Time now{1'000'000};
  hmi.logic.press();
  (void)hmi.send(now);
  const Bytes rogue = estop::test::encode(
      estop::test::message(false, "rogue", "clear attempt", 1, "hmi", hmi.logic.pub_seq()));
  CHECK(!hmi.receive(rogue, now));
}

void lost_clear_attempts_and_not_engaged_are_resent() {
  Bench b;
  b.boot();
  b.hmi.logic.press();
  b.run(2);
  b.clear();
  b.step(true, false); // the attempt is lost
  b.step(false, true); // the attempt arrives; the HMI's NOT_ENGAGED is lost
  CHECK(b.latch.latched());
  b.run(2); // the next NOT_ENGAGED gets through
  CHECK(!b.latch.latched());
}

void link_drop_trips_the_deadman_and_blocks_release() {
  Bench b;
  b.boot();
  b.run(periods(rammp::kEStopPolicy.timeout) + 2, false);
  CHECK(b.latch.latched());
  CHECK(std::string_view(b.latch.cause()).find("silent") != std::string_view::npos);
  (void)b.latch.start_clear_attempt(b.now); // may start...
  b.run(periods(rammp::kEStopPolicy.clear_window) + 2, false);
  CHECK(b.latch.latched()); // ...but cannot finish while silent
}

void hmi_reboot_restarts_pub_seq_and_still_clears() {
  Bench b(rammp::kEStopPolicy, 5000);
  b.boot();
  b.now += 3s; // rebooting: silence far beyond the timeout
  b.latch.tick(b.now);
  CHECK(b.latch.latched());
  b.hmi.logic = NodeLogic({.name = "hmi"}); // pub_seq from 0 again
  b.run(3);
  b.clear(); // clear_pub_seq is 3, not 5000-something: the old pub_seq was forgotten
  const auto out = b.latch.next(b.now);
  CHECK(out.clear_target == "hmi" && out.clear_pub_seq == 3);
  b.run(3);
  CHECK(!b.latch.latched());
}

void deadman_off_ignores_silence() {
  Bench b(kNoDeadman);
  b.boot();
  b.run(20, false);
  CHECK(!b.latch.latched());
  b.hmi.logic.press();
  b.run(1);
  CHECK(b.latch.latched());
}

void ros_layout_engages_but_never_clears() {
  // The 4-field ROS layout: what a node that knows only the ROS message sends.
  struct RosEStop {
    rammp::Header header;
    bool engaged;
    std::string source;
    std::string reason;
  };
  Bench b;
  b.boot();
  std::array<std::byte, 128> buf{};
  auto ros = [&](bool engaged, std::string source) {
    const auto size = cdr::serialize_into<cdr::xcdr1>(
        RosEStop{.header = {}, .engaged = engaged, .source = std::move(source), .reason = "x"},
        std::span(buf));
    CHECK(size.has_value());
    Bytes out(*size);
    for (std::size_t i = 0; i < out.size(); i++) {
      out[i] = static_cast<uint8_t>(buf[i]);
    }
    return out;
  };
  const Bytes ros_engaged = ros(true, "ros_node");
  deliver(b.latch, ros_engaged, b.now);
  CHECK(b.latch.latched());
  CHECK(estop::test::decode(ros_engaged, "mib").pub_seq == 0);
  CHECK(!estop::test::decode(ros(false, "mib"), "hmi").clear_for_self);
}

void release_needs_a_not_engaged_sent_after_the_decision() {
  // The HMI was never engaged (the MIB latched on its own), so its newest word is already
  // NOT_ENGAGED -- but at clear_pub_seq, from before the attempt. That must not release:
  // only a NOT_ENGAGED newer than clear_pub_seq proves the HMI answered.
  Bench b;
  b.boot();
  b.latch.engage("bench");
  b.clear();
  b.latch.tick(b.now);
  CHECK(b.latch.latched());
  b.run(1);
  CHECK(!b.latch.latched());
}

void reordered_older_message_does_not_overwrite_the_newest() {
  Bench b;
  b.boot();
  b.hmi.logic.press();
  b.run(2);
  b.clear();
  const Bytes attempt = latch_send(b.latch, b.now);
  const Bytes old_engaged = b.hmi.send(b.now); // sent before the attempt arrived...
  CHECK(b.hmi.receive(attempt, b.now));
  const Bytes new_not_engaged = b.hmi.send(b.now);
  deliver(b.latch, new_not_engaged, b.now); // ...but delivered after the NOT_ENGAGED
  deliver(b.latch, old_engaged, b.now);
  b.latch.tick(b.now);
  CHECK(!b.latch.latched()); // the newest word stands
}

void pub_seq_wraps() {
  Bench b(rammp::kEStopPolicy, 0xFFFFFFFF - 3);
  b.boot();
  b.hmi.logic.press();
  b.run(4); // crosses the wrap while engaged
  b.clear();
  b.run(3);
  CHECK(!b.latch.latched());
  CHECK(rammp::estop_seq_newer(1, 0xFFFFFFFF));
}

void two_nodes_both_must_answer() {
  // Beyond the Python suite: two watched nodes; the stop ends only when both answer.
  constexpr std::string_view watched[] = {"hmi", "pendant"};
  LatchLogic latch({.watched = watched});
  Hmi hmi("hmi");
  Hmi pendant("pendant");
  Time now{1'000'000};
  auto step = [&](bool pendant_to_mib) {
    now += P;
    deliver(latch, hmi.send(now), now);
    const Bytes p = pendant.send(now);
    if (pendant_to_mib) {
      deliver(latch, p, now);
    }
    latch.tick(now);
    const Bytes back = latch_send(latch, now);
    (void)hmi.receive(back, now);
    (void)pendant.receive(back, now);
  };
  step(true);
  pendant.logic.press();
  step(true);
  hmi.logic.press();
  step(true);
  CHECK(latch.start_clear_attempt(now));
  for (int i = 0; i < 6; i++) {
    step(true);
  }
  CHECK(!latch.latched() && !hmi.logic.engaged() && !pendant.logic.engaged());
  // The pendant goes silent: the deadman latches, and nothing releases without it.
  for (int i = 0; i < 5; i++) {
    step(false);
  }
  CHECK(latch.latched());
  CHECK(latch.start_clear_attempt(now)); // its last_pub_seq was kept until it is heard again
  for (int i = 0; i < 12; i++) {
    step(false);
  }
  CHECK(latch.latched());
}

} // namespace

int main() {
  boot_is_latched_until_cleared();
  press_latches_and_holds_after_release_of_the_switch();
  clear_releases_through_the_handshake();
  engaged_in_flight_after_the_clear_attempt_does_not_release_or_confuse();
  press_again_while_the_clear_attempt_is_in_flight_is_refused();
  old_clear_attempt_arriving_later_is_refused();
  stale_clear_attempt_is_refused_by_receive_age();
  held_switch_refuses_the_clear_attempt();
  another_nodes_heartbeat_is_not_a_clear_attempt();
  clear_attempt_from_a_non_clearer_is_ignored();
  lost_clear_attempts_and_not_engaged_are_resent();
  link_drop_trips_the_deadman_and_blocks_release();
  hmi_reboot_restarts_pub_seq_and_still_clears();
  deadman_off_ignores_silence();
  ros_layout_engages_but_never_clears();
  release_needs_a_not_engaged_sent_after_the_decision();
  reordered_older_message_does_not_overwrite_the_newest();
  pub_seq_wraps();
  two_nodes_both_must_answer();
  return estop::test::verdict("race");
}
