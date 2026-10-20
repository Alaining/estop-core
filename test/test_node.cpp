// NodeLogic on its own: the send cadence, engaged_pub_seq, and every condition on a
// clear attempt. The protocol against a real latch is in test_race.cpp.

#include <chrono>
#include <cstring>

#include "check.hpp"
#include "estop/core/node_logic.hpp"
#include "wire.hpp"

namespace {

using namespace std::chrono_literals;
using estop::core::NodeLogic;
using estop::core::Time;
using estop::test::received;

constexpr Time P = 100ms;
constexpr Time T = 300ms;

estop::core::Decoded clear_attempt(uint32_t clear_pub_seq, std::string target = "hmi",
                                   std::string source = "mib") {
  return received("hmi", false, std::move(source), 50, std::move(target), clear_pub_seq);
}

/// Pressed and sent once: engaged_pub_seq is 1.
NodeLogic engaged_node(Time now) {
  NodeLogic n({.name = "hmi"});
  n.press();
  (void)n.next(now);
  return n;
}

void test_cadence() {
  NodeLogic n({.name = "hmi"});
  Time now{0};
  CHECK(n.send_due(now)); // the first heartbeat goes out at once
  auto out = n.next(now);
  CHECK(out.state == rammp::EStopState::NOT_ENGAGED && out.pub_seq == 1);
  CHECK(n.next_send() == now + P);
  CHECK(!n.send_due(now + P - 1us));
  CHECK(n.send_due(now + P));
  out = n.next(now + P);
  CHECK(out.pub_seq == 2 && n.engaged_pub_seq() == 0);
}

void test_press_sends_at_once_and_sets_engaged_pub_seq() {
  NodeLogic n({.name = "hmi"});
  (void)n.next(Time{0});
  n.press();
  CHECK(n.engaged() && n.send_due(Time{1}) && n.counters().presses == 1);
  auto out = n.next(Time{1});
  CHECK(out.state == rammp::EStopState::ENGAGED && out.pub_seq == 2);
  CHECK(n.engaged_pub_seq() == 2);
  // Further periods keep ENGAGED going out, engaged_pub_seq stays at the first one.
  out = n.next(Time{1} + P);
  CHECK(out.state == rammp::EStopState::ENGAGED && out.pub_seq == 3 && n.engaged_pub_seq() == 2);
  // A second press moves it to the next send.
  n.press();
  (void)n.next(Time{1} + P + 1us);
  CHECK(n.engaged_pub_seq() == 4 && n.counters().presses == 2);
}

void test_clear_attempt_accepted() {
  const Time now{1'000'000};
  NodeLogic n = engaged_node(now);
  n.receive(clear_attempt(1), now, now, false);
  CHECK(!n.engaged() && n.counters().clear_attempts_accepted == 1);
  CHECK(n.send_due(now)); // NOT_ENGAGED at once
  CHECK(n.next(now).state == rammp::EStopState::NOT_ENGAGED);
  // Repeats of the same attempt after that are not counted either way.
  n.receive(clear_attempt(1), now, now, false);
  CHECK(n.counters().clear_attempts_accepted == 1 && n.counters().clear_attempts_refused == 0);
}

void test_clear_attempt_covering_later_pub_seq_accepted() {
  const Time now{1'000'000};
  NodeLogic n = engaged_node(now);
  (void)n.next(now + P); // pub_seq 2: clear_pub_seq 2 covers engaged_pub_seq 1
  n.receive(clear_attempt(2), now + P, now + P, false);
  CHECK(!n.engaged());
}

void test_clear_attempt_refusals() {
  const Time now{1'000'000};
  {
    NodeLogic n = engaged_node(now);              // engaged_pub_seq 1
    n.receive(clear_attempt(0), now, now, false); // older than the press
    CHECK(n.engaged() && n.counters().clear_attempts_refused == 1);
  }
  {
    NodeLogic n = engaged_node(now);
    n.receive(clear_attempt(1), now, now + P + 1us, false); // received over a period ago
    CHECK(n.engaged() && n.counters().clear_attempts_refused == 1);
    n.receive(clear_attempt(1), now, now + P, false); // exactly a period: still fresh
    CHECK(!n.engaged());
  }
  {
    NodeLogic n = engaged_node(now);
    n.receive(clear_attempt(1), now, now, true); // switch held
    CHECK(n.engaged() && n.counters().clear_attempts_refused == 1);
  }
  {
    NodeLogic n = engaged_node(now);
    n.press(); // pressed again, not sent yet: any attempt is older than it
    n.receive(clear_attempt(1000), now, now, false);
    CHECK(n.engaged() && n.counters().clear_attempts_refused == 1);
  }
  {
    NodeLogic n = engaged_node(now);
    n.receive(clear_attempt(1, "pendant"), now, now, false);                // for another node
    n.receive(clear_attempt(1, "hmi", "rogue"), now, now, false);           // not from the clearer
    n.receive(received("hmi", false, "hmi", 9, "hmi", 1), now, now, false); // our own
    CHECK(n.engaged());
    CHECK(n.counters().clear_attempts_refused == 0 && n.counters().clear_attempts_accepted == 0);
  }
}

void test_clear_attempt_wraps() {
  const Time now{1'000'000};
  NodeLogic n({.name = "hmi", .first_pub_seq = 0xFFFFFFFE});
  n.press();
  (void)n.next(now);                                    // engaged_pub_seq 0xFFFFFFFF
  (void)n.next(now + P);                                // pub_seq 0
  n.receive(clear_attempt(0), now + P, now + P, false); // 0 is after 0xFFFFFFFF
  CHECK(!n.engaged());
}

void test_latch_view() {
  const Time now{1'000'000};
  NodeLogic n({.name = "hmi"});
  CHECK(!n.latch_fresh(now) && !n.latch_engaged(now));
  n.receive(received("hmi", true, "mib", 1, "", 0, "hmi: switch gpio2"), now, now, false);
  CHECK(n.latch_fresh(now) && n.latch_engaged(now));
  CHECK(std::strcmp(n.latch_source(), "mib") == 0);
  CHECK(std::strcmp(n.latch_reason(), "hmi: switch gpio2") == 0);
  CHECK(n.latch_engaged(now + T) && !n.latch_engaged(now + T + 1us)); // stale latch: not fresh
  n.receive(received("hmi", false, "mib", 2), now + P, now + P, false);
  CHECK(n.latch_fresh(now + P) && !n.latch_engaged(now + P));
  // Another node's ENGAGED is the latch's business, not this view's.
  n.receive(received("hmi", true, "pendant", 1), now + P, now + P, false);
  CHECK(!n.latch_engaged(now + P));
}

void test_active() {
  const Time now{1'000'000};
  NodeLogic n({.name = "hmi"});
  CHECK(!n.active(now, true));
  CHECK(n.active(now, false)); // the switch input is not armed
  n.receive(received("hmi", true, "mib", 1), now, now, false);
  CHECK(n.active(now, true));            // the latch says ENGAGED
  CHECK(!n.active(now + T + 1us, true)); // ...but that word is stale
  n.press();
  CHECK(n.active(now + T + 1us, true));

  NodeLogic strict({.name = "hmi", .require_latch = true});
  CHECK(strict.active(now, true)); // the latch never heard
  strict.receive(received("hmi", false, "mib", 1), now, now, false);
  CHECK(!strict.active(now, true));
  CHECK(strict.active(now + T + 1us, true)); // the latch went silent
}

} // namespace

int main() {
  test_cadence();
  test_press_sends_at_once_and_sets_engaged_pub_seq();
  test_clear_attempt_accepted();
  test_clear_attempt_covering_later_pub_seq_accepted();
  test_clear_attempt_refusals();
  test_clear_attempt_wraps();
  test_latch_view();
  test_active();
  return estop::test::verdict("node");
}
