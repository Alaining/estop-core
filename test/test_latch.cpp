// LatchLogic on its own, fed messages as the MIB decodes them: the scenarios of the MIB
// draft's host test, plus the event ring, the source cap and the local switch.

#include <chrono>
#include <cstring>
#include <string>
#include <string_view>

#include "check.hpp"
#include "estop/core/latch_logic.hpp"
#include "wire.hpp"

namespace {

using namespace std::chrono_literals;
using estop::core::LatchEvent;
using estop::core::LatchLogic;
using estop::core::Time;

constexpr std::string_view kHmi[] = {"hmi"};

estop::core::Decoded msg(bool engaged, std::string source, uint32_t pub_seq) {
  return estop::test::received(rammp::kEStopClearer, engaged, std::move(source), pub_seq, "", 0,
                               engaged ? "switch gpio2" : "");
}

bool cause_is(const LatchLogic &l, std::string_view want) { return l.cause() == want; }

/// Takes every event; true if one of `kind` has `text` in it.
bool saw(LatchLogic &l, LatchEvent::Kind kind, std::string_view text = {}) {
  bool found = false;
  LatchEvent e;
  while (l.take_event(e)) {
    found |= e.kind == kind && std::string_view(e.text).find(text) != std::string_view::npos;
  }
  return found;
}

void test_protocol() {
  Time t{1'000'000};
  LatchLogic l({.watched = kHmi});
  // Boot: latched; a clear attempt before the HMI is heard is refused.
  CHECK(l.latched() && cause_is(l, "boot"));
  CHECK(!l.start_clear_attempt(t));
  CHECK(saw(l, LatchEvent::Kind::CLEAR_ATTEMPT_REFUSED, "hmi: never heard"));
  // Heard (heartbeats), clear attempt: needs a NOT_ENGAGED newer than clear_pub_seq.
  l.receive(msg(false, "hmi", 1), t);
  l.receive(msg(false, "hmi", 2), t);
  CHECK(l.start_clear_attempt(t));
  CHECK(l.send_due(t)); // the attempt goes out at once
  auto out = l.next(t);
  CHECK(out.state == rammp::EStopState::NOT_ENGAGED && out.clear_target == "hmi" &&
        out.clear_pub_seq == 2 && out.reason == "clear attempt");
  l.tick(t);
  CHECK(l.latched()); // nothing newer than clear_pub_seq 2 yet
  t += 100ms;
  l.receive(msg(false, "hmi", 3), t);
  l.tick(t);
  CHECK(!l.latched() && l.counters().releases == 1);
  CHECK(saw(l, LatchEvent::Kind::RELEASED));
  out = l.next(t);
  CHECK(out.state == rammp::EStopState::NOT_ENGAGED && out.clear_target.empty());
  CHECK(!l.start_clear_attempt(t)); // nothing to clear
  // Press: ENGAGED latches at once, and the latch's ENGAGED is due at once, mid-period.
  t += 50ms;
  CHECK(!l.send_due(t));
  l.receive(msg(true, "hmi", 4), t);
  CHECK(l.latched() && cause_is(l, "hmi: switch gpio2") && l.send_due(t));
  out = l.next(t);
  CHECK(out.state == rammp::EStopState::ENGAGED && out.reason == "hmi: switch gpio2");
  // Clear attempt with clear_pub_seq fixed at 5; an ENGAGED already in flight (6) does
  // not release, nor end the attempt.
  t += 100ms;
  l.receive(msg(true, "hmi", 5), t);
  CHECK(l.start_clear_attempt(t));
  t += 50ms;
  l.receive(msg(true, "hmi", 6), t);
  l.tick(t);
  CHECK(l.latched() && l.clear_attempt_running());
  CHECK(l.next(t).clear_pub_seq == 5); // never updated mid-attempt
  t += 50ms;
  l.receive(msg(false, "hmi", 7), t);
  l.tick(t);
  CHECK(!l.latched());
  // A reordered older ENGAGED after release: engaging is never seq-checked (latches),
  // but the source's newest word (NOT_ENGAGED, 7) stands.
  l.receive(msg(true, "hmi", 6), t);
  CHECK(l.latched());
  // Clear window: the source keeps sending ENGAGED (pressed again): the attempt expires.
  t += 100ms;
  l.receive(msg(true, "hmi", 8), t);
  CHECK(l.start_clear_attempt(t));
  for (uint32_t i = 0; i < 12; i++) {
    t += 100ms;
    l.receive(msg(true, "hmi", 9 + i), t);
    l.tick(t);
  }
  CHECK(l.latched() && !l.clear_attempt_running());
  CHECK(saw(l, LatchEvent::Kind::CLEAR_ATTEMPT_EXPIRED, "clear window"));
  // Release, then the deadman: silence past the timeout latches.
  t += 100ms;
  l.receive(msg(false, "hmi", 30), t);
  CHECK(l.start_clear_attempt(t));
  t += 100ms;
  l.receive(msg(false, "hmi", 31), t);
  l.tick(t);
  CHECK(!l.latched());
  t += 300ms;
  l.tick(t);
  CHECK(!l.latched()); // exactly the timeout: not yet silent
  t += 1us;
  l.tick(t);
  CHECK(l.latched() && cause_is(l, "hmi: silent (deadman)"));
  // A rebooted HMI restarts at pub_seq 0: forgotten after the silence, accepted, clears.
  t += 2s;
  l.receive(msg(false, "hmi", 1), t);
  CHECK(l.start_clear_attempt(t));
  t += 100ms;
  l.receive(msg(false, "hmi", 2), t);
  l.tick(t);
  CHECK(!l.latched());
  // Another node's heartbeat is not a clear, and our own messages are ignored.
  t += 50ms;
  l.receive(msg(true, "pendant", 1), t);
  CHECK(l.latched());
  l.receive(msg(false, "pendant", 2), t);
  l.receive(msg(false, "hmi", 3), t);
  l.tick(t);
  CHECK(l.latched());
  l.receive(msg(false, "mib", 99), t);
  // Clear-attempt messages go out one target per period, in turn.
  CHECK(l.start_clear_attempt(t));
  const auto a = l.next(t);
  const auto b = l.next(t);
  const auto c = l.next(t);
  CHECK(a.clear_target != b.clear_target && a.clear_target == c.clear_target);
  t += 100ms;
  l.receive(msg(false, "pendant", 3), t);
  l.tick(t);
  CHECK(l.latched()); // one of two answered
  l.receive(msg(false, "hmi", 4), t);
  l.tick(t);
  CHECK(!l.latched());
  // Reordered during a clear attempt: the NOT_ENGAGED (newer) arrives before the
  // ENGAGED it overtook. The source's newest word is the NOT_ENGAGED, so it releases.
  t += 50ms;
  l.receive(msg(true, "hmi", 40), t);
  l.receive(msg(false, "pendant", 10), t);
  CHECK(l.start_clear_attempt(t)); // clear_pub_seq: hmi 40, pendant 10
  t += 50ms;
  l.receive(msg(false, "hmi", 42), t);
  l.receive(msg(true, "hmi", 41), t);
  l.receive(msg(false, "pendant", 11), t);
  l.tick(t);
  CHECK(!l.latched());
}

void test_unknown_state_engages() {
  Time t{1'000'000};
  LatchLogic l({.watched = kHmi});
  l.receive(msg(false, "hmi", 1), t);
  CHECK(l.start_clear_attempt(t));
  l.receive(msg(false, "hmi", 2), t);
  l.tick(t);
  CHECK(!l.latched());
  auto odd = estop::test::message(false, "hmi", "", 3);
  odd.state = static_cast<rammp::EStopState>(7);
  l.receive(estop::test::decode(estop::test::encode(odd), rammp::kEStopClearer), t);
  CHECK(l.latched());
}

void test_deadman_off() {
  Time t{1'000'000};
  LatchLogic quiet(
      {.watched = kHmi,
       .policy = {.period = 100ms, .timeout = 300ms, .deadman = false, .clear_window = 1000ms}});
  CHECK(quiet.start_clear_attempt(t)); // nothing heard, nothing to wait for
  quiet.tick(t);
  CHECK(!quiet.latched());
  t += 5s;
  quiet.tick(t);
  CHECK(!quiet.latched());
}

void test_engage_and_local_switch() {
  Time t{1'000'000};
  LatchLogic l({.watched = kHmi});
  l.receive(msg(false, "hmi", 1), t);
  CHECK(!l.start_clear_attempt(t, true)); // local switch held
  CHECK(saw(l, LatchEvent::Kind::CLEAR_ATTEMPT_REFUSED, "local switch held"));
  CHECK(l.start_clear_attempt(t));
  l.tick(t, true); // held during the attempt: it ends
  CHECK(!l.clear_attempt_running());
  CHECK(saw(l, LatchEvent::Kind::CLEAR_ATTEMPT_EXPIRED, "local switch held"));
  CHECK(l.start_clear_attempt(t));
  t += 100ms;
  l.receive(msg(false, "hmi", 2), t);
  l.tick(t);
  CHECK(!l.latched());
  (void)l.next(t);
  CHECK(!l.send_due(t));
  // The latch's own reason: latched at once, ENGAGED due at once, the cause says so.
  l.engage("switch gpio35");
  CHECK(l.latched() && cause_is(l, "mib: switch gpio35") && l.send_due(t));
  // engage() during an attempt ends it.
  CHECK(l.start_clear_attempt(t));
  l.engage("fault");
  CHECK(!l.clear_attempt_running());
  CHECK(saw(l, LatchEvent::Kind::CLEAR_ATTEMPT_EXPIRED, "engaged by this node"));
}

void test_release_needs_a_fresh_answer() {
  // An unwatched node's answer counts only while it is fresh: one heard and then silent
  // past the timeout does not release (the deadman would not notice, it is unwatched).
  Time t{1'000'000};
  LatchLogic l({.watched = kHmi});
  l.receive(msg(false, "hmi", 1), t);
  l.receive(msg(false, "pendant", 1), t);
  CHECK(l.start_clear_attempt(t));
  l.receive(msg(false, "pendant", 2), t);
  t += 400ms;
  l.receive(msg(false, "hmi", 2), t);
  l.tick(t);
  CHECK(l.latched());
  l.receive(msg(false, "pendant", 3), t);
  l.tick(t);
  CHECK(!l.latched());
}

void test_release_needs_a_newer_not_engaged() {
  // The HMI's newest word is already NOT_ENGAGED, but at clear_pub_seq, from before the
  // attempt: that must not release.
  Time t{1'000'000};
  LatchLogic l({.watched = kHmi});
  l.receive(msg(false, "hmi", 1), t);
  CHECK(l.start_clear_attempt(t));
  l.tick(t);
  CHECK(l.latched());
  l.receive(msg(false, "hmi", 1), t); // a duplicate
  l.tick(t);
  CHECK(l.latched());
}

void test_event_ring_drops_the_oldest() {
  Time t{1'000'000};
  LatchLogic l({.watched = kHmi});
  (void)l.start_clear_attempt(t, true); // the oldest: refused, switch held
  for (std::size_t i = 0; i < LatchLogic::kMaxEvents + 1; i++) {
    (void)l.start_clear_attempt(t); // refused: never heard, one event each
  }
  (void)l.start_clear_attempt(t, true); // the newest
  CHECK(l.counters().events_dropped == 3);
  std::size_t n = 0;
  std::size_t held = 0;
  LatchEvent e;
  while (l.take_event(e)) {
    n++;
    held += std::string_view(e.text) == "local switch held" ? 1 : 0;
  }
  CHECK(n == LatchLogic::kMaxEvents);
  CHECK(held == 1 && std::string_view(e.text) == "local switch held"); // only the newest
}

void test_source_cap() {
  Time t{1'000'000};
  LatchLogic l({.watched = kHmi});
  l.receive(msg(false, "hmi", 1), t);
  for (std::size_t i = 0; i < LatchLogic::kMaxSources + 4; i++) {
    l.receive(msg(false, "n" + std::to_string(i), 1), t);
  }
  CHECK(l.counters().sources_dropped == 5); // 16 slots: hmi and n0..n14
  // A source past the cap still latches on ENGAGED.
  CHECK(l.start_clear_attempt(t));
  t += 100ms;
  l.receive(msg(false, "hmi", 2), t);
  for (std::size_t i = 0; i < LatchLogic::kMaxSources - 1; i++) {
    l.receive(msg(false, "n" + std::to_string(i), 2), t);
  }
  l.tick(t);
  CHECK(!l.latched());
  l.receive(msg(true, "n99", 1), t);
  CHECK(l.latched() && cause_is(l, "n99: switch gpio2"));
}

void test_long_cause_truncates() {
  Time t{1'000'000};
  LatchLogic l({.watched = kHmi});
  l.engage(std::string(200, 'x'));
  CHECK(std::strlen(l.cause()) == 47);
  CHECK(l.next(t).reason.size() == 47);
}

} // namespace

int main() {
  test_protocol();
  test_unknown_state_engages();
  test_deadman_off();
  test_engage_and_local_switch();
  test_release_needs_a_fresh_answer();
  test_release_needs_a_newer_not_engaged();
  test_event_ring_drops_the_oldest();
  test_source_cap();
  test_long_cause_truncates();
  return estop::test::verdict("latch");
}
