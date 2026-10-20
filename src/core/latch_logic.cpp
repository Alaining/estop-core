#include "estop/core/latch_logic.hpp"

#include <algorithm>

namespace estop::core {

namespace {

constexpr Time to_time(std::chrono::milliseconds ms) {
  return std::chrono::duration_cast<Time>(ms);
}

/// Copies `a` + ": " + `b` (or `a` alone when `b` is empty), truncated.
template <std::size_t N> void join_text(char (&out)[N], std::string_view a, std::string_view b) {
  std::size_t n = 0;
  auto put = [&](std::string_view s) {
    for (char c : s) {
      if (n + 1 < N) {
        out[n++] = c;
      }
    }
  };
  put(a);
  if (!b.empty()) {
    put(": ");
    put(b);
  }
  out[n] = '\0';
}

} // namespace

LatchLogic::LatchLogic(const LatchConfig &config)
    : config_(config) {}

bool LatchLogic::silent(const Source &src, Time now) const {
  return !src.heard || now - src.last_rx > to_time(config_.policy.timeout);
}

void LatchLogic::event(LatchEvent::Kind kind, std::string_view a, std::string_view b) {
  if (events_count_ == kMaxEvents) {
    events_head_ = (events_head_ + 1) % kMaxEvents; // drop the oldest
    events_count_--;
    counters_.events_dropped++;
  }
  LatchEvent &e = events_[(events_head_ + events_count_) % kMaxEvents];
  e.kind = kind;
  join_text(e.text, a, b);
  events_count_++;
}

bool LatchLogic::take_event(LatchEvent &out) {
  if (events_count_ == 0) {
    return false;
  }
  out = events_[events_head_];
  events_head_ = (events_head_ + 1) % kMaxEvents;
  events_count_--;
  return true;
}

void LatchLogic::latch(std::string_view source, std::string_view reason) {
  char cause[sizeof(cause_)];
  join_text(cause, source, reason);
  if (!latched_ || std::string_view(cause_) != cause) {
    event(LatchEvent::Kind::LATCHED, cause);
  }
  if (!latched_) {
    send_now_ = true; // engaging does not wait for the period
  }
  latched_ = true;
  copy_text(cause_, cause);
}

void LatchLogic::cancel_attempt() {
  attempt_running_ = false;
  for (auto &[name, src] : sources_) {
    src.clear_pub_seq.reset();
  }
}

void LatchLogic::expire_attempt(std::string_view why, std::string_view who) {
  if (attempt_running_) {
    event(LatchEvent::Kind::CLEAR_ATTEMPT_EXPIRED, who.empty() ? why : who, who.empty() ? "" : why);
    cancel_attempt();
  }
}

LatchLogic::Source *LatchLogic::find_or_add(std::string_view name) {
  if (const auto it = sources_.find(name); it != sources_.end()) {
    return &it->second;
  }
  if (sources_.size() >= kMaxSources) {
    return nullptr;
  }
  return &sources_.emplace(std::string(name), Source{}).first->second;
}

void LatchLogic::receive(const Decoded &msg, Time rx) {
  if (msg.from_self) {
    return; // our own, looped back
  }
  // Anything but NOT_ENGAGED engages (the decoder's fail-safe reading of an unknown
  // value), and engaging is never age- or pub_seq-checked: any ENGAGED latches.
  // A clear attempt keeps running: an ENGAGED already in flight when it started is
  // followed by a newer NOT_ENGAGED once the node accepts it.
  if (msg.engaged) {
    latch(msg.source, msg.reason);
  }
  Source *src = find_or_add(msg.source);
  if (src == nullptr) {
    counters_.sources_dropped++;
    return;
  }
  if (silent(*src, rx)) {
    src->last_pub_seq.reset(); // silent past the timeout: a rebooted node restarts at 0
  }
  src->heard = true;
  src->last_rx = rx;
  // The source's word moves only on a newer message, so a late or repeated older one
  // cannot stand in for its newest.
  if (!src->last_pub_seq || rammp::estop_seq_newer(msg.pub_seq, *src->last_pub_seq)) {
    src->last_pub_seq = msg.pub_seq;
    src->engaged = msg.engaged;
  }
}

bool LatchLogic::start_clear_attempt(Time now, bool switch_held) {
  if (!latched_) {
    return false;
  }
  if (switch_held) {
    event(LatchEvent::Kind::CLEAR_ATTEMPT_REFUSED, "local switch held");
    return false;
  }
  if (config_.policy.deadman) {
    for (const std::string_view name : config_.watched) {
      const auto it = sources_.find(name);
      if (it == sources_.end() || !it->second.last_pub_seq) {
        event(LatchEvent::Kind::CLEAR_ATTEMPT_REFUSED, name, "never heard");
        return false;
      }
    }
  }
  // Every source's target is set (or cleared) here: a restart replaces a running attempt.
  for (auto &[name, src] : sources_) {
    src.clear_pub_seq = src.last_pub_seq;
  }
  attempt_running_ = true;
  attempt_started_ = now;
  attempt_turn_ = 0;
  counters_.clear_attempts++;
  send_now_ = true;
  event(LatchEvent::Kind::CLEAR_ATTEMPT_STARTED, {});
  return true;
}

void LatchLogic::engage(std::string_view reason) {
  expire_attempt("engaged by this node");
  latch(rammp::kEStopClearer, reason);
  send_now_ = true;
}

void LatchLogic::tick(Time now, bool switch_held) {
  if (switch_held) {
    expire_attempt("local switch held");
  }
  if (config_.policy.deadman) {
    for (const std::string_view name : config_.watched) {
      const auto it = sources_.find(name);
      if (it == sources_.end() || silent(it->second, now)) {
        expire_attempt("silent", name);
        latch(name, "silent (deadman)");
      }
    }
  }
  if (!attempt_running_) {
    return;
  }
  // Release: every target's newest message is NOT_ENGAGED, sent after the attempt
  // started (pub_seq > clear_pub_seq), and received recently.
  const bool released = std::all_of(sources_.begin(), sources_.end(), [&](const auto &entry) {
    const Source &src = entry.second;
    return !src.clear_pub_seq ||
           (src.last_pub_seq && !src.engaged &&
            rammp::estop_seq_newer(*src.last_pub_seq, *src.clear_pub_seq) && !silent(src, now));
  });
  if (released) {
    cancel_attempt();
    event(LatchEvent::Kind::RELEASED, {});
    latched_ = false;
    cause_[0] = '\0';
    counters_.releases++;
    send_now_ = true;
  } else if (now - attempt_started_ > to_time(config_.policy.clear_window)) {
    expire_attempt("clear window passed");
  }
}

LatchOut LatchLogic::next(Time now) {
  send_now_ = false;
  next_send_ = now + to_time(config_.policy.period);
  LatchOut out{.state = latched_ ? rammp::EStopState::ENGAGED : rammp::EStopState::NOT_ENGAGED,
               .pub_seq = ++pub_seq_,
               .reason = latched_ ? std::string_view(cause_) : std::string_view(),
               .clear_target = {},
               .clear_pub_seq = 0};
  if (!attempt_running_) {
    return out;
  }
  // One clear-attempt message per period, the targets in turn.
  std::size_t targets = 0;
  for (const auto &[name, src] : sources_) {
    targets += src.clear_pub_seq ? 1 : 0;
  }
  if (targets == 0) {
    return out;
  }
  std::size_t want = attempt_turn_++ % targets;
  for (const auto &[name, src] : sources_) {
    if (src.clear_pub_seq && want-- == 0) {
      out.state = rammp::EStopState::NOT_ENGAGED;
      out.reason = "clear attempt";
      out.clear_target = name;
      out.clear_pub_seq = *src.clear_pub_seq;
      break;
    }
  }
  return out;
}

} // namespace estop::core
