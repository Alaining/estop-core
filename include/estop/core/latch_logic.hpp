#pragma once

/**
 * @file latch_logic.hpp
 * @brief The rules of the node that owns the latch (rammp::kEStopClearer, the MIB), with
 *        no I/O and no clock of its own: the caller passes the time.
 *
 *  - Latched at start: nothing has been heard, nothing is known safe.
 *  - Latched while any source's newest message is ENGAGED, or (deadman) any watched
 *    source has been silent longer than policy.timeout. Engaging is never age- or
 *    pub_seq-checked: any ENGAGED latches.
 *  - A clear attempt fixes clear_pub_seq to each source's last_pub_seq and sends it,
 *    one target per period, with that same value. The stop is released only once every
 *    target's newest message is NOT_ENGAGED with pub_seq > clear_pub_seq, received within
 *    the timeout; otherwise the attempt expires after policy.clear_window.
 *  - A source silent longer than the timeout is forgotten (a rebooted node restarts its
 *    pub_seq at 0).
 *
 * Times are receive times on the caller's steady clock: a message's stamp is never used.
 * Single-threaded: the owner (estop::Latch's task) is the only caller. The source table
 * allocates once per new source name, up to kMaxSources; nothing else allocates.
 */

#include <cstddef>
#include <cstdint>
#include <functional>
#include <map>
#include <optional>
#include <span>
#include <string>
#include <string_view>

#include "estop/core/codec.hpp"
#include "estop/core/node_logic.hpp"
#include "messages/estop_message.hpp"

namespace estop::core {

struct LatchConfig {
  /// Sources the deadman expects to hear. The names must outlive the latch.
  std::span<const std::string_view> watched;
  rammp::EStopPolicy policy = rammp::kEStopPolicy;
};

/// One message to send: the fields that change; the caller fills the rest.
struct LatchOut {
  rammp::EStopState state;
  uint32_t pub_seq;
  std::string_view reason;       ///< the cause while latched, "clear attempt" during one
  std::string_view clear_target; ///< empty unless this is a clear-attempt message
  uint32_t clear_pub_seq;
};

/// A latch change worth logging.
struct LatchEvent {
  enum class Kind : uint8_t {
    LATCHED,               ///< text: the cause
    CLEAR_ATTEMPT_STARTED, ///< text: empty
    CLEAR_ATTEMPT_REFUSED, ///< text: why
    CLEAR_ATTEMPT_EXPIRED, ///< text: why
    RELEASED,
  };
  Kind kind = Kind::LATCHED;
  char text[48] = {};
};

struct LatchCounters {
  uint32_t clear_attempts = 0; ///< attempts started
  uint32_t releases = 0;       ///< stops ended
  uint32_t sources_dropped =
      0; ///< messages from a source beyond kMaxSources (ENGAGED still latches)
  uint32_t events_dropped = 0; ///< events overwritten before take_event()
};

class LatchLogic {
public:
  static constexpr std::size_t kMaxSources = 16;
  static constexpr std::size_t kMaxEvents = 8;

  explicit LatchLogic(const LatchConfig &config);

  /// A received e-stop message, already decoded with self = rammp::kEStopClearer, at its
  /// receive time. Our own (from_self) is ignored.
  void receive(const Decoded &msg, Time rx);

  /// The operator asks to end the stop. False if not latched, the local switch is held,
  /// or (deadman) a watched source has never been heard, so it could never succeed.
  bool start_clear_attempt(Time now, bool switch_held = false);

  /// This node's own reason to stop (its switch, a fault). Ends any clear attempt; the
  /// next message is due at once. Not for an ISR.
  void engage(std::string_view reason);

  /// Deadman and clear-attempt progress. Call at least every policy.period. A held local
  /// switch ends any clear attempt.
  void tick(Time now, bool switch_held = false);

  /// Whether a message is due: an engage not yet sent, or a period since the last one.
  bool send_due(Time now) const { return send_now_ || now >= next_send_; }

  /// The message to send now: a clear-attempt message (one target per period, in turn)
  /// while an attempt runs, else the latch state. Advances pub_seq and the next send time.
  /// The views in it stay valid until the next call that changes the latch.
  LatchOut next(Time now);

  /// When the next message is due, for the caller's wait.
  Time next_send() const { return next_send_; }

  bool latched() const { return latched_; }
  const char *cause() const { return cause_; }
  bool clear_attempt_running() const { return attempt_running_; }

  /// The oldest event not yet taken. False when there is none.
  bool take_event(LatchEvent &out);

  uint32_t pub_seq() const { return pub_seq_; }
  const LatchCounters &counters() const { return counters_; }

private:
  struct Source {
    std::optional<uint32_t> last_pub_seq; ///< newest pub_seq received; empty = forgotten
    bool engaged = false;                 ///< what that newest message said
    bool heard = false;
    Time last_rx{0};
    std::optional<uint32_t> clear_pub_seq; ///< set while a clear attempt targets it
  };
  using Sources = std::map<std::string, Source, std::less<>>;

  bool silent(const Source &src, Time now) const;
  void latch(std::string_view source, std::string_view reason);
  void cancel_attempt();
  void expire_attempt(std::string_view why, std::string_view who = {});
  void event(LatchEvent::Kind kind, std::string_view a, std::string_view b = {});
  Source *find_or_add(std::string_view name);

  LatchConfig config_;
  bool latched_ = true;
  char cause_[48] = "boot";
  Sources sources_;
  bool attempt_running_ = false;
  Time attempt_started_{0};
  std::size_t attempt_turn_ = 0;
  uint32_t pub_seq_ = 0;
  bool send_now_ = false;
  Time next_send_{0};
  LatchEvent events_[kMaxEvents];
  std::size_t events_head_ = 0;
  std::size_t events_count_ = 0;
  LatchCounters counters_;
};

} // namespace estop::core
