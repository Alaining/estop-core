#pragma once

/**
 * @file codec.hpp
 * @brief The e-stop message on the wire: encode with espp/cdr, decode by hand.
 *
 * Encoding goes through cdr::serialize_into, so the layout is rammp::EStop's by
 * construction. Decoding walks the received bytes and builds no std::string, so the
 * receive path never allocates; test/test_codec.cpp checks it against cdr both ways.
 *
 * Accepted: XCDR1 little-endian (encapsulation 0x00 0x01), which every RAMMP node sends,
 * in the full layout or the 4-field ROS one. The ROS layout reads as pub_seq 0 and no
 * clear_target, so it is a heartbeat or an engage, never a clear attempt. The state octet
 * is read fail-safe: anything but NOT_ENGAGED (0) is ENGAGED.
 */

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>

#include "cdr/cdr.hpp"
#include "messages/estop_message.hpp"

namespace estop::core {

/// Copies `in` into `out`, truncated and always NUL-terminated.
template <std::size_t N> constexpr void copy_text(char (&out)[N], std::string_view in) {
  const std::size_t n = in.size() < N - 1 ? in.size() : N - 1;
  for (std::size_t i = 0; i < n; i++) {
    out[i] = in[i];
  }
  out[n] = '\0';
}

/// What a receiver needs from one e-stop message: plain values, so it fits a queue.
struct Decoded {
  uint32_t pub_seq = 0;
  uint32_t clear_pub_seq = 0;
  bool engaged = false;        ///< state is anything but NOT_ENGAGED
  bool from_self = false;      ///< source is this node: our own sample, ignored
  bool from_clearer = false;   ///< source is rammp::kEStopClearer
  bool clear_for_self = false; ///< clear_target is this node: a clear attempt for us
  char source[16] = {};        ///< truncated, for the log and the latch's table
  char reason[48] = {};
};

/// XCDR1 little-endian reader over the body (after the 4-byte encapsulation header,
/// which is the alignment origin). Every read is bounds-checked.
class Reader {
public:
  explicit constexpr Reader(std::span<const uint8_t> body)
      : body_(body) {}

  /// Whether `size` more bytes, aligned to `size`, are left: an appended field that is
  /// absent (a shorter layout, or only the trailing padding) reads as "no more".
  constexpr bool has(std::size_t size) const { return aligned(size) + size <= body_.size(); }

  constexpr bool u8(uint8_t &out) {
    if (pos_ + 1 > body_.size()) {
      return false;
    }
    out = body_[pos_++];
    return true;
  }

  constexpr bool u32(uint32_t &out) {
    const std::size_t at = aligned(4);
    if (at + 4 > body_.size()) {
      return false;
    }
    out = static_cast<uint32_t>(body_[at]) | (static_cast<uint32_t>(body_[at + 1]) << 8) |
          (static_cast<uint32_t>(body_[at + 2]) << 16) |
          (static_cast<uint32_t>(body_[at + 3]) << 24);
    pos_ = at + 4;
    return true;
  }

  /// A CDR string: length including the NUL, the bytes, the NUL. A length of 0 (some
  /// writers send it for "") reads as empty. Not constexpr: it views bytes as chars.
  bool string(std::string_view &out) {
    uint32_t len = 0;
    if (!u32(len)) {
      return false;
    }
    if (len == 0) {
      out = {};
      return true;
    }
    if (len > body_.size() - pos_ || body_[pos_ + len - 1] != 0) {
      return false;
    }
    out = std::string_view(reinterpret_cast<const char *>(body_.data() + pos_), len - 1);
    pos_ += len;
    return true;
  }

private:
  constexpr std::size_t aligned(std::size_t size) const { return (pos_ + size - 1) / size * size; }

  std::span<const uint8_t> body_;
  std::size_t pos_ = 0;
};

/// Decodes `payload` (encapsulation header + body). `self` is this node's source name,
/// `clearer` rammp::kEStopClearer. False = not an e-stop message this code can read.
inline bool decode(std::span<const uint8_t> payload, std::string_view self,
                   std::string_view clearer, Decoded &out) {
  out = Decoded{};
  if (payload.size() < 4 || payload[0] != 0x00 || payload[1] != 0x01) {
    return false; // not XCDR1 little-endian
  }
  Reader r(payload.subspan(4));
  uint32_t sec = 0;
  uint32_t nanosec = 0;
  std::string_view frame_id;
  uint8_t state = 0;
  std::string_view source;
  std::string_view reason;
  // header.stamp and header.frame_id are read past: the stamp is for the log only.
  if (!r.u32(sec) || !r.u32(nanosec) || !r.string(frame_id) || !r.u8(state) || !r.string(source) ||
      !r.string(reason)) {
    return false;
  }
  out.engaged = state != static_cast<uint8_t>(rammp::EStopState::NOT_ENGAGED);
  out.from_self = source == self;
  out.from_clearer = source == clearer;
  copy_text(out.source, source);
  copy_text(out.reason, reason);
  // The appended fields. Absent = the 4-field ROS layout: pub_seq 0, no clear_target.
  if (!r.has(4)) {
    return true;
  }
  if (!r.u32(out.pub_seq)) {
    return false;
  }
  if (!r.has(4)) {
    return true;
  }
  std::string_view clear_target;
  if (!r.string(clear_target) || !r.u32(out.clear_pub_seq)) {
    return false;
  }
  out.clear_for_self = !clear_target.empty() && clear_target == self;
  return true;
}

/// Largest message a node sends with names and reasons of 15 characters or fewer.
inline constexpr std::size_t kMaxEncoded = 128;

/// Encodes `msg` into `out`. The size written, or 0 if it does not fit. Allocation-free
/// as long as msg's strings are 15 characters or fewer (std::string's in-place buffer).
inline std::size_t encode(const rammp::EStop &msg, std::span<uint8_t, kMaxEncoded> out) {
  const auto size = cdr::serialize_into<cdr::xcdr1>(msg, std::as_writable_bytes(std::span(out)));
  return size ? *size : 0;
}

} // namespace estop::core
