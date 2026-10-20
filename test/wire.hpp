#pragma once

// Test messages through the real codec: built as rammp::EStop, encoded with core::encode,
// decoded with core::decode as the receiving node would.

#include <array>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <string_view>
#include <vector>

#include "estop/core/codec.hpp"
#include "messages/estop_message.hpp"

namespace estop::test {

using Bytes = std::vector<uint8_t>;

inline rammp::EStop message(bool engaged, std::string source, std::string reason, uint32_t pub_seq,
                            std::string clear_target = "", uint32_t clear_pub_seq = 0) {
  return rammp::EStop{.header = {},
                      .state =
                          engaged ? rammp::EStopState::ENGAGED : rammp::EStopState::NOT_ENGAGED,
                      .source = std::move(source),
                      .reason = std::move(reason),
                      .pub_seq = pub_seq,
                      .clear_target = std::move(clear_target),
                      .clear_pub_seq = clear_pub_seq};
}

inline Bytes encode(const rammp::EStop &msg) {
  std::array<uint8_t, core::kMaxEncoded> buf{};
  const std::size_t size = core::encode(msg, buf);
  if (size == 0) {
    std::printf("encode failed\n");
    std::exit(2);
  }
  return Bytes(buf.begin(), buf.begin() + static_cast<long>(size));
}

/// What `self` decodes from `bytes`.
inline core::Decoded decode(const Bytes &bytes, std::string_view self) {
  core::Decoded d;
  if (!core::decode(bytes, self, rammp::kEStopClearer, d)) {
    std::printf("decode failed\n");
    std::exit(2);
  }
  return d;
}

/// A message as `self` receives it.
inline core::Decoded received(std::string_view self, bool engaged, std::string source,
                              uint32_t pub_seq, std::string clear_target = "",
                              uint32_t clear_pub_seq = 0, std::string reason = "") {
  return decode(encode(message(engaged, std::move(source), std::move(reason), pub_seq,
                               std::move(clear_target), clear_pub_seq)),
                self);
}

} // namespace estop::test
