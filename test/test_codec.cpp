// The decoder against espp/cdr's encoder (what every espp node sends with), both ways,
// plus truncation, corruption and fuzzing under ASan/UBSan.

#include <array>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <string>
#include <vector>

#include <cdr/cdr.hpp>

#include "check.hpp"
#include "estop/core/codec.hpp"
#include "messages/estop_message.hpp"

namespace {

using estop::core::Decoded;

// The ROS EStop layout, without the appended fields: what a node that knows only
// the ROS message sends.
struct RosEStop {
  rammp::Header header;
  bool engaged;
  std::string source;
  std::string reason;
};

template <class T> std::vector<uint8_t> encode(const T &msg) {
  auto bytes = cdr::serialize<cdr::xcdr1>(msg);
  if (!bytes) {
    std::printf("encode failed\n");
    std::exit(2);
  }
  std::vector<uint8_t> out(bytes->size());
  for (size_t i = 0; i < out.size(); i++) {
    out[i] = static_cast<uint8_t>((*bytes)[i]);
  }
  return out;
}

bool decode(const std::vector<uint8_t> &bytes, Decoded &out) {
  return estop::core::decode(bytes, "hmi", rammp::kEStopClearer, out);
}

rammp::EStop full(bool engaged, std::string source, std::string reason, uint32_t pub_seq,
                  std::string clear_target, uint32_t clear_pub_seq) {
  return rammp::EStop{.header = {.stamp = {1'760'000'000, 123}, .frame_id = ""},
                      .state =
                          engaged ? rammp::EStopState::ENGAGED : rammp::EStopState::NOT_ENGAGED,
                      .source = std::move(source),
                      .reason = std::move(reason),
                      .pub_seq = pub_seq,
                      .clear_target = std::move(clear_target),
                      .clear_pub_seq = clear_pub_seq};
}

void test_round_trip() {
  // String lengths 0..9 move every later field through each alignment.
  for (size_t src_len = 0; src_len < 10; src_len++) {
    for (size_t why_len = 0; why_len < 10; why_len++) {
      const std::string src(src_len, 's');
      const std::string why(why_len, 'r');
      Decoded d;
      CHECK(decode(encode(full(true, src, why, 0xDEADBEEF, "", 0)), d));
      CHECK(d.engaged && d.pub_seq == 0xDEADBEEF && !d.clear_for_self && d.clear_pub_seq == 0);
      CHECK(std::string(d.source) == src && std::string(d.reason) == why);
    }
  }
}

void test_clear() {
  Decoded d;
  CHECK(decode(encode(full(false, "mib", "operator clear", 7, "hmi", 42)), d));
  CHECK(!d.engaged && d.from_clearer && !d.from_self && d.clear_for_self);
  CHECK(d.pub_seq == 7 && d.clear_pub_seq == 42);

  // A clear attempt for someone else is not ours.
  CHECK(decode(encode(full(false, "mib", "", 8, "other", 42)), d));
  CHECK(d.from_clearer && !d.clear_for_self);

  // A clear attempt from anyone but the MIB is decoded, and flagged as not the clearer.
  CHECK(decode(encode(full(false, "rogue", "", 9, "hmi", 42)), d));
  CHECK(!d.from_clearer && d.clear_for_self);

  // A node with no name (misconfigured) never reads an empty clear_target as its own.
  CHECK(
      estop::core::decode(encode(full(false, "mib", "", 10, "", 0)), "", rammp::kEStopClearer, d));
  CHECK(!d.clear_for_self);

  // Our own sample.
  CHECK(decode(encode(full(true, "hmi", "switch gpio2", 3, "", 0)), d));
  CHECK(d.from_self);
}

void test_ros_layout() {
  // The 4-field layout reads as pub_seq 0 with no clear_target: a heartbeat or an engage, never a
  // clear attempt. Lengths vary so the message ends with and without trailing padding.
  for (size_t why_len = 0; why_len < 8; why_len++) {
    const RosEStop ros{.header = {.stamp = {0, 0}, .frame_id = "base"},
                       .engaged = why_len % 2 == 0,
                       .source = "mib",
                       .reason = std::string(why_len, 'x')};
    auto bytes = encode(ros);
    Decoded d;
    CHECK(decode(bytes, d));
    CHECK(d.engaged == ros.engaged && d.pub_seq == 0 && !d.clear_for_self && d.from_clearer);
    // And with up to 3 padding bytes after it, as an RTPS writer may add.
    for (int pad = 1; pad <= 3; pad++) {
      bytes.push_back(0);
      CHECK(decode(bytes, d));
      CHECK(d.pub_seq == 0 && !d.clear_for_self);
    }
  }
}

void test_rejects() {
  auto good = encode(full(true, "mib", "x", 1, "", 0));
  Decoded d;

  auto be = good;
  be[1] = 0x00; // CDR big-endian
  CHECK(!decode(be, d));
  auto xcdr2 = good;
  xcdr2[1] = 0x07; // XCDR2 little-endian
  CHECK(!decode(xcdr2, d));
  CHECK(!decode({}, d));

  // Every truncation: refused, or (cut inside the appended fields) read as the
  // shorter layout -- never a clear attempt, never out of bounds (ASan watches).
  auto clear = encode(full(false, "mib", "why", 5, "hmi", 4));
  for (size_t n = 0; n < clear.size(); n++) {
    const std::vector<uint8_t> cut(clear.begin(), clear.begin() + static_cast<long>(n));
    if (decode(cut, d)) {
      CHECK(!d.clear_for_self);
    }
  }

  // A string whose last byte is not the NUL.
  auto no_nul = encode(full(true, "mib", "abc", 1, "", 0));
  // Body offsets: sec 0, nanosec 4, frame_id length 8 and its NUL 12, engaged 13,
  // source length 16, then "mib\0" at 20..23. Overwrite that NUL (header 4 + 23).
  CHECK(no_nul[4 + 23] == 0);
  no_nul[4 + 23] = 'X';
  CHECK(!decode(no_nul, d));
}

void test_unknown_state_engages() {
  // EStopState is one octet: 0 NOT_ENGAGED, 1 ENGAGED. Any other value must read as
  // engaged. Body offsets: sec 0, nanosec 4, frame_id length 8 and its NUL 12, state 13.
  auto bytes = encode(full(false, "mib", "x", 1, "", 0));
  CHECK(bytes[4 + 13] == 0);
  for (int v : {2, 0x7F, 0xFF}) {
    bytes[4 + 13] = static_cast<uint8_t>(v);
    Decoded d;
    CHECK(decode(bytes, d) && d.engaged);
  }
}

void test_long_text_truncates() {
  Decoded d;
  const std::string long_reason(200, 'L');
  CHECK(decode(encode(full(true, "a-very-long-node-name", long_reason, 1, "", 0)), d));
  CHECK(std::string(d.source).size() == sizeof(d.source) - 1);
  CHECK(std::string(d.reason).size() == sizeof(d.reason) - 1);
  CHECK(!d.from_clearer);
}

void test_encode() {
  // core::encode is cdr's encoding, byte for byte, and decodes back.
  const auto msg = full(true, "hmi", "switch gpio47", 0xFFFFFFFF, "", 0);
  std::array<uint8_t, estop::core::kMaxEncoded> buf{};
  const std::size_t size = estop::core::encode(msg, buf);
  const auto ref = encode(msg);
  CHECK(size == ref.size());
  CHECK(std::equal(ref.begin(), ref.end(), buf.begin()));
  Decoded d;
  CHECK(estop::core::decode(std::span(buf.data(), size), "mib", rammp::kEStopClearer, d));
  CHECK(d.engaged && d.pub_seq == 0xFFFFFFFF && std::string(d.reason) == "switch gpio47");

  // The longest a node sends with 15-character names and reasons fits.
  const std::string n15(15, 'n');
  CHECK(estop::core::encode(full(false, n15, n15, 1, n15, 2), buf) > 0);

  // Too long for the buffer: 0, nothing written past it (ASan watches).
  CHECK(estop::core::encode(full(true, "hmi", std::string(200, 'L'), 1, "", 0), buf) == 0);
}

void test_fuzz() {
  std::mt19937 rng(1234);
  const auto seed = encode(full(false, "mib", "fuzz", 77, "hmi", 76));
  for (int i = 0; i < 200'000; i++) {
    std::vector<uint8_t> bytes = seed;
    const int flips = 1 + static_cast<int>(rng() % 4);
    for (int f = 0; f < flips; f++) {
      bytes[rng() % bytes.size()] = static_cast<uint8_t>(rng());
    }
    if (rng() % 4 == 0) {
      bytes.resize(rng() % (bytes.size() + 8));
    }
    Decoded d;
    (void)decode(bytes, d); // only that it never reads out of bounds
  }
}

} // namespace

int main() {
  test_round_trip();
  test_clear();
  test_ros_layout();
  test_rejects();
  test_unknown_state_engages();
  test_long_text_truncates();
  test_encode();
  test_fuzz();
  return estop::test::verdict("codec");
}
