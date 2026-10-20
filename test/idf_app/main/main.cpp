// The core on the target: one node and one latch talking through the codec, on the
// steady clock. A build check first; on a board it logs the handshake.

#include <array>
#include <chrono>
#include <span>
#include <string>

#include "esp_log.h"
#include "esp_timer.h"

#include "estop/core/latch_logic.hpp"
#include "estop/core/node_logic.hpp"

namespace {

constexpr const char *kTag = "estop_core";
constexpr std::string_view kWatched[] = {"hmi"};

estop::core::Time now() { return estop::core::Time{esp_timer_get_time()}; }

std::size_t encode(const rammp::EStop &msg, std::array<uint8_t, estop::core::kMaxEncoded> &buf) {
  return estop::core::encode(msg, buf);
}

} // namespace

extern "C" void app_main(void) {
  estop::core::NodeLogic node({.name = "hmi"});
  estop::core::LatchLogic latch({.watched = kWatched});
  std::array<uint8_t, estop::core::kMaxEncoded> buf{};
  rammp::EStop msg{};

  node.press();
  for (int period = 0; period < 6; period++) {
    const auto t = now();
    const auto out = node.next(t);
    msg.state = out.state;
    msg.source = "hmi";
    msg.pub_seq = out.pub_seq;
    estop::core::Decoded d;
    if (estop::core::decode(std::span(buf.data(), encode(msg, buf)), rammp::kEStopClearer,
                            rammp::kEStopClearer, d)) {
      latch.receive(d, t);
    }
    if (period == 2) {
      (void)latch.start_clear_attempt(t);
    }
    latch.tick(t);
    const auto back = latch.next(t);
    msg.state = back.state;
    msg.source = rammp::kEStopClearer;
    msg.pub_seq = back.pub_seq;
    msg.clear_target = std::string(back.clear_target);
    msg.clear_pub_seq = back.clear_pub_seq;
    if (estop::core::decode(std::span(buf.data(), encode(msg, buf)), "hmi", rammp::kEStopClearer,
                            d)) {
      node.receive(d, t, t, false);
    }
    msg.clear_target.clear();
    ESP_LOGI(kTag, "period %d: node %s, latch %s", period,
             node.engaged() ? "ENGAGED" : "NOT_ENGAGED", latch.latched() ? "LATCHED" : "RELEASED");
  }
}
