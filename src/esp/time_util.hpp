#pragma once

// Clocks and small text helpers shared by Node and Latch. No allocation.

#include <sys/time.h>

#include "esp_timer.h"
#include "freertos/FreeRTOS.h"

#include "estop/core/node_logic.hpp"
#include "messages/estop_message.hpp"

namespace estop::detail {

/// Steady time since boot: what every e-stop age is measured on.
inline core::Time now() { return core::Time{esp_timer_get_time()}; }

/// Ticks to wait until `next`, at least one so the task always yields.
inline TickType_t ticks_until(core::Time now, core::Time next) {
  const int64_t us = next > now ? (next - now).count() : 0;
  return pdMS_TO_TICKS(us / 1000) + 1;
}

/// The wall clock for the header stamp (the log only). Before anything has set the clock
/// it reads 1970: send the all-zero "unstamped".
inline rammp::Time stamp_now() {
  timeval tv{};
  constexpr int64_t kClockSetAfter = 1'600'000'000; // 2020-09-13
  if (gettimeofday(&tv, nullptr) != 0 || tv.tv_sec < kClockSetAfter) {
    return {};
  }
  return {static_cast<int32_t>(tv.tv_sec), static_cast<uint32_t>(tv.tv_usec) * 1000u};
}

/// "switch gpio<n>": fits the 16-byte buffer for any GPIO number.
inline void switch_reason(char (&out)[16], int gpio) {
  constexpr char kPrefix[] = "switch gpio";
  std::size_t n = 0;
  for (; kPrefix[n] != '\0'; n++) {
    out[n] = kPrefix[n];
  }
  char digits[4] = {};
  std::size_t d = 0;
  unsigned value = gpio < 0 ? 0u : static_cast<unsigned>(gpio);
  do {
    digits[d++] = static_cast<char>('0' + value % 10);
    value /= 10;
  } while (value != 0 && d < sizeof(digits));
  while (d > 0 && n + 1 < sizeof(out)) {
    out[n++] = digits[--d];
  }
  out[n] = '\0';
}

} // namespace estop::detail
