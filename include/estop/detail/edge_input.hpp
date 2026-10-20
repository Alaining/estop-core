#pragma once

// A switch input for the e-stop tasks: the configured pull, an interrupt on the pressing
// edge that only counts and wakes the owning task. No polling: a press shorter than any
// poll period is still counted. Owned by one task; edges() and pressed() are safe anywhere.

#include <atomic>
#include <cstdint>

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace estop {

/// The level the pin reads while the switch is pressed. The interrupt fires on the edge
/// into that level: falling for LOW, rising for HIGH.
enum class ActiveLevel : uint8_t {
  LOW = 0,  ///< pressed pulls the pin low (a normally-open switch to GND, with a pull-up)
  HIGH = 1, ///< pressed takes the pin high (a normally-closed switch to GND, with a pull-up)
};

/// The pin's internal resistor.
enum class Pull : uint8_t {
  UP,   ///< internal pull-up
  DOWN, ///< internal pull-down
  NONE, ///< none: the board has its own resistor
};

/// One switch input. The defaults are the RAMMP wiring: a normally-open switch shorting the
/// pin to GND, the internal pull-up holding it high otherwise.
struct Input {
  gpio_num_t gpio_num = GPIO_NUM_NC;           ///< GPIO_NUM_NC = no switch
  ActiveLevel active_level = ActiveLevel::LOW; ///< the level while pressed
  Pull pull = Pull::UP;
};

} // namespace estop

namespace estop::detail {

class EdgeInput {
public:
  enum class Edges : uint8_t { PRESS, BOTH };

  /// Arms `input`: the pin as an input with its pull, then (after it settles) the interrupt
  /// on the pressing edge, or on both. Installs the GPIO ISR service only if nobody has.
  /// False = not armed; the caller retries.
  bool arm(const Input &input, Edges edges, TaskHandle_t task);
  void disarm();

  bool armed() const { return armed_; }
  uint32_t edges() const { return edges_.load(std::memory_order_relaxed); }

  /// Whether the pin reads its active level now. False when not armed.
  bool pressed() const;

private:
  static void on_edge(void *self);

  std::atomic<uint32_t> edges_{0};
  TaskHandle_t task_ = nullptr;
  Input input_{};
  bool armed_ = false;
};

} // namespace estop::detail
