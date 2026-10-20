#pragma once

// A switch input for the e-stop tasks: the pull-up on, an interrupt on the chosen edge
// that only counts and wakes the owning task. No polling: a press shorter than any poll
// period is still counted. Owned by one task; edges() and pressed() are safe anywhere.

#include <atomic>
#include <cstdint>

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace estop {

/// How a switch is wired. Both switch to GND against the internal pull-up.
enum class Contact : uint8_t {
  NO = 0, ///< normally open: pressed pulls the pin LOW
  NC = 1, ///< normally closed: pressed opens it, the pull-up takes it HIGH
};

} // namespace estop

namespace estop::detail {

class EdgeInput {
public:
  enum class Edges : uint8_t { PRESS, BOTH };

  /// Arms `gpio`: input with the pull-up, then (after it settles) the interrupt on the
  /// pressing edge (falling for NO, rising for NC) or both. Installs the GPIO ISR service
  /// only if nobody has. False = not armed; the caller retries.
  bool arm(int gpio, Contact contact, Edges edges, TaskHandle_t task);
  void disarm();

  bool armed() const { return armed_; }
  int gpio() const { return gpio_; }
  Contact contact() const { return contact_; }
  uint32_t edges() const { return edges_.load(std::memory_order_relaxed); }

  /// The contact's state now: pressed is LOW for NO, HIGH for NC. False when not armed.
  bool pressed() const;

private:
  static void on_edge(void *self);

  std::atomic<uint32_t> edges_{0};
  TaskHandle_t task_ = nullptr;
  int gpio_ = -1;
  Contact contact_ = Contact::NO;
  bool armed_ = false;
};

} // namespace estop::detail
