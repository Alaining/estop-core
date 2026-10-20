// estop::Latch example: the board that owns the robot-wide stop (the MIB).
//
// What the latch does:
//   - It starts LATCHED (stopped): at boot nothing is known to be safe.
//   - It latches on any ENGAGED message from any node (a pressed switch somewhere), and
//     (deadman) when a watched node has been silent longer than 300 ms.
//   - It ends the stop only through a CLEAR ATTEMPT: the operator holds the clear button
//     for 2 s (or the firmware calls start_clear_attempt()). The latch then asks every
//     node to clear; it RELEASES once each one has answered NOT_ENGAGED. A node refuses
//     while its switch is still held, and the attempt expires after 1 s.
//   - It sends its own state on rammp/estop every 100 ms, so nodes can show the stop.
//   - active() says the stop is in force. The motor code must check it: while active(),
//     no motion.
//
// Wiring assumed here: a normally-open clear button between GPIO35 and GND (internal
// pull-up). This board has no e-stop switch of its own; give it one with Config::input.

#include <cstdio>
#include <string_view>

#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "protocol_examples_common.h"
#include "rtps_participant.hpp"

#include "estop/estop.hpp"

namespace {

constexpr const char *kTag = "estop_latch";

// The nodes this latch expects to hear every 100 ms (their Node::Config::name). If one goes
// silent for 300 ms, the robot stops. List every board that has an e-stop switch.
constexpr std::string_view kWatched[] = {"hmi"};

// One latch per robot, declared static: it holds its own task stack and queues.
estop::Latch latch({
    .watched = kWatched,
    // Optional: an e-stop switch on this board too. GPIO_NUM_NC (the default) = none.
    // .input = {.gpio_num = GPIO_NUM_XX},
    // The clear button: held for clear_hold, it starts a clear attempt. Defaults spelled
    // out: pressed reads LOW (normally open, to GND) with the internal pull-up.
    .clear_input = {.gpio_num = GPIO_NUM_35,
                    .active_level = estop::ActiveLevel::LOW,
                    .pull = estop::Pull::UP},
    .clear_hold = std::chrono::milliseconds(2000),
    // Latch changes (LATCHED, clear attempt, RELEASED) go to the log, tag "estop".
    .log_events = true,
});

} // namespace

extern "C" void app_main(void) {
  // 1. Start the latch first, before the network and before the motors: it starts latched,
  //    so nothing can move until every watched node has been heard and a clear attempt
  //    has succeeded.
  if (!latch.start()) {
    ESP_LOGE(kTag, "e-stop latch did not start: active() stays true");
  }

  // 2. Bring up the network (here IDF's example helper: the P4's internal Ethernet).
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  ESP_ERROR_CHECK(example_connect());
  esp_netif_ip_info_t ip{};
  ESP_ERROR_CHECK(esp_netif_get_ip_info(get_example_netif(), &ip));
  char address[16];
  std::snprintf(address, sizeof(address), IPSTR, IP2STR(&ip.ip));

  // 3. Attach the latch to your firmware's RTPS participant: this adds the rammp/estop
  //    writer and reader. Without it the latch hears nobody and stays latched.
  static espp::RtpsParticipant participant({.interface_address = address});
  if (!participant.start() || !latch.attach(participant)) {
    ESP_LOGE(kTag, "RTPS did not come up: the latch holds");
  }

  // 4. Use it:
  //    - latch.active(): one atomic load, any task. Gate every motor command on it, e.g. in
  //      your drive state machine: if (latch.active()) { disable_drive(); }
  //    - latch.engage("reason"): your own reasons to stop (a fault, an overcurrent).
  //    - latch.start_clear_attempt(): the same as holding the clear button, for a clear
  //      that comes from software (a UI command, a CLI).
  //    - latch.status(): a full copy (cause, attempts, releases) for the UI and the log.
  for (;;) {
    if (latch.active()) {
      // Stop the motors here, every cycle.
    }
    const estop::LatchStatus s = latch.status();
    ESP_LOGI(kTag, "%s%s%s releases %lu", s.latched ? "LATCHED: " : "released",
             s.latched ? s.cause : "", s.clear_attempt_running ? " (clear attempt)" : "",
             static_cast<unsigned long>(s.releases));
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}
