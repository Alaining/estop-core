// estop::Node example: a board with an e-stop switch (the HMI is one).
//
// What a node does:
//   - Its switch (or engage()) REQUESTS the robot-wide stop: the node sends
//     state=ENGAGED on rammp/estop every 100 ms, released switch or not.
//   - Otherwise it sends state=NOT_ENGAGED every 100 ms: the heartbeat the latch (the MIB)
//     watches. If it goes silent, the latch stops the robot anyway (deadman).
//   - Only the latch can end the request: it sends a "clear attempt", and the node accepts
//     it only if the switch is no longer held.
//   - active() says whether driving must not happen. Check it before every motion command.
//
// Wiring assumed here: a normally-open switch between GPIO2 and GND. The internal pull-up
// keeps the pin high; pressing pulls it low.

#include <cstdio>

#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "protocol_examples_common.h"
#include "rtps_participant.hpp"

#include "estop/estop.hpp"

namespace {

constexpr const char *kTag = "estop_node";

// One node per board, declared static: the object holds its own task stack and queues, so
// nothing is allocated at run time.
estop::Node node({
    // The name other boards see in EStop.source. Unique per board, 15 characters at most.
    // The latch watches nodes by this name (its Config::watched).
    .name = "hmi",
    // The e-stop switch. These are the defaults spelled out: pressed reads LOW (a
    // normally-open switch to GND) with the internal pull-up. For a normally-closed switch
    // to GND use .active_level = estop::ActiveLevel::HIGH. GPIO_NUM_NC = no switch.
    .input = {.gpio_num = GPIO_NUM_2,
              .active_level = estop::ActiveLevel::LOW,
              .pull = estop::Pull::UP},
});

} // namespace

extern "C" void app_main(void) {
  // 1. Start the node first, before the network: from here the switch is watched and
  //    active() is meaningful. Until start() succeeds, active() is true (fail-safe).
  if (!node.start()) {
    ESP_LOGE(kTag, "e-stop node did not start: active() stays true");
  }

  // 2. Bring up the network (here IDF's example helper: the P4's internal Ethernet).
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  ESP_ERROR_CHECK(example_connect());
  esp_netif_ip_info_t ip{};
  ESP_ERROR_CHECK(esp_netif_get_ip_info(get_example_netif(), &ip));
  char address[16];
  std::snprintf(address, sizeof(address), IPSTR, IP2STR(&ip.ip));

  // 3. Start your RTPS participant (the one the rest of your firmware uses) and attach
  //    the node to it: this adds the rammp/estop writer and reader. Attach before your
  //    other endpoints, so a short endpoint pool fails them rather than the e-stop.
  static espp::RtpsParticipant participant({.interface_address = address});
  if (!participant.start() || !node.attach(participant)) {
    ESP_LOGE(kTag, "RTPS did not come up: the latch will stop the robot (deadman)");
  }

  // 4. Use it. active() is one atomic load, safe from any task (and from an ISR): gate
  //    every motion command on it. status() is a full copy, for the UI and the log.
  //    Software can also request the stop: node.engage("motor fault").
  for (;;) {
    if (node.active()) {
      // Stop, or refuse to start, any motion here.
    }
    const estop::NodeStatus s = node.status();
    ESP_LOGI(kTag, "active %d engaged %d latch %s%s pub_seq %lu", s.active, s.engaged,
             s.latch_fresh ? (s.latch_engaged ? "ENGAGED " : "NOT_ENGAGED") : "silent",
             s.latch_engaged ? s.latch_reason : "", static_cast<unsigned long>(s.pub_seq));
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}
