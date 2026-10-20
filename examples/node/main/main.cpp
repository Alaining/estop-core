// estop::Node: a node with an e-stop switch on GPIO2 (normally open, to GND). A press
// requests the stop; only the latch (the MIB) can end it.

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

// Static: the node holds its task's stack and queues.
estop::Node node({.name = "hmi", .gpio = 2, .contact = estop::Contact::NO});

} // namespace

extern "C" void app_main(void) {
  // First, before the network: the switch works, and active() is meaningful, from here.
  if (!node.start()) {
    ESP_LOGE(kTag, "e-stop node did not start: active() stays true");
  }

  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  ESP_ERROR_CHECK(example_connect());
  esp_netif_ip_info_t ip{};
  ESP_ERROR_CHECK(esp_netif_get_ip_info(get_example_netif(), &ip));
  char address[16];
  std::snprintf(address, sizeof(address), IPSTR, IP2STR(&ip.ip));

  static espp::RtpsParticipant participant({.interface_address = address});
  if (!participant.start() || !node.attach(participant)) {
    ESP_LOGE(kTag, "RTPS did not come up: the latch will stop the robot (deadman)");
  }

  for (;;) {
    const estop::NodeStatus s = node.status();
    ESP_LOGI(kTag, "active %d engaged %d latch %s%s pub_seq %lu", s.active, s.engaged,
             s.latch_fresh ? (s.latch_engaged ? "ENGAGED " : "NOT_ENGAGED") : "silent",
             s.latch_engaged ? s.latch_reason : "", static_cast<unsigned long>(s.pub_seq));
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}
