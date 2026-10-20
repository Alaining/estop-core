// estop::Latch: the node that owns the latch (the MIB). It watches the node "hmi",
// latches on any ENGAGED or on "hmi" going silent, and starts a clear attempt when the
// clear button on GPIO35 (normally open, to GND) is held for 2 s.

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

constexpr std::string_view kWatched[] = {"hmi"};

// Static: the latch holds its task's stack and queues.
estop::Latch latch({.watched = kWatched, .clear_gpio = 35});

} // namespace

extern "C" void app_main(void) {
  if (!latch.start()) {
    ESP_LOGE(kTag, "e-stop latch did not start: active() stays true");
  }

  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  ESP_ERROR_CHECK(example_connect());
  esp_netif_ip_info_t ip{};
  ESP_ERROR_CHECK(esp_netif_get_ip_info(get_example_netif(), &ip));
  char address[16];
  std::snprintf(address, sizeof(address), IPSTR, IP2STR(&ip.ip));

  static espp::RtpsParticipant participant({.interface_address = address});
  if (!participant.start() || !latch.attach(participant)) {
    ESP_LOGE(kTag, "RTPS did not come up: the latch holds");
  }

  for (;;) {
    const estop::LatchStatus s = latch.status();
    ESP_LOGI(kTag, "%s%s%s releases %lu", s.latched ? "LATCHED: " : "released",
             s.latched ? s.cause : "", s.clear_attempt_running ? " (clear attempt)" : "",
             static_cast<unsigned long>(s.releases));
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}
