#include "estop/detail/edge_input.hpp"

#include "esp_attr.h"

namespace estop::detail {

namespace {

// Settling time after the pull-up goes on, before the edge interrupt is enabled: so the
// pin's own rise is not taken for a press.
constexpr TickType_t kSettleTicks = pdMS_TO_TICKS(2);

} // namespace

void IRAM_ATTR EdgeInput::on_edge(void *self) {
  auto *in = static_cast<EdgeInput *>(self);
  in->edges_.fetch_add(1, std::memory_order_relaxed);
  BaseType_t woken = pdFALSE;
  vTaskNotifyGiveFromISR(in->task_, &woken);
  portYIELD_FROM_ISR(woken);
}

bool EdgeInput::pressed() const {
  if (!armed_) {
    return false;
  }
  return gpio_get_level(input_.gpio_num) == static_cast<int>(input_.active_level);
}

void EdgeInput::disarm() {
  if (input_.gpio_num == GPIO_NUM_NC) {
    return;
  }
  // Best effort: a pin that will not let go still reads as not armed.
  (void)gpio_intr_disable(input_.gpio_num);
  (void)gpio_isr_handler_remove(input_.gpio_num);
  input_.gpio_num = GPIO_NUM_NC;
  armed_ = false;
}

bool EdgeInput::arm(const Input &input, Edges edges, TaskHandle_t task) {
  disarm();
  if (!GPIO_IS_VALID_GPIO(input.gpio_num)) {
    return false;
  }
  const gpio_num_t pin = input.gpio_num;
  task_ = task;
  gpio_config_t io = {};
  io.pin_bit_mask = 1ULL << static_cast<unsigned>(pin);
  io.mode = GPIO_MODE_INPUT;
  io.pull_up_en = input.pull == Pull::UP ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE;
  io.pull_down_en = input.pull == Pull::DOWN ? GPIO_PULLDOWN_ENABLE : GPIO_PULLDOWN_DISABLE;
  io.intr_type = GPIO_INTR_DISABLE;
  if (gpio_config(&io) != ESP_OK) {
    return false;
  }
  vTaskDelay(kSettleTicks);
  // The pressing edge is the one into the active level.
  const gpio_int_type_t type = edges == Edges::BOTH                     ? GPIO_INTR_ANYEDGE
                               : input.active_level == ActiveLevel::LOW ? GPIO_INTR_NEGEDGE
                                                                        : GPIO_INTR_POSEDGE;
  if (gpio_set_intr_type(pin, type) != ESP_OK) {
    return false;
  }
  esp_err_t err = gpio_isr_handler_add(pin, on_edge, this);
  if (err == ESP_ERR_INVALID_STATE) {
    // No ISR service yet: install it. A component that installs it after us gets
    // ESP_ERR_INVALID_STATE; espp::Interrupt logs that and its handlers still attach.
    (void)gpio_install_isr_service(0);
    err = gpio_isr_handler_add(pin, on_edge, this);
  }
  if (err != ESP_OK) {
    return false;
  }
  if (gpio_intr_enable(pin) != ESP_OK) {
    (void)gpio_isr_handler_remove(pin);
    return false;
  }
  input_ = input;
  armed_ = true;
  return true;
}

} // namespace estop::detail
