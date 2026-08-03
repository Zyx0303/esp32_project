#include "power_control.h"

#include "esp_check.h"

esp_err_t power_control_init(power_control_t *control, gpio_num_t gpio)
{
    ESP_RETURN_ON_FALSE(control != NULL, ESP_ERR_INVALID_ARG, "power_control", "control required");
    ESP_RETURN_ON_FALSE(GPIO_IS_VALID_OUTPUT_GPIO(gpio), ESP_ERR_INVALID_ARG,
                        "power_control", "invalid output GPIO");

    // Set the output latch low before enabling the pin as an output. This keeps
    // the 2N7002 off and leaves the external active-low interface deasserted.
    ESP_RETURN_ON_ERROR(gpio_set_level(gpio, 0), "power_control", "set safe level");
    const gpio_config_t config = {
        .pin_bit_mask = 1ULL << gpio,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&config), "power_control", "configure GPIO");

    control->gpio = gpio;
    control->initialized = true;
    return ESP_OK;
}

esp_err_t power_control_set_asserted(power_control_t *control, bool asserted)
{
    ESP_RETURN_ON_FALSE(control != NULL && control->initialized,
                        ESP_ERR_INVALID_STATE, "power_control", "not initialized");
    return gpio_set_level(control->gpio, asserted ? 1 : 0);
}
