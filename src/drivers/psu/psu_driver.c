#include "driver/gpio.h"
#include "esp_log.h"

#include "defines.h"
#include "psu_driver.h"

static const char *TAG = "PSU";

static bool psu_enabled = false;

void psu_driver_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << GPIO_PSU_EN),
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };
    gpio_config(&io_conf);
    gpio_set_level(GPIO_PSU_EN, 0);
    psu_enabled = false;

    ESP_LOGI(TAG, "HV PSU driver initialized on GPIO%d (held off)", GPIO_PSU_EN);
}

void psu_driver_enable(void)
{
    gpio_set_level(GPIO_PSU_EN, 1);
    psu_enabled = true;
    ESP_LOGI(TAG, "HV PSU enabled");
}

void psu_driver_disable(void)
{
    gpio_set_level(GPIO_PSU_EN, 0);
    psu_enabled = false;
    ESP_LOGI(TAG, "HV PSU disabled");
}

bool psu_driver_is_enabled(void)
{
    return psu_enabled;
}
