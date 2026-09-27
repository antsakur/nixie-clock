#include "driver/ledc.h"
#include "esp_log.h"

#include "defines.h"
#include "pwm_driver.h"

static const char *TAG = "PWM";

#define PWM_FREQ_HZ 30000

void pwm_driver_init(int gpio_num)
{
    ledc_timer_config_t pwm_timer = {
        .duty_resolution = PWM_RESOLUTION_BITS,
        .freq_hz         = PWM_FREQ_HZ,
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .timer_num       = LEDC_TIMER_0,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&pwm_timer));

    ledc_channel_config_t pwm_channel_config = {
        .gpio_num   = gpio_num,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = LEDC_CHANNEL_0,
        .intr_type  = LEDC_INTR_DISABLE,
        .timer_sel  = LEDC_TIMER_0,
        .duty       = 0,
        .hpoint     = 0,
        .flags.output_invert = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&pwm_channel_config));
    ESP_ERROR_CHECK(ledc_fade_func_install(0));

    ESP_LOGI(TAG, "PWM initialized on GPIO %d (%d Hz, %d-bit)",
             gpio_num, PWM_FREQ_HZ, PWM_RESOLUTION_BITS);
}

void pwm_driver_set_duty(uint32_t duty_cycle_percent)
{
    if (duty_cycle_percent > 100) {
        duty_cycle_percent = 100;
    }

    const uint32_t max_duty = (1u << PWM_RESOLUTION_BITS) - 1u;
    const uint32_t duty = (max_duty * duty_cycle_percent) / 100u;

    ESP_ERROR_CHECK(ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty, 0));
}
