#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "defines.h"
#include "display.h"
#include "pwm_driver.h"
#include "shift_reg_driver.h"

static const char *TAG = "Display";

#define HOUR_HIGH_LD (1ULL << 47)
#define HOUR_HIGH_1  (1ULL << 46)
#define HOUR_HIGH_0  (1ULL << 37)
#define HOUR_HIGH_RD (1ULL << 36)

#define HOUR_LOW_LD (1ULL << 35)
#define HOUR_LOW_0  (1ULL << 25)
#define HOUR_LOW_RD (1ULL << 24)

#define MINUTE_HIGH_LD (1ULL << 23)
#define MINUTE_HIGH_0  (1ULL << 13)
#define MINUTE_HIGH_RD (1ULL << 12)

#define MINUTE_LOW_LD (1ULL << 11)
#define MINUTE_LOW_1  (1ULL << 10)
#define MINUTE_LOW_0  (1ULL << 1)
#define MINUTE_LOW_RD (1ULL << 0)

static const uint64_t dot_animation_lut[8] = {
    MINUTE_LOW_RD,
    MINUTE_LOW_LD,
    MINUTE_HIGH_RD,
    MINUTE_HIGH_LD,
    HOUR_LOW_RD,
    HOUR_LOW_LD,
    HOUR_HIGH_RD,
    HOUR_HIGH_LD,
};

void display_init(void)
{
    shift_reg_driver_init();
    pwm_driver_init(GPIO_OUTPUT_EN);
    ESP_LOGI(TAG, "Display initialized");
}

void display_set_brightness(uint32_t percent)
{
    pwm_driver_set_duty(percent);
}

void display_clear(void)
{
    shift_reg_driver_write(0);
}

void display_show_bitmap(uint64_t data)
{
    shift_reg_driver_write(data);
}

uint64_t display_format_time(struct tm timeinfo)
{
    uint64_t minute_high = (timeinfo.tm_min / 10 == 0)
        ? MINUTE_HIGH_0
        : (uint64_t)1 << (13 + (10 - (timeinfo.tm_min / 10)));
    uint64_t minute_low = (timeinfo.tm_min % 10 == 0)
        ? MINUTE_LOW_0
        : (uint64_t)1 << (1 + (10 - (timeinfo.tm_min % 10)));
    uint64_t hour_high = (timeinfo.tm_hour / 10 == 0)
        ? HOUR_HIGH_0
        : (uint64_t)1 << (37 + (10 - (timeinfo.tm_hour / 10)));
    uint64_t hour_low = (timeinfo.tm_hour % 10 == 0)
        ? HOUR_LOW_0
        : (uint64_t)1 << (25 + (10 - (timeinfo.tm_hour % 10)));

    return hour_high | hour_low | minute_high | minute_low;
}

void display_show_time(struct tm timeinfo)
{
    shift_reg_driver_write(display_format_time(timeinfo));
}

void display_roll(struct tm timeinfo)
{
    uint64_t time_data = display_format_time(timeinfo);
    uint64_t digit_data[4];
    uint64_t curr_time[4];

    digit_data[0] = MINUTE_LOW_LD;
    digit_data[1] = MINUTE_LOW_LD;
    digit_data[2] = MINUTE_LOW_LD;
    digit_data[3] = MINUTE_LOW_LD;

    curr_time[0] = 0xFFF & time_data;
    curr_time[1] = 0xFFF & (time_data >> 12);
    curr_time[2] = 0xFFF & (time_data >> 24);
    curr_time[3] = 0xFFF & (time_data >> 36);

    for (int i = 0; i < 22; i++) {
        digit_data[0] = (i >= 10) ? curr_time[0] : MINUTE_LOW_1 >> i;
        digit_data[1] = ((i >= 4) && (i < 13)) ? MINUTE_LOW_1 >> (i - 3) : curr_time[1];
        digit_data[2] = ((i >= 8) && (i < 17)) ? MINUTE_LOW_1 >> (i - 7) : curr_time[2];
        digit_data[3] = ((i >= 12) && (i < 21)) ? MINUTE_LOW_1 >> (i - 11) : curr_time[3];

        uint64_t tmp_data = (digit_data[3] << 36) | (digit_data[2] << 24)
                          | (digit_data[1] << 12) | digit_data[0];
        shift_reg_driver_write(tmp_data);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void display_waiting_frame(uint32_t index)
{
    if (index >= 8) {
        index = 7;
    }
    shift_reg_driver_write(dot_animation_lut[index]);
}

void display_test_loop(void)
{
    uint64_t digit_data[4] = {MINUTE_LOW_LD, MINUTE_LOW_LD, MINUTE_LOW_LD, MINUTE_LOW_LD};
    uint64_t test_data = (digit_data[3] << 36) | (digit_data[2] << 24)
                       | (digit_data[1] << 12) | digit_data[0];

    for (int i = 0; i <= 11; i++) {
        shift_reg_driver_write(test_data >> i);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void display_test_task(void *pvParameters)
{
    (void)pvParameters;
    ESP_LOGI(TAG, "Digit test loop started");
    for (;;) {
        display_test_loop();
    }
}
