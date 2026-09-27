#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "defines.h"
#include "clock.h"
#include "display.h"
#include "gpio_driver.h"
#include "psu_driver.h"

static const char *TAG = "Main";

void app_main(void)
{
    display_init();
    psu_driver_init();
    gpio_driver_init();

    display_clear();
    display_set_brightness(DISPLAY_BRIGHTNESS_DEFAULT_PERCENT);
    psu_driver_enable();

#ifdef BUILD_MAIN_PROGRAM
    clock_start();
#else
    ESP_LOGI(TAG, "Building test program");
    xTaskCreate(display_test_task, "test", 2048, NULL, 5, NULL);
#endif
}
