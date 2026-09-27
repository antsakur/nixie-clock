#include <stdint.h>
#include <sys/time.h>
#include <time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "freertos/timers.h"

#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_netif_sntp.h"

#include "defines.h"
#include "clock.h"
#include "display.h"
#include "gpio_driver.h"
#include "psu_driver.h"
#include "web_server.h"
#include "wifi_driver.h"

static const char *TAG = "Clock";

static TimerHandle_t timers[3];
static EventGroupHandle_t setup_events;
static TaskHandle_t roll_task_handle;
static volatile bool rolling;

static void nvs_init(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS needs erase (%s)", esp_err_to_name(err));
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
}

static void local_time(struct tm *out)
{
    time_t now;
    time(&now);
    localtime_r(&now, out);
}

static void time_sync_notification_cb(struct timeval *tv)
{
    (void)tv;
    ESP_LOGI(TAG, "SNTP time synchronized");
}

static void presence_changed(bool present)
{
    if (present) {
        psu_driver_enable();
        xTimerStop(timers[2], 0);
    } else if (xTimerReset(timers[2], 0) != pdPASS) {
        ESP_LOGE(TAG, "Failed to reset idle timer");
    }
}

static void roll_task(void *arg)
{
    (void)arg;
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        rolling = true;
        struct tm timeinfo;
        local_time(&timeinfo);
        display_roll(timeinfo);
        rolling = false;
        local_time(&timeinfo);
        display_show_time(timeinfo);
    }
}

static void timer_callback(TimerHandle_t timer)
{
    uint32_t id = (uint32_t)(uintptr_t)pvTimerGetTimerID(timer);

    switch (id) {
    case TIMER_TICK: {
        static int last_hour = -1;
        static int last_min = -1;
        struct tm timeinfo;
        local_time(&timeinfo);
        if (!rolling && (timeinfo.tm_hour != last_hour || timeinfo.tm_min != last_min)) {
            last_hour = timeinfo.tm_hour;
            last_min = timeinfo.tm_min;
            display_show_time(timeinfo);
        }
        break;
    }
    case TIMER_ROLL_DISP:
        if (roll_task_handle) {
            xTaskNotifyGive(roll_task_handle);
        }
        break;
    case TIMER_NO_MOVEMENT:
        psu_driver_disable();
        break;
    default:
        break;
    }
}

static void waiting_animation_task(void *pvParameters)
{
    (void)pvParameters;
    uint32_t dot_index = 0;
    bool dir_left = true;

    for (;;) {
        if (xEventGroupGetBits(setup_events) & MAIN_TASK_SETUP_DONE) {
            xEventGroupSetBits(setup_events, ANIMATION_TASK_DONE);
            vTaskDelete(NULL);
        }

        display_waiting_frame(dot_index);
        if (dot_index < 7 && dir_left) {
            ++dot_index;
        } else {
            --dot_index;
        }
        dir_left = (dot_index == 7) ? false : (dot_index == 0) ? true : dir_left;
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

static void setup_task(void *pvParameters)
{
    (void)pvParameters;

    nvs_init();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    setenv("TZ", "EET-2EEST,M3.5.0/3,M10.5.0/4", 1);
    tzset();

    esp_sntp_config_t config = ESP_NETIF_SNTP_DEFAULT_CONFIG(SNTP_TIME_SERVER);
    config.start = false;
    config.sync_cb = time_sync_notification_cb;
    esp_netif_sntp_init(&config);

    esp_err_t wifi_ok = wifi_driver_connect();
    if (wifi_ok == ESP_OK) {
        web_server_start();
        esp_netif_sntp_start();
        int retry = 0;
        const int retry_count = 15;
        while (esp_netif_sntp_sync_wait(pdMS_TO_TICKS(2000)) == ESP_ERR_TIMEOUT && ++retry < retry_count) {
            ESP_LOGI(TAG, "Waiting for time sync... (%d/%d)", retry, retry_count);
        }
    } else {
        ESP_LOGW(TAG, "Starting without network; tubes will show unsynced time until Wi-Fi connects");
    }

    struct tm timeinfo;
    local_time(&timeinfo);
    char strftime_buf[64];
    strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
    ESP_LOGI(TAG, "Current date/time: %s", strftime_buf);

    timers[0] = xTimerCreate("tick", pdMS_TO_TICKS(1000), pdTRUE,
                             (void *)(uintptr_t)TIMER_TICK, timer_callback);
    timers[1] = xTimerCreate("roll", pdMS_TO_TICKS(300000), pdTRUE,
                             (void *)(uintptr_t)TIMER_ROLL_DISP, timer_callback);
    timers[2] = xTimerCreate("idle", pdMS_TO_TICKS(600000), pdFALSE,
                             (void *)(uintptr_t)TIMER_NO_MOVEMENT, timer_callback);

    xTaskCreate(roll_task, "roll", 2048, NULL, 5, &roll_task_handle);

    xEventGroupSetBits(setup_events, MAIN_TASK_SETUP_DONE);
    EventBits_t bits = xEventGroupWaitBits(setup_events, ANIMATION_TASK_DONE,
                                           pdFALSE, pdFALSE, portMAX_DELAY);
    if (!(bits & ANIMATION_TASK_DONE)) {
        ESP_LOGE(TAG, "Waiting animation did not stop cleanly");
    }

    display_show_time(timeinfo);

    gpio_driver_presence_start(presence_changed);

    xTimerStart(timers[0], 0);
    xTimerStart(timers[1], 0);
    if (gpio_driver_presence_get() == 0) {
        ESP_LOGI(TAG, "No presence at init; starting idle timer");
        xTimerStart(timers[2], 0);
    }

    vTaskDelete(NULL);
}

void clock_start(void)
{
    setup_events = xEventGroupCreate();
    configASSERT(setup_events);
    xTaskCreate(waiting_animation_task, "waiting", 2048, NULL, 10, NULL);
    xTaskCreate(setup_task, "setup", 4096, NULL, 5, NULL);
}
