#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"

#include "defines.h"
#include "presence_driver.h"

static const char *TAG = "Presence";

static QueueHandle_t presence_queue;
static presence_cb_t presence_cb;

static void IRAM_ATTR presence_isr(void *arg)
{
    (void)arg;
    int32_t level = gpio_get_level(GPIO_PRESENCE_SENSOR);
    BaseType_t woken = pdFALSE;
    xQueueSendFromISR(presence_queue, &level, &woken);
    portYIELD_FROM_ISR(woken);
}

static void presence_task(void *arg)
{
    (void)arg;
    int32_t level;
    int last_reported = -1;

    for (;;) {
        if (!xQueueReceive(presence_queue, &level, portMAX_DELAY)) {
            continue;
        }

        vTaskDelay(pdMS_TO_TICKS(PRESENCE_DEBOUNCE_MS));
        int stable = gpio_get_level(GPIO_PRESENCE_SENSOR);
        if (stable == last_reported) {
            continue;
        }
        last_reported = stable;

        ESP_LOGI(TAG, "Presence %s", stable ? "detected" : "cleared");
        if (presence_cb) {
            presence_cb(stable != 0);
        }
    }
}

void presence_driver_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << GPIO_PRESENCE_SENSOR),
        .intr_type = GPIO_INTR_ANYEDGE,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };
    gpio_config(&io_conf);
    ESP_LOGI(TAG, "Presence sensor on GPIO%d", GPIO_PRESENCE_SENSOR);
}

void presence_driver_start(presence_cb_t callback)
{
    presence_cb = callback;
    presence_queue = xQueueCreate(4, sizeof(int32_t));
    configASSERT(presence_queue);

    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(GPIO_PRESENCE_SENSOR, presence_isr, NULL));
    xTaskCreate(presence_task, "presence", 2048, NULL, 10, NULL);
}

int presence_driver_get(void)
{
    return gpio_get_level(GPIO_PRESENCE_SENSOR);
}
