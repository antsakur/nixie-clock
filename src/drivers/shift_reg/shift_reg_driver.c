#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/rmt_tx.h"
#include "esp_log.h"
#include "esp_rom_sys.h"

#include "defines.h"
#include "shift_reg_driver.h"

static const char *TAG = "ShiftReg";

static uint8_t CLK_DATA[6];
static uint8_t serial_data[6];
static SemaphoreHandle_t write_mutex;

static rmt_channel_handle_t SRDATA_channel = NULL;
static rmt_tx_channel_config_t SRDATA_channel_config = {
    .clk_src = RMT_CLK_SRC_DEFAULT,
    .gpio_num = GPIO_SRDATA,
    .mem_block_symbols = 48,
    .resolution_hz = RMT_RESOLUTION_HZ,
    .trans_queue_depth = 1,
    .flags.invert_out = true,
    .flags.with_dma = false,
};

static rmt_channel_handle_t SRCLK_channel = NULL;
static rmt_tx_channel_config_t SRCLK_channel_config = {
    .clk_src = RMT_CLK_SRC_DEFAULT,
    .gpio_num = GPIO_SRCLK,
    .mem_block_symbols = 48,
    .resolution_hz = RMT_RESOLUTION_HZ,
    .trans_queue_depth = 1,
    .flags.invert_out = true,
    .flags.with_dma = false,
};

static rmt_encoder_handle_t SRDATA_encoder = NULL;
static rmt_bytes_encoder_config_t SRDATA_encoder_config = {
    .bit0 = {
        .level0 = 0,
        .duration0 = 2,
        .level1 = 0,
        .duration1 = 2,
    },
    .bit1 = {
        .level0 = 1,
        .duration0 = 2,
        .level1 = 1,
        .duration1 = 2,
    },
    .flags.msb_first = 0,
};

static rmt_encoder_handle_t SRCLK_encoder = NULL;
static rmt_bytes_encoder_config_t SRCLK_encoder_config = {
    .bit0 = {
        .level0 = 0,
        .duration0 = 1,
        .level1 = 0,
        .duration1 = 1,
    },
    .bit1 = {
        .level0 = 0,
        .duration0 = 2,
        .level1 = 1,
        .duration1 = 2,
    },
    .flags.msb_first = 0,
};

static rmt_transmit_config_t SRDATA_transmit_config = {
    .loop_count = 0,
    .flags.eot_level = 0,
};

static rmt_transmit_config_t SRCLK_transmit_config = {
    .loop_count = 0,
    .flags.eot_level = 0,
};

static rmt_sync_manager_handle_t synchro = NULL;

static void shift_reg_transmit(uint64_t data)
{
    for (int i = 0; i < 6; i++) {
        serial_data[i] = (uint8_t)(data >> (i * 8));
    }

    ESP_ERROR_CHECK(rmt_sync_reset(synchro));
    ESP_ERROR_CHECK(rmt_transmit(SRCLK_channel, SRCLK_encoder, CLK_DATA, sizeof(CLK_DATA),
                                 &SRCLK_transmit_config));
    ESP_ERROR_CHECK(rmt_transmit(SRDATA_channel, SRDATA_encoder, serial_data, sizeof(serial_data),
                                 &SRDATA_transmit_config));

    ESP_ERROR_CHECK(rmt_tx_wait_all_done(SRDATA_channel, portMAX_DELAY));
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(SRCLK_channel, portMAX_DELAY));

    gpio_set_level(GPIO_RCLK, 0);
    esp_rom_delay_us(10);
    gpio_set_level(GPIO_RCLK, 1);
}

void shift_reg_driver_init(void)
{
    memset(CLK_DATA, 0xFF, sizeof(CLK_DATA));
    write_mutex = xSemaphoreCreateMutex();
    configASSERT(write_mutex);

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << GPIO_RCLK),
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };
    gpio_config(&io_conf);
    gpio_set_level(GPIO_RCLK, 0);

    ESP_ERROR_CHECK(rmt_new_tx_channel(&SRDATA_channel_config, &SRDATA_channel));
    ESP_ERROR_CHECK(rmt_new_tx_channel(&SRCLK_channel_config, &SRCLK_channel));
    ESP_ERROR_CHECK(rmt_new_bytes_encoder(&SRDATA_encoder_config, &SRDATA_encoder));
    ESP_ERROR_CHECK(rmt_new_bytes_encoder(&SRCLK_encoder_config, &SRCLK_encoder));
    ESP_ERROR_CHECK(rmt_enable(SRDATA_channel));
    ESP_ERROR_CHECK(rmt_enable(SRCLK_channel));

    rmt_channel_handle_t channels[2] = {SRDATA_channel, SRCLK_channel};
    rmt_sync_manager_config_t synchro_config = {
        .tx_channel_array = channels,
        .array_size = sizeof(channels) / sizeof(channels[0]),
    };
    ESP_ERROR_CHECK(rmt_new_sync_manager(&synchro_config, &synchro));

    shift_reg_transmit(0);

    ESP_LOGI(TAG, "Shift register driver initialized (RMT + latch on GPIO%d)", GPIO_RCLK);
}

void shift_reg_driver_write(uint64_t data)
{
    xSemaphoreTake(write_mutex, portMAX_DELAY);
    shift_reg_transmit(data);
    xSemaphoreGive(write_mutex);
}
