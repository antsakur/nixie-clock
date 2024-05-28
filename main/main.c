#include <stdio.h>
#include <string.h>
#include <math.h>
#include <inttypes.h>
#include <sys/time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "nvs_flash.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/rmt_tx.h"
#include "esp_log.h"
#include "esp_event.h"
#include "esp_netif_sntp.h"
#include "esp_wifi.h"

#include "defines.h"
#include "helper_functions.h"

time_t now;
struct tm timeinfo;

static const char *TAG = "Main";

static uint8_t CLK_DATA[6];
static uint8_t serial_data[6];
static uint64_t serial_tm_data;

// Flag for indicating end of WIFI/SNTP setup phase
static bool setup_done = false;

// RMT channel configurations
rmt_channel_handle_t SRDATA_channel = NULL;
rmt_tx_channel_config_t SRDATA_channel_config = {
.clk_src = RMT_CLK_SRC_DEFAULT,   // select source clock
.gpio_num = GPIO_SRDATA,          // GPIO number
.mem_block_symbols = 48,          // memory block size, 64 * 4 = 256 Bytes
.resolution_hz = RMT_RESOLUTION_HZ,
.trans_queue_depth = 1,           // set the number of transactions that can pend in the background
.flags.invert_out = true,         // invert output signal
.flags.with_dma = false,          // do not need DMA backend
};

rmt_channel_handle_t SRCLK_channel = NULL;
rmt_tx_channel_config_t SRCLK_channel_config = {
.clk_src = RMT_CLK_SRC_DEFAULT,   // select source clock
.gpio_num = GPIO_SRCLK,           // GPIO number
.mem_block_symbols = 48,          // memory block size, 64 * 4 = 256 Bytes
.resolution_hz = RMT_RESOLUTION_HZ,
.trans_queue_depth = 1,           // set the number of transactions that can pend in the background
.flags.invert_out = true,         // invert output signal
.flags.with_dma = false,          // do not need DMA backend
};

rmt_encoder_handle_t SRDATA_encoder = NULL;
rmt_bytes_encoder_config_t SRDATA_encoder_config = {
    .bit0 = {
        .level0 = 0,
        .duration0 = 2, // 1us
        .level1 = 0,
        .duration1 = 2, // 1us
    },
    .bit1 = {
        .level0 = 1,
        .duration0 = 2,  // 1us
        .level1 = 1,
        .duration1 = 2,  // 1us -> Period of 4us, f = 250 kHz
    },
    .flags.msb_first = 0
};

// Clock signal is shifted in time to ensure sufficient holding time for the sample
rmt_encoder_handle_t SRCLK_encoder = NULL;
rmt_bytes_encoder_config_t SRCLK_encoder_config = {
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
    .flags.msb_first = 0
};

rmt_transmit_config_t SRDATA_transmit_config = {
    .loop_count = 0,
    .flags.eot_level = 0,
};

rmt_transmit_config_t SRCLK_transmit_config = {
    .loop_count = 0,
    .flags.eot_level = 0,
};

rmt_sync_manager_handle_t synchro = NULL;


// init timer handles
TimerHandle_t xTimer[3];

// Waiting animation task handle
static TaskHandle_t xHandle_waiting_animation = NULL;

// Declare handler for setup phase event group
// Used to stop waiting animation task and
// to inform main task when waiting animation task has stopped
static EventGroupHandle_t xSetup_event_group;
static EventBits_t xSetup_event_group_bits;

// Declare handler for GPIO interrupt queue
static QueueHandle_t xGpio_evt_queue = NULL;

// FreeRTOS event group to signal when we are connected to Wi-Fi
static EventGroupHandle_t xWifi_event_group;
static int wifi_retry_num = 0;

// configure GPIOs
static void configure_gpio();

// initialize PWM
static void init_pwm(int GPIO_NUM);

// Set PWM duty-cycle (0-100) of the shift register output enable
static void set_shift_reg_output_pwm(uint32_t duty_cycle);

// init RMT perihperal
static void init_rmt();

// Timer callback function
static void vTimerCallBack(TimerHandle_t timer);

// GPIO ISR handler
static void IRAM_ATTR gpio_isr_handler(void* arg);

// GPIO ISR task
void gpio_isr_task(void* arg);

// Connect to Wi-Fi
static esp_err_t connect_wifi();

// Wi-Fi event handler
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data);

// NTP time sync notification message
static void time_sync_notification_cb(struct timeval *tv);

// Write data to shift registers
static void write_shift_reg(uint64_t data);

// Sequentially roll digits from 0 to 9, leaves digits in state specified in time_data
static void roll_display(uint64_t time_data);

// Task for executing waiting animation functionality
TaskFunction_t waiting_animation_task();

// Loop all digits once
static void digitTestLoop();


void app_main(void)
{
    memset(&CLK_DATA, 0xFF, sizeof(CLK_DATA));

    // configure GPIOs
    configure_gpio();

    // initialize PWM GPIO
    init_pwm(GPIO_OUTPUT_EN);

    // initialize RMT components used for writing data to shift registers
    init_rmt();

    // clear shift registers
    write_shift_reg(0x0ULL);

    // enable shift register output
    set_shift_reg_output_pwm(80);

    // enable HV PSU
    gpio_set_level(GPIO_PSU_EN, 1);

    #ifdef BUILD_MAIN_PROGRAM
    xSetup_event_group = xEventGroupCreate();

    // create task for waiting animation
    xTaskCreate(waiting_animation_task, "waiting_animation_task", 2048, NULL, 10, &xHandle_waiting_animation);

    ESP_ERROR_CHECK(nvs_flash_init()); // Initialize non-volatile flash
    ESP_ERROR_CHECK(esp_netif_init()); // Initialize TCP/IP stack
    ESP_ERROR_CHECK(esp_event_loop_create_default()); // Event loop for Wi-Fi events

    // Set timezone enviroment variable
    setenv("TZ", "EET-2EEST,M3.5.0/3,M10.5.0/4", 1);
    tzset();

    // Set SNTP time server
    // Synchronizing interval is set in in the menuconfig
    esp_sntp_config_t config = ESP_NETIF_SNTP_DEFAULT_CONFIG(SNTP_TIME_SERVER);
    config.start = false;
    config.sync_cb = time_sync_notification_cb;
    esp_netif_sntp_init(&config);

    // connect to wifi
    ESP_ERROR_CHECK(connect_wifi());

    // start sntp service
    esp_netif_sntp_start();
    int retry = 0;
    const int retry_count = 15;
    while (esp_netif_sntp_sync_wait(2000 / portTICK_PERIOD_MS) == ESP_ERR_TIMEOUT && ++retry < retry_count) {
        ESP_LOGI(TAG, "Waiting for the time to be synced... (%d/%d)", retry, retry_count);
    };

    // Update 'now' variable with current time
    time(&now);
    // Format calendar time to 'timeinfo' struct
    localtime_r(&now, &timeinfo);
    
    char strftime_buf[64];
    strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
    ESP_LOGI(TAG, "The current date/time is: %s", strftime_buf);

    // Timer for updating time displayed
    xTimer[0] = xTimerCreate(  "timer1", 
                                (60000 / portTICK_PERIOD_MS), // 1 min
                                pdTRUE,
                                (void*) TIMER_UPDATE_DISP,
                                vTimerCallBack
                            );

    // Timer for rolling all digits of the display
    xTimer[1] = xTimerCreate(  "timer2", 
                                (300000  / portTICK_PERIOD_MS), // 5 min
                                pdTRUE,
                                (void*) TIMER_ROLL_DISP,
                                vTimerCallBack
                            );

    // Timer for turning the digits off
    xTimer[2] = xTimerCreate(  "timer3", 
                                (600000  / portTICK_PERIOD_MS), // 10 min
                                pdFALSE,
                                (void*) TIMER_NO_MOVEMENT,
                                vTimerCallBack
                            );


    // setup done
    // inform setup waiting animation task to stop and delete itself
    xEventGroupSetBits(xSetup_event_group, MAIN_TASK_SETUP_DONE);

    // wait for the animation task to stop
    xSetup_event_group_bits = xEventGroupWaitBits(xSetup_event_group,
        ANIMATION_TASK_DONE,
        pdFALSE,
        pdFALSE,
        portMAX_DELAY);

    // check that the bits were set
    if ( !(xSetup_event_group_bits & ANIMATION_TASK_DONE) )
        ESP_LOGE(TAG, "Event group bits not correctly set by animation task");

    // Update current time to display
    serial_tm_data = formatTime(timeinfo);
    // Ignore first write as it writes carbage for some reason
    write_shift_reg(serial_tm_data);
    // Second write works fine
    write_shift_reg(serial_tm_data);

    // define a queue to handle GPIO event from ISR
    xGpio_evt_queue = xQueueCreate(2, sizeof(uint32_t));
    // install GPIO ISR service
    gpio_install_isr_service(0);
    // hook ISR handler for specific GPIO pin
    gpio_isr_handler_add(GPIO_PRESENCE_SENSOR, gpio_isr_handler, (void*) GPIO_PRESENCE_SENSOR);
    // start GPIO ISR task
    xTaskCreate(gpio_isr_task, "gpio_isr_task", 2048, NULL, 10, NULL);

    // start timers
    xTimerStart(xTimer[0], 0);
    xTimerStart(xTimer[1], 0);

    // NO_MOVEMENT_DETECTED timer is started only if movement isn't detected
    if ( gpio_get_level(GPIO_PRESENCE_SENSOR) == 0) {
        ESP_LOGI(TAG, "Movement not detected on init phase");
        xTimerStart(xTimer[2], 0);
    }

    #else
        ESP_LOGI(TAG, "Test program loop started");
        for (;;) 
            digitTestLoop();
    #endif
}

static void configure_gpio()
{
    // Zero-initialize the config structure.
    gpio_config_t io_conf = {};

    //// USB VBUS CC1
    //io_conf.pin_bit_mask = (1ULL<<GPIO_VBUS_CC1);
    //io_conf.intr_type = GPIO_INTR_DISABLE;
    //io_conf.mode = GPIO_MODE_INPUT;
    //io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    //io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    //gpio_config(&io_conf);

    // USB VBUS CC2
    //io_conf.pin_bit_mask = (1ULL<<GPIO_VBUS_CC2);
    //io_conf.intr_type = GPIO_INTR_DISABLE;
    //io_conf.mode = GPIO_MODE_INPUT;
    //io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    //io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    //gpio_config(&io_conf);

    // Presence sensor OUT
    io_conf.pin_bit_mask = (1ULL<<GPIO_PRESENCE_SENSOR);
    io_conf.intr_type = GPIO_INTR_ANYEDGE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_config(&io_conf);

    // UART RX
    //io_conf.pin_bit_mask = (1ULL<<GPIO_UART_RX);
    //io_conf.intr_type = GPIO_INTR_DISABLE;
    //io_conf.mode = GPIO_MODE_OUTPUT;
    //io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    //io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
    //gpio_config(&io_conf);

    // UART TX
    //io_conf.pin_bit_mask = (1ULL<<GPIO_UART_TX);
    //io_conf.intr_type = GPIO_INTR_DISABLE;
    //io_conf.mode = GPIO_MODE_INPUT;
    //io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    //io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    //gpio_config(&io_conf);

    // HV PSU enable, pulldown disables the PSU
    io_conf.pin_bit_mask = (1ULL<<GPIO_PSU_EN);
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_config(&io_conf);

    // Shift register latch
    io_conf.pin_bit_mask = (1ULL<<GPIO_RCLK);
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_config(&io_conf);
}

static void init_pwm(int GPIO_NUM)
{
    ledc_timer_config_t pwm_timer = {
    .duty_resolution = PWM_RESOLUTION_BITS, // resolution of PWM duty-cycle
    .freq_hz         = 30000,               // frequency of PWM signal
    .speed_mode      = LEDC_LOW_SPEED_MODE, // timer mode
    .timer_num       = LEDC_TIMER_0,        // timer index
    .clk_cfg         = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&pwm_timer));

    ledc_channel_config_t pwm_channel_config = {
        .gpio_num   = GPIO_NUM,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = LEDC_CHANNEL_0,
        .intr_type  = LEDC_INTR_DISABLE,
        .timer_sel  = LEDC_TIMER_0,
        .duty       = 0,
        .hpoint     = 0,
        .flags.output_invert = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&pwm_channel_config));

    // install fade function to be able to change PWM duty-cycle on the fly
    ESP_ERROR_CHECK(ledc_fade_func_install(0));
}

static void set_shift_reg_output_pwm(uint32_t duty_cycle)
{
    // scale duty-cycle from 0-100 to 0-2**PWM_RESOLUTION_BITS
    float duty_cycle_scaled = (float) duty_cycle/100 * pow(2, PWM_RESOLUTION_BITS);

    // update duty-cycle
    ESP_ERROR_CHECK(ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE,
                                            LEDC_CHANNEL_0,
                                            (uint32_t)duty_cycle_scaled,
                                            0));
}

static void gpio_isr_handler(void* arg)
{
    int32_t gpio_level = gpio_get_level(GPIO_PRESENCE_SENSOR);
    xQueueSendFromISR(xGpio_evt_queue, &gpio_level, NULL);
}

void gpio_isr_task(void* arg)
{
    int32_t gpio_level;

    for (;;) {
        if (xQueueReceive(xGpio_evt_queue, &gpio_level, portMAX_DELAY)) {

            ESP_LOGI(TAG, "IRQ triggered, presence sensor level: %ld", gpio_level);

            if ( gpio_level == 1) {
                // turn on the HV PSU on raising edge
                gpio_set_level(GPIO_PSU_EN, 1);

                // stop the NO_MOVEMENT_DETECTED timer
                xTimerStopFromISR(xTimer[2], NULL);
            }
            else {
                // reactivate the timer on falling edge
                if ( xTimerResetFromISR( xTimer[2], NULL ) != pdPASS ) {

                    // The reset command was not executed successfully
                    ESP_LOGE(TAG, "Error while resetting timer %d", TIMER_NO_MOVEMENT);
                }

            }
        }
    }
}

static void vTimerCallBack(TimerHandle_t timer)
{
    uint32_t timer_index;

    // get current systemtime
    time(&now);
    localtime_r(&now, &timeinfo);
    serial_tm_data = formatTime(timeinfo);

    // get index of the expired timer
    timer_index = (uint32_t) pvTimerGetTimerID(timer);
    ESP_LOGI( TAG, "Timer callback triggered by timer %li", timer_index );

    switch ( timer_index ) {
        case TIMER_UPDATE_DISP:
            // Write updated time to shift registers
            write_shift_reg(serial_tm_data);
            break;

        case TIMER_ROLL_DISP:
            // Roll all digits from 0 to 9 to prevent cathode poisoning
            roll_display(serial_tm_data);
            break;

        case TIMER_NO_MOVEMENT:
            // Turn off the digits by disabling HV PSU
            gpio_set_level(GPIO_PSU_EN, 0);
            break;

        default:
    }
}

static void time_sync_notification_cb(struct timeval *tv)
{
    ESP_LOGI(TAG, "Notification of a time synchronization event");
}

static esp_err_t connect_wifi()
{
    esp_err_t status;

    esp_netif_create_default_wifi_sta();

    wifi_init_config_t wifi_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wifi_cfg));

    xWifi_event_group = xEventGroupCreate();

    esp_event_handler_instance_t wifi_event_instance;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &wifi_event_instance));

    esp_event_handler_instance_t got_ip_event_instance;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &got_ip_event_instance));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = ESP_WIFI_SSID,
            .password = ESP_WIFI_PASS,
            .threshold.authmode = ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD,
            .sae_pwe_h2e = ESP_WIFI_SAE_MODE,
            .sae_h2e_identifier = H2E_IDENTIFIER,
        },
    };

    // Set wifi controller to station mode
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

    // set the wifi config
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));

    // start the wifi driver
    ESP_ERROR_CHECK(esp_wifi_start());

    // Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
    // number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above)
    EventBits_t bits = xEventGroupWaitBits(xWifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);           

    // xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
    // happened.
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Connected to AP SSID:%s password:%s", ESP_WIFI_SSID, ESP_WIFI_PASS);
        status = ESP_OK;
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s", ESP_WIFI_SSID, ESP_WIFI_PASS);
        status = ESP_ERR_WIFI_NOT_STARTED;
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
        status = ESP_ERR_WIFI_NOT_STARTED;
    }

    return status;
}

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (wifi_retry_num  < ESP_MAXIMUM_RETRY) {
            esp_wifi_connect();
            wifi_retry_num ++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(xWifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        wifi_retry_num  = 0;
        xEventGroupSetBits(xWifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static void init_rmt()
{
    // create channels and encoders
    ESP_ERROR_CHECK(rmt_new_tx_channel(&SRDATA_channel_config, &SRDATA_channel));
    ESP_ERROR_CHECK(rmt_new_tx_channel(&SRCLK_channel_config, &SRCLK_channel));
    ESP_ERROR_CHECK(rmt_new_bytes_encoder(&SRDATA_encoder_config, &SRDATA_encoder));
    ESP_ERROR_CHECK(rmt_new_bytes_encoder(&SRCLK_encoder_config, &SRCLK_encoder));
    ESP_ERROR_CHECK(rmt_enable(SRDATA_channel));
    ESP_ERROR_CHECK(rmt_enable(SRCLK_channel));

    // synchronize channels
    rmt_channel_handle_t channels[2] = {SRDATA_channel, SRCLK_channel};
    rmt_sync_manager_config_t synchro_config = {
    .tx_channel_array = channels,
    .array_size = sizeof(channels) / sizeof(channels[0]),
    };
    ESP_ERROR_CHECK(rmt_new_sync_manager(&synchro_config, &synchro));
}

void write_shift_reg(uint64_t data)
{
        // convert uint64_t to bytes
        for (int i = 0; i < 6; i++) {
            serial_data[i] = (uint8_t) (data >> (i*8));
        }

        // send data, RMT payload is passed as reference to array consisting of bytes
        rmt_sync_reset(synchro);
        rmt_transmit(SRCLK_channel, SRCLK_encoder, &CLK_DATA, sizeof(CLK_DATA), &SRCLK_transmit_config);
        rmt_transmit(SRDATA_channel, SRDATA_encoder, &serial_data, sizeof(serial_data), &SRDATA_transmit_config);

        rmt_tx_wait_all_done(SRDATA_channel, portMAX_DELAY);

        // latch the data
        gpio_set_level(GPIO_RCLK, 0);
        esp_rom_delay_us(10);
        gpio_set_level(GPIO_RCLK, 1);
}

static void roll_display(uint64_t time_data)
{
    uint64_t tmp_data = 0;
    uint64_t digit_data[4];
    uint64_t curr_time[4];
    
    // rolling starts from digit 0
    digit_data[0] = MINUTE_LOW_LD;
    digit_data[1] = MINUTE_LOW_LD;
    digit_data[2] = MINUTE_LOW_LD;
    digit_data[3] = MINUTE_LOW_LD;

    // save current digits
    curr_time[0] = 0xFFF & time_data;
    curr_time[1] = 0xFFF & (time_data >> 12);
    curr_time[2] = 0xFFF & (time_data >> 24);
    curr_time[3] = 0xFFF & (time_data >> 36);

        for (int i = 0; i < 22; i++) {

            digit_data[0] = (i >= 10) ? curr_time[0] : MINUTE_LOW_1 >> i;
            digit_data[1] = ((i >= 4) && (i < 13)) ? MINUTE_LOW_1 >> (i-3) : curr_time[1];
            digit_data[2] = ((i >= 8) && (i < 17)) ? MINUTE_LOW_1 >> (i-7) : curr_time[2];
            digit_data[3] = ((i >= 12) && (i < 21)) ? MINUTE_LOW_1 >> (i-11) : curr_time[3];

            tmp_data = (digit_data[3] << 36) + (digit_data[2] << 24) + (digit_data[1] << 12) + (digit_data[0]);

            write_shift_reg(tmp_data);
            esp_rom_delay_us(50000);
        }
}

// loop all digits once
static void digitTestLoop() {

    uint64_t test_data = 0;
    uint64_t digit_data[4];
    
    // rolling starts from digit 0
    digit_data[0] = MINUTE_LOW_LD;
    digit_data[1] = MINUTE_LOW_LD;
    digit_data[2] = MINUTE_LOW_LD;
    digit_data[3] = MINUTE_LOW_LD;

    test_data = (digit_data[3] << 36) + (digit_data[2] << 24) + (digit_data[1] << 12) + (digit_data[0]);

        for (int i = 0; i <= 11; i++) {
            write_shift_reg(test_data >> i);
            esp_rom_delay_us(500000);
        }
   
}

TaskFunction_t waiting_animation_task()
{
    uint32_t dot_index = 0;

    // loop direction flag
    // 0 if current direction is from right to left
    // else 1
    bool dir_left = 1;

    for (;;) {

        xSetup_event_group_bits =  xEventGroupGetBits( xSetup_event_group );

        if ( xSetup_event_group_bits & MAIN_TASK_SETUP_DONE ) {
            // Delete task if setup phase is over in main()
            xEventGroupSetBits(xSetup_event_group, ANIMATION_TASK_DONE);
            vTaskDelete( NULL );
        }
        else {
            // Else perform waiting animation
            write_shift_reg( dot_animation_LUT[dot_index] );

            (( dot_index < 7 ) & dir_left) ? ++dot_index : --dot_index;
            dir_left = ( dot_index == 7 ) ? 0 : ( dot_index == 0 ) ? 1 : dir_left;

            // 100ms delay
            vTaskDelay( 100 / portTICK_PERIOD_MS );
        }
    }    
}