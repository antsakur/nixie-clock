#pragma once

#include "driver/gpio.h"

#if CONFIG_ESP32_C6_WROOM_1
    #define GPIO_PRESENCE_SENSOR GPIO_NUM_0
    #define GPIO_VBUS_CC2        GPIO_NUM_1
    #define GPIO_VBUS_CC1        GPIO_NUM_2
    #define GPIO_UART_RX         GPIO_NUM_6
    #define GPIO_UART_TX         GPIO_NUM_7
    #define GPIO_PSU_EN          GPIO_NUM_11
    #define GPIO_SHIFT_REG_LATCH          GPIO_NUM_18
    #define GPIO_SHIFT_REG_DATA           GPIO_NUM_19
    #define GPIO_SHIFT_REG_CLOCK          GPIO_NUM_20
    #define GPIO_SHIFT_REG_OUTPUT_ENABLE  GPIO_NUM_21
#elif CONFIG_ESP32_C6_MINI_1
    #define GPIO_PRESENCE_SENSOR GPIO_NUM_0
    #define GPIO_UART_RX         GPIO_NUM_4
    #define GPIO_UART_TX         GPIO_NUM_5
    #define GPIO_PSU_EN          GPIO_NUM_7
    #define GPIO_SHIFT_REG_LATCH          GPIO_NUM_15
    #define GPIO_SHIFT_REG_DATA           GPIO_NUM_18
    #define GPIO_SHIFT_REG_CLOCK          GPIO_NUM_19
    #define GPIO_SHIFT_REG_OUTPUT_ENABLE  GPIO_NUM_20
#else
    #define GPIO_PRESENCE_SENSOR GPIO_NUM_0
    #define GPIO_UART_RX         GPIO_NUM_6
    #define GPIO_UART_TX         GPIO_NUM_7
    #define GPIO_PSU_EN          GPIO_NUM_11
    #define GPIO_SHIFT_REG_LATCH          GPIO_NUM_18
    #define GPIO_SHIFT_REG_DATA           GPIO_NUM_19
    #define GPIO_SHIFT_REG_CLOCK          GPIO_NUM_20
    #define GPIO_SHIFT_REG_OUTPUT_ENABLE  GPIO_NUM_21
#endif

#define RMT_RESOLUTION_HZ 1000000
#define PWM_RESOLUTION_BITS 10

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

#define TIMER_TICK        1
#define TIMER_ROLL_DISP   2
#define TIMER_NO_MOVEMENT 3

#define ESP_MAXIMUM_RETRY CONFIG_ESP_MAXIMUM_RETRY
#define SNTP_TIME_SERVER CONFIG_SNTP_TIME_SERVER
#define SNTP_TIME_SERVER_BACKUP CONFIG_SNTP_TIME_SERVER_BACKUP

#define PRESENCE_DEBOUNCE_MS 50
#define DISPLAY_BRIGHTNESS_DEFAULT_PERCENT 80
