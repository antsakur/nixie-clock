#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

typedef struct {
    uint8_t day_brightness;
    uint8_t night_brightness;
    bool night_enabled;
    uint8_t night_start;
    uint8_t night_end;
    uint16_t transition_s;
} clock_brightness_settings_t;

void clock_start(void);
esp_err_t clock_set_brightness_settings(const clock_brightness_settings_t *settings);
uint8_t clock_get_brightness(void);
uint8_t clock_get_night_brightness(void);
bool clock_get_night_enabled(void);
uint8_t clock_get_night_start(void);
uint8_t clock_get_night_end(void);
int clock_zone_count(void);
const char *clock_zone_id(int index);
const char *clock_zone_label(int index);
bool clock_zone_has_dst(int index);
int clock_get_zone_index(void);
bool clock_get_dst(void);
esp_err_t clock_set_zone(int index, bool dst);
void clock_format_now(char *buf, size_t len);
bool clock_get_presence_enabled(void);
esp_err_t clock_set_presence_settings(bool enabled, uint16_t idle_minutes);
uint16_t clock_get_idle_minutes(void);
uint16_t clock_get_transition_s(void);
void clock_test_brightness_transition(void);
uint16_t clock_get_roll_minutes(void);
esp_err_t clock_set_roll_minutes(uint16_t minutes);
const char *clock_get_ntp_primary(void);
const char *clock_get_ntp_backup(void);
