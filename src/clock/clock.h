#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

void clock_start(void);
uint8_t clock_get_brightness(void);
esp_err_t clock_set_brightness(uint8_t percent);
uint8_t clock_get_night_brightness(void);
esp_err_t clock_set_night_brightness(uint8_t percent);
bool clock_get_night_enabled(void);
esp_err_t clock_set_night_enabled(bool enabled);
uint8_t clock_get_night_start(void);
uint8_t clock_get_night_end(void);
esp_err_t clock_set_night_hours(uint8_t start_hour, uint8_t end_hour);
const char *clock_get_timezone(void);
esp_err_t clock_set_timezone(const char *tz);
int clock_zone_count(void);
const char *clock_zone_id(int index);
const char *clock_zone_label(int index);
bool clock_zone_has_dst(int index);
int clock_get_zone_index(void);
bool clock_get_dst(void);
esp_err_t clock_set_zone(int index, bool dst);
esp_err_t clock_set_custom_timezone(const char *tz);
void clock_format_now(char *buf, size_t len);
bool clock_get_force_on(void);
esp_err_t clock_set_force_on(bool force);
bool clock_get_presence_enabled(void);
esp_err_t clock_set_presence_enabled(bool enabled);
uint16_t clock_get_idle_minutes(void);
esp_err_t clock_set_idle_minutes(uint16_t minutes);
uint16_t clock_get_transition_s(void);
esp_err_t clock_set_transition_s(uint16_t seconds);
void clock_test_brightness_transition(void);
uint16_t clock_get_roll_minutes(void);
esp_err_t clock_set_roll_minutes(uint16_t minutes);
const char *clock_get_ntp_primary(void);
const char *clock_get_ntp_backup(void);
bool clock_set_ntp_servers(const char *primary, const char *backup);
