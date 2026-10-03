#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <time.h>

#include "esp_err.h"

typedef struct {
    int32_t digit_duration_ms;
    int32_t run_duration_ms;
    uint8_t offset;
    bool inverse_direction;
} display_poison_cfg_t;

typedef struct {
    int32_t digit_duration_ms;
    int32_t run_duration_ms;
} display_random_cfg_t;

typedef struct {
    display_poison_cfg_t poison;
    display_random_cfg_t random;
    int32_t fade_ms;
    bool fade_enabled;
} display_settings_t;

void display_init(void);
void display_settings_load(void);
esp_err_t display_set_settings(const display_settings_t *settings);
void display_set_brightness(uint32_t percent);
void display_set_brightness_counts(uint32_t duty);
void display_clear(void);

uint64_t display_format_time(struct tm timeinfo);

bool display_is_showing_time(void);

void display_get_poison(display_poison_cfg_t *out);
esp_err_t display_set_poison(const display_poison_cfg_t *cfg);
void display_get_random(display_random_cfg_t *out);
esp_err_t display_set_random(const display_random_cfg_t *cfg);
int32_t display_get_fade_ms(void);
esp_err_t display_set_fade_ms(int32_t fade_ms);
bool display_get_fade_enabled(void);
esp_err_t display_set_fade_enabled(bool enabled);

typedef enum {
    DISPLAY_AFTER_NONE = 0,
    DISPLAY_AFTER_SHOW,
    DISPLAY_AFTER_POISON,
    DISPLAY_AFTER_RANDOM,
    DISPLAY_AFTER_FADE_TEST,
} display_after_t;

void display_request_show(void);
void display_request_fade(void);
void display_request_fade_test(void);
void display_request_poison(const display_poison_cfg_t *override_or_null);
void display_request_random(void);
void display_request_save_result(bool ok, display_after_t after);
