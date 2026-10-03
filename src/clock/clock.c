#include <stdint.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "freertos/timers.h"

#include "nvs_flash.h"
#include "nvs.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_netif_sntp.h"
#include "apps/esp_sntp.h"

#include "defines.h"
#include "clock.h"
#include "display.h"
#include "presence_driver.h"
#include "psu_driver.h"
#include "web_server.h"
#include "wifi_driver.h"
#include "wifi_store.h"

static const char *TAG = "Clock";
static const char *CLOCK_TZ_DEFAULT = "EET-2EEST,M3.5.0/3,M10.5.0/4";
#define CLOCK_BRIGHT_DEFAULT 80
#define CLOCK_NIGHT_DEFAULT 20

typedef struct {
    const char *id;
    const char *label;
    const char *std_tz;
    const char *dst_tz;
} clock_zone_t;

static const clock_zone_t clock_zones[] = {
    {"utc", "UTC (UTC+0)", "UTC0", NULL},
    {"uk", "United Kingdom (UTC+0 / UTC+1)", "GMT0", "GMT0BST,M3.5.0/1,M10.5.0/2"},
    {"cet", "Central Europe (UTC+1 / UTC+2)", "CET-1", "CET-1CEST,M3.5.0,M10.5.0/3"},
    {"fi", "Finland (UTC+2 / UTC+3)", "EET-2", "EET-2EEST,M3.5.0/3,M10.5.0/4"},
    {"eet", "Eastern Europe (UTC+2 / UTC+3)", "EET-2", "EET-2EEST,M3.5.0/3,M10.5.0/4"},
    {"msk", "Moscow (UTC+3)", "MSK-3", NULL},
    {"in", "India (UTC+5:30)", "IST-5:30", NULL},
    {"cn", "China (UTC+8)", "CST-8", NULL},
    {"jp", "Japan (UTC+9)", "JST-9", NULL},
    {"au", "Australia Eastern (UTC+10 / UTC+11)", "AEST-10", "AEST-10AEDT,M10.1.0,M4.1.0/3"},
    {"us_e", "US Eastern (UTC-5 / UTC-4)", "EST5", "EST5EDT,M3.2.0,M11.1.0"},
    {"us_c", "US Central (UTC-6 / UTC-5)", "CST6", "CST6CDT,M3.2.0,M11.1.0"},
    {"us_m", "US Mountain (UTC-7 / UTC-6)", "MST7", "MST7MDT,M3.2.0,M11.1.0"},
    {"us_p", "US Pacific (UTC-8 / UTC-7)", "PST8", "PST8PDT,M3.2.0,M11.1.0"},
};

static TimerHandle_t timers[3];
static SemaphoreHandle_t state_lock;
static uint8_t day_brightness = CLOCK_BRIGHT_DEFAULT;
static uint8_t night_brightness = CLOCK_NIGHT_DEFAULT;
static bool night_enabled = true;
static uint8_t night_start = 23;
static uint8_t night_end = 7;
static uint32_t applied_brightness = 0xFFFFFFFF;
static bool hv_force;
static bool presence_enabled = true;
static uint16_t idle_minutes = 10;
static uint16_t roll_minutes = 5;
static uint16_t transition_s = 10;
static bool brightness_zero;
static bool presence_idle_off;
static TimerHandle_t ramp_timer;
static bool ramp_active;
static bool ramp_test;
static int ramp_leg;
static bool ramp_hold;
static int ramp_from;
static int ramp_to;
static int ramp_origin;
static int64_t ramp_start_us;
static int64_t ramp_hold_start_us;
static esp_timer_handle_t duty_timer;
static int ramp_duty;
static int ramp_duty_goal;
static uint64_t ramp_step_us;
static int64_t ramp_next_step_us;
static char timezone[64] = "EET-2EEST,M3.5.0/3,M10.5.0/4";
static int zone_index = 3;
static bool dst_enabled = true;

static void state_lock_take(void)
{
    xSemaphoreTakeRecursive(state_lock, portMAX_DELAY);
}

static void state_lock_give(void)
{
    xSemaphoreGiveRecursive(state_lock);
}

static esp_err_t clock_settings_save(void)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open("clock_cfg", NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Could not open clock settings: %s", esp_err_to_name(err));
        return err;
    }

#define CLOCK_NVS_SET(call) do { if (err == ESP_OK) err = (call); } while (0)
    CLOCK_NVS_SET(nvs_set_u8(handle, "bright", day_brightness));
    CLOCK_NVS_SET(nvs_set_u8(handle, "nbright", night_brightness));
    CLOCK_NVS_SET(nvs_set_u8(handle, "night", night_enabled ? 1 : 0));
    CLOCK_NVS_SET(nvs_set_u8(handle, "nstart", night_start));
    CLOCK_NVS_SET(nvs_set_u8(handle, "nend", night_end));
    CLOCK_NVS_SET(nvs_set_u8(handle, "force", hv_force ? 1 : 0));
    CLOCK_NVS_SET(nvs_set_u8(handle, "pres", presence_enabled ? 1 : 0));
    CLOCK_NVS_SET(nvs_set_u16(handle, "idle", idle_minutes));
    CLOCK_NVS_SET(nvs_set_u16(handle, "rollm", roll_minutes));
    CLOCK_NVS_SET(nvs_set_u16(handle, "trans", transition_s));
    CLOCK_NVS_SET(nvs_set_u8(handle, "zone", zone_index < 0 ? 255 : (uint8_t)zone_index));
    CLOCK_NVS_SET(nvs_set_u8(handle, "dst", dst_enabled ? 1 : 0));
    CLOCK_NVS_SET(nvs_set_str(handle, "tz", timezone));
    CLOCK_NVS_SET(nvs_commit(handle));
#undef CLOCK_NVS_SET

    nvs_close(handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Could not save clock settings: %s", esp_err_to_name(err));
    }
    return err;
}

static int clock_match_zone(const char *tz, bool *dst)
{
    for (int i = 0; i < (int)(sizeof(clock_zones) / sizeof(clock_zones[0])); i++) {
        if (clock_zones[i].dst_tz && strcmp(tz, clock_zones[i].dst_tz) == 0) {
            *dst = true;
            return i;
        }
        if (strcmp(tz, clock_zones[i].std_tz) == 0) {
            *dst = false;
            return i;
        }
    }
    return -1;
}

static void clock_settings_load(void)
{
    nvs_handle_t handle;
    if (nvs_open("clock_cfg", NVS_READONLY, &handle) != ESP_OK) {
        return;
    }
    uint8_t value = 0;
    if (nvs_get_u8(handle, "bright", &value) == ESP_OK && value <= 100) {
        day_brightness = value;
    }
    if (nvs_get_u8(handle, "nbright", &value) == ESP_OK && value <= 100) {
        night_brightness = value;
    }
    if (nvs_get_u8(handle, "night", &value) == ESP_OK) {
        night_enabled = value != 0;
    }
    if (nvs_get_u8(handle, "nstart", &value) == ESP_OK && value < 24) {
        night_start = value;
    }
    if (nvs_get_u8(handle, "nend", &value) == ESP_OK && value < 24) {
        night_end = value;
    }
    if (nvs_get_u8(handle, "force", &value) == ESP_OK) {
        hv_force = value != 0;
    }
    if (nvs_get_u8(handle, "pres", &value) == ESP_OK) {
        presence_enabled = value != 0;
    } else if (hv_force) {
        presence_enabled = false;
    }
    uint16_t idle = 0;
    if (nvs_get_u16(handle, "idle", &idle) == ESP_OK && idle >= 1 && idle <= 240) {
        idle_minutes = idle;
    }
    uint16_t roll = 0;
    if (nvs_get_u16(handle, "rollm", &roll) == ESP_OK && roll >= 1 && roll <= 1440) {
        roll_minutes = roll;
    }
    uint16_t transition = 0;
    if (nvs_get_u16(handle, "trans", &transition) == ESP_OK && transition >= 1 && transition <= 60) {
        transition_s = transition;
    }
    size_t tz_len = sizeof(timezone);
    if (nvs_get_str(handle, "tz", timezone, &tz_len) != ESP_OK || timezone[0] == '\0') {
        strncpy(timezone, CLOCK_TZ_DEFAULT, sizeof(timezone) - 1);
        timezone[sizeof(timezone) - 1] = '\0';
    }
    {
        bool dst_from_tz = false;
        int matched = clock_match_zone(timezone, &dst_from_tz);
        if (matched >= 0) {
            zone_index = matched;
        } else {
            zone_index = 3;
        }
    }
    if (nvs_get_u8(handle, "dst", &value) == ESP_OK) {
        dst_enabled = value != 0;
    }
    nvs_close(handle);
}

static void clock_apply_timezone(void)
{
    setenv("TZ", timezone, 1);
    tzset();
}

static void clock_apply_timezone(void);
static void local_time(struct tm *out);

static bool clock_is_night(const struct tm *timeinfo)
{
    if (!night_enabled || timeinfo->tm_year < (2020 - 1900) || night_start == night_end) {
        return false;
    }
    int hour = timeinfo->tm_hour;
    if (night_start < night_end) {
        return hour >= night_start && hour < night_end;
    }
    return hour >= night_start || hour < night_end;
}

static uint8_t clock_target_brightness(const struct tm *timeinfo)
{
    return clock_is_night(timeinfo) ? night_brightness : day_brightness;
}

static void clock_output_level(uint32_t level)
{
    if (level > 100) {
        level = 100;
    }
    applied_brightness = level;
    display_set_brightness(level);
    if (level == 0) {
        brightness_zero = true;
        psu_driver_disable();
    } else if (brightness_zero) {
        brightness_zero = false;
        if (!presence_idle_off) {
            psu_driver_enable();
        }
    }
}

static uint32_t percent_to_duty(int percent)
{
    if (percent < 0) {
        percent = 0;
    } else if (percent > 100) {
        percent = 100;
    }
    return (((1u << PWM_RESOLUTION_BITS) - 1u) * (uint32_t)percent) / 100u;
}

static void apply_duty_count(uint32_t duty)
{
    uint32_t max_duty = (1u << PWM_RESOLUTION_BITS) - 1u;
    if (duty > max_duty) {
        duty = max_duty;
    }
    display_set_brightness_counts(duty);
    applied_brightness = max_duty ? (duty * 100u + max_duty / 2) / max_duty : 0;
    if (duty == 0) {
        brightness_zero = true;
        psu_driver_disable();
    } else if (brightness_zero) {
        brightness_zero = false;
        if (!presence_idle_off) {
            psu_driver_enable();
        }
    }
}

static void finish_duty_ramp(void)
{
    int origin = ramp_origin;
    ramp_active = false;
    ramp_test = false;
    ramp_hold = false;
    if (duty_timer) {
        esp_timer_stop(duty_timer);
    }
    clock_output_level((uint32_t)origin);
}

static void arm_duty_steps(int goal_duty)
{
    ramp_duty_goal = goal_duty;
    int steps = ramp_duty_goal - ramp_duty;
    if (steps < 0) {
        steps = -steps;
    }
    if (steps < 1) {
        steps = 1;
    }
    ramp_step_us = (uint64_t)transition_s * 1000000ULL / (uint64_t)steps;
    if (ramp_step_us < 1000) {
        ramp_step_us = 1000;
    }
    ramp_next_step_us = esp_timer_get_time() + (int64_t)ramp_step_us;
}

static void duty_timer_cb(void *arg)
{
    (void)arg;
    state_lock_take();
    int64_t now = esp_timer_get_time();
    if (ramp_hold) {
        if (now - ramp_hold_start_us < 1000000) {
            goto done;
        }
        ramp_hold = false;
        ramp_leg = 1;
        arm_duty_steps((int)percent_to_duty(ramp_origin));
        goto done;
    }
    if (now < ramp_next_step_us) {
        goto done;
    }
    if (ramp_duty == ramp_duty_goal) {
        if (ramp_leg == 0) {
            ramp_hold = true;
            ramp_hold_start_us = now;
            goto done;
        }
        finish_duty_ramp();
        goto done;
    }
    ramp_duty += (ramp_duty < ramp_duty_goal) ? 1 : -1;
    apply_duty_count((uint32_t)ramp_duty);
    ramp_next_step_us += (int64_t)ramp_step_us;
    if (ramp_next_step_us < now) {
        ramp_next_step_us = now + (int64_t)ramp_step_us;
    }
done:
    state_lock_give();
}

static void ramp_timer_cb(TimerHandle_t timer)
{
    (void)timer;
    state_lock_take();
    if (ramp_hold) {
        if ((esp_timer_get_time() - ramp_hold_start_us) < 1000000) {
            goto done;
        }
        ramp_hold = false;
        ramp_from = (int)applied_brightness;
        ramp_to = ramp_origin;
        ramp_start_us = esp_timer_get_time();
        if (ramp_from == ramp_to) {
            ramp_active = false;
            ramp_test = false;
            xTimerStop(ramp_timer, 0);
        }
        goto done;
    }
    int64_t elapsed_ms = (esp_timer_get_time() - ramp_start_us) / 1000;
    int32_t total_ms = (int32_t)transition_s * 1000;
    if (elapsed_ms >= total_ms) {
        clock_output_level((uint32_t)ramp_to);
        if (ramp_test && ramp_leg == 0) {
            ramp_leg = 1;
            ramp_hold = true;
            ramp_hold_start_us = esp_timer_get_time();
            goto done;
        }
        ramp_active = false;
        ramp_test = false;
        xTimerStop(ramp_timer, 0);
        goto done;
    }
    int level = ramp_from + (int)(((int64_t)(ramp_to - ramp_from) * elapsed_ms) / total_ms);
    if (level < 0) {
        level = 0;
    } else if (level > 100) {
        level = 100;
    }
    clock_output_level((uint32_t)level);
done:
    state_lock_give();
}

static void start_ramp(int from, int to)
{
    if (from < 0) {
        from = 0;
    } else if (from > 100) {
        from = 100;
    }
    if (to < 0) {
        to = 0;
    } else     if (to > 100) {
        to = 100;
    }
    ramp_hold = false;
    if (ramp_test && duty_timer) {
        if (ramp_timer) {
            xTimerStop(ramp_timer, 0);
        }
        ramp_duty = (int)percent_to_duty(from);
        apply_duty_count((uint32_t)ramp_duty);
        arm_duty_steps((int)percent_to_duty(to));
        ramp_from = from;
        ramp_to = to;
        ramp_active = true;
        esp_timer_stop(duty_timer);
        esp_timer_start_periodic(duty_timer, 1000);
        return;
    }
    if (duty_timer) {
        esp_timer_stop(duty_timer);
    }
    if (!ramp_timer || from == to) {
        clock_output_level((uint32_t)to);
        ramp_active = false;
        if (ramp_leg != 0 || from == to) {
            ramp_test = false;
        }
        if (ramp_timer) {
            xTimerStop(ramp_timer, 0);
        }
        return;
    }
    ramp_from = from;
    ramp_to = to;
    ramp_start_us = esp_timer_get_time();
    ramp_active = true;
    xTimerStart(ramp_timer, 0);
}

static void clock_follow_brightness(const struct tm *timeinfo)
{
    uint8_t target = clock_target_brightness(timeinfo);
    if (ramp_test) {
        return;
    }
    if (applied_brightness == 0xFFFFFFFF) {
        clock_output_level(target);
        return;
    }
    if (ramp_active && ramp_to == (int)target) {
        return;
    }
    if ((uint32_t)target != applied_brightness) {
        start_ramp((int)applied_brightness, (int)target);
    }
}

static void clock_apply_brightness(const struct tm *timeinfo)
{
    state_lock_take();
    clock_follow_brightness(timeinfo);
    state_lock_give();
}

static void clock_brightness_changed_locked(void)
{
    struct tm timeinfo;
    local_time(&timeinfo);
    ramp_test = false;
    clock_follow_brightness(&timeinfo);
}

static esp_err_t clock_brightness_changed(void)
{
    state_lock_take();
    clock_brightness_changed_locked();
    state_lock_give();
    return clock_settings_save();
}

esp_err_t clock_set_brightness_settings(const clock_brightness_settings_t *settings)
{
    if (!settings) {
        return ESP_ERR_INVALID_ARG;
    }

    state_lock_take();
    day_brightness = settings->day_brightness > 100 ? 100 : settings->day_brightness;
    night_brightness = settings->night_brightness > 100 ? 100 : settings->night_brightness;
    night_enabled = settings->night_enabled;
    night_start = settings->night_start > 23 ? 23 : settings->night_start;
    night_end = settings->night_end > 23 ? 23 : settings->night_end;
    transition_s = settings->transition_s;
    if (transition_s < 1) {
        transition_s = 1;
    } else if (transition_s > 60) {
        transition_s = 60;
    }
    clock_brightness_changed_locked();
    state_lock_give();
    return clock_settings_save();
}

uint8_t clock_get_brightness(void)
{
    state_lock_take();
    uint8_t value = day_brightness;
    state_lock_give();
    return value;
}

esp_err_t clock_set_brightness(uint8_t percent)
{
    if (percent > 100) {
        percent = 100;
    }
    state_lock_take();
    day_brightness = percent;
    state_lock_give();
    return clock_brightness_changed();
}

uint8_t clock_get_night_brightness(void)
{
    state_lock_take();
    uint8_t value = night_brightness;
    state_lock_give();
    return value;
}

esp_err_t clock_set_night_brightness(uint8_t percent)
{
    if (percent > 100) {
        percent = 100;
    }
    state_lock_take();
    night_brightness = percent;
    state_lock_give();
    return clock_brightness_changed();
}

bool clock_get_night_enabled(void)
{
    state_lock_take();
    bool enabled = night_enabled;
    state_lock_give();
    return enabled;
}

esp_err_t clock_set_night_enabled(bool enabled)
{
    state_lock_take();
    night_enabled = enabled;
    state_lock_give();
    return clock_brightness_changed();
}

uint8_t clock_get_night_start(void)
{
    state_lock_take();
    uint8_t value = night_start;
    state_lock_give();
    return value;
}

uint8_t clock_get_night_end(void)
{
    state_lock_take();
    uint8_t value = night_end;
    state_lock_give();
    return value;
}

esp_err_t clock_set_night_hours(uint8_t start_hour, uint8_t end_hour)
{
    if (start_hour > 23) {
        start_hour = 23;
    }
    if (end_hour > 23) {
        end_hour = 23;
    }
    state_lock_take();
    night_start = start_hour;
    night_end = end_hour;
    state_lock_give();
    return clock_brightness_changed();
}

int clock_zone_count(void)
{
    return (int)(sizeof(clock_zones) / sizeof(clock_zones[0]));
}

const char *clock_zone_id(int index)
{
    if (index < 0 || index >= clock_zone_count()) {
        return "custom";
    }
    return clock_zones[index].id;
}

const char *clock_zone_label(int index)
{
    if (index < 0 || index >= clock_zone_count()) {
        return "Custom";
    }
    return clock_zones[index].label;
}

bool clock_zone_has_dst(int index)
{
    if (index < 0 || index >= clock_zone_count()) {
        return true;
    }
    return clock_zones[index].dst_tz != NULL;
}

int clock_get_zone_index(void)
{
    return zone_index;
}

bool clock_get_dst(void)
{
    return dst_enabled;
}

static esp_err_t clock_store_tz(const char *tz)
{
    strncpy(timezone, tz, sizeof(timezone) - 1);
    timezone[sizeof(timezone) - 1] = '\0';
    clock_apply_timezone();
    return clock_settings_save();
}

esp_err_t clock_set_zone(int index, bool dst)
{
    if (index < 0 || index >= clock_zone_count()) {
        return ESP_ERR_INVALID_ARG;
    }
    zone_index = index;
    dst_enabled = dst && clock_zones[index].dst_tz != NULL;
    const char *tz = dst_enabled ? clock_zones[index].dst_tz : clock_zones[index].std_tz;
    return clock_store_tz(tz);
}

esp_err_t clock_set_custom_timezone(const char *tz)
{
    if (!tz || tz[0] == '\0') {
        return ESP_ERR_INVALID_ARG;
    }
    zone_index = -1;
    dst_enabled = false;
    return clock_store_tz(tz);
}

void clock_format_now(char *buf, size_t len)
{
    if (!buf || len == 0) {
        return;
    }
    struct tm timeinfo;
    local_time(&timeinfo);
    if (strftime(buf, len, "%Y-%m-%d %H:%M:%S %Z", &timeinfo) == 0) {
        buf[0] = '\0';
    }
}

const char *clock_get_timezone(void)
{
    return timezone;
}

esp_err_t clock_set_timezone(const char *tz)
{
    return clock_set_custom_timezone(tz);
}

bool clock_get_force_on(void)
{
    state_lock_take();
    bool force = hv_force;
    state_lock_give();
    return force;
}

esp_err_t clock_set_force_on(bool force)
{
    state_lock_take();
    hv_force = force;
    if (hv_force) {
        psu_driver_enable();
        if (timers[2]) {
            xTimerStop(timers[2], 0);
        }
    }
    state_lock_give();
    return clock_settings_save();
}

bool clock_get_presence_enabled(void)
{
    state_lock_take();
    bool enabled = presence_enabled;
    state_lock_give();
    return enabled;
}

esp_err_t clock_set_presence_settings(bool enabled, uint16_t minutes)
{
    if (minutes < 1) {
        minutes = 1;
    } else if (minutes > 240) {
        minutes = 240;
    }

    state_lock_take();
    presence_enabled = enabled;
    if (enabled) {
        idle_minutes = minutes;
        if (timers[2]) {
            bool running = xTimerIsTimerActive(timers[2]) != pdFALSE;
            xTimerChangePeriod(timers[2],
                               pdMS_TO_TICKS((uint32_t)idle_minutes * 60000UL), 0);
            if (!running) {
                xTimerStop(timers[2], 0);
            }
        }
    }

    if (!presence_enabled || hv_force) {
        presence_idle_off = false;
        if (!brightness_zero) {
            psu_driver_enable();
        }
        if (timers[2]) {
            xTimerStop(timers[2], 0);
        }
    } else if (presence_driver_get() == 0 && timers[2]) {
        xTimerReset(timers[2], 0);
    }
    state_lock_give();
    return clock_settings_save();
}

esp_err_t clock_set_presence_enabled(bool enabled)
{
    state_lock_take();
    presence_enabled = enabled;
    if (!presence_enabled || hv_force) {
        presence_idle_off = false;
        if (!brightness_zero) {
            psu_driver_enable();
        }
        if (timers[2]) {
            xTimerStop(timers[2], 0);
        }
    } else if (presence_driver_get() == 0 && timers[2]) {
        xTimerReset(timers[2], 0);
    }
    state_lock_give();
    return clock_settings_save();
}

uint16_t clock_get_idle_minutes(void)
{
    state_lock_take();
    uint16_t value = idle_minutes;
    state_lock_give();
    return value;
}

esp_err_t clock_set_idle_minutes(uint16_t minutes)
{
    if (minutes < 1) {
        minutes = 1;
    } else if (minutes > 240) {
        minutes = 240;
    }
    state_lock_take();
    idle_minutes = minutes;
    if (timers[2]) {
        bool running = xTimerIsTimerActive(timers[2]) != pdFALSE;
        xTimerChangePeriod(timers[2], pdMS_TO_TICKS((uint32_t)idle_minutes * 60000UL), 0);
        if (!running) {
            xTimerStop(timers[2], 0);
        }
    }
    state_lock_give();
    return clock_settings_save();
}

uint16_t clock_get_roll_minutes(void)
{
    return roll_minutes;
}

esp_err_t clock_set_roll_minutes(uint16_t minutes)
{
    if (minutes < 1) {
        minutes = 1;
    } else if (minutes > 1440) {
        minutes = 1440;
    }
    roll_minutes = minutes;
    if (timers[1]) {
        xTimerChangePeriod(timers[1], pdMS_TO_TICKS((uint32_t)roll_minutes * 60000UL), 0);
    }
    return clock_settings_save();
}

uint16_t clock_get_transition_s(void)
{
    state_lock_take();
    uint16_t value = transition_s;
    state_lock_give();
    return value;
}

esp_err_t clock_set_transition_s(uint16_t seconds)
{
    if (seconds < 1) {
        seconds = 1;
    } else if (seconds > 60) {
        seconds = 60;
    }
    state_lock_take();
    transition_s = seconds;
    state_lock_give();
    return clock_settings_save();
}

void clock_test_brightness_transition(void)
{
    state_lock_take();
    struct tm timeinfo;
    local_time(&timeinfo);
    bool night = clock_is_night(&timeinfo);
    int origin = night ? night_brightness : day_brightness;
    int other = night ? day_brightness : night_brightness;
    ramp_test = true;
    ramp_leg = 0;
    ramp_origin = origin;
    if (applied_brightness == 0xFFFFFFFF) {
        clock_output_level((uint32_t)origin);
    }
    start_ramp((int)applied_brightness, other);
    state_lock_give();
}

const char *clock_get_ntp_primary(void)
{
    return SNTP_TIME_SERVER;
}

const char *clock_get_ntp_backup(void)
{
    return SNTP_TIME_SERVER_BACKUP;
}

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

static volatile bool time_synced;

static void time_sync_notification_cb(struct timeval *tv)
{
    (void)tv;
    ESP_LOGI(TAG, "SNTP synchronized (reachability primary=%u backup=%u)",
             esp_sntp_getreachability(0), esp_sntp_getreachability(1));
    if (!time_synced) {
        time_synced = true;
        display_request_show();
    }
}

static void on_sta_got_ip(void)
{
    static bool sntp_started;
    if (!sntp_started) {
        esp_netif_sntp_start();
        sntp_started = true;
        ESP_LOGI(TAG, "SNTP started");
    }
}

static void presence_changed(bool present)
{
    state_lock_take();
    if (!presence_enabled || hv_force) {
        presence_idle_off = false;
        if (!brightness_zero) {
            psu_driver_enable();
        }
        xTimerStop(timers[2], 0);
        state_lock_give();
        return;
    }
    if (present) {
        presence_idle_off = false;
        if (!brightness_zero) {
            psu_driver_enable();
        }
        xTimerStop(timers[2], 0);
    } else if (xTimerReset(timers[2], 0) != pdPASS) {
        ESP_LOGE(TAG, "Failed to reset idle timer");
    }
    state_lock_give();
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
        if (timeinfo.tm_hour != last_hour || timeinfo.tm_min != last_min) {
            last_hour = timeinfo.tm_hour;
            last_min = timeinfo.tm_min;
            if (display_is_showing_time()) {
                display_request_fade();
            }
        }
        clock_apply_brightness(&timeinfo);
        break;
    }
    case TIMER_ROLL_DISP:
        if (time_synced) {
            display_request_poison(NULL);
        }
        break;
    case TIMER_NO_MOVEMENT:
        state_lock_take();
        if (presence_enabled && !hv_force) {
            presence_idle_off = true;
            psu_driver_disable();
        }
        state_lock_give();
        break;
    default:
        break;
    }
}

static void setup_task(void *pvParameters)
{
    (void)pvParameters;

    nvs_init();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    clock_settings_load();
    display_settings_load();
    clock_apply_timezone();
    struct tm boot_time;
    local_time(&boot_time);
    clock_apply_brightness(&boot_time);
    if ((hv_force || !presence_enabled) && !brightness_zero) {
        psu_driver_enable();
    }

    display_poison_cfg_t boot_poison = {
        .digit_duration_ms = 100,
        .run_duration_ms = -1,
        .offset = 1,
        .inverse_direction = false,
    };
    display_request_poison(&boot_poison);

    esp_sntp_config_t config = ESP_NETIF_SNTP_DEFAULT_CONFIG_MULTIPLE(2,
        ESP_SNTP_SERVER_LIST(SNTP_TIME_SERVER, SNTP_TIME_SERVER_BACKUP));
    config.start = false;
    config.sync_cb = time_sync_notification_cb;
    esp_netif_sntp_init(&config);
    ESP_LOGI(TAG, "SNTP servers: %s, %s", SNTP_TIME_SERVER, SNTP_TIME_SERVER_BACKUP);

    wifi_driver_init();
    wifi_driver_set_got_ip_cb(on_sta_got_ip);
    wifi_driver_start_ap();

    char ssid[WIFI_STORE_SSID_MAX + 1];
    char pass[WIFI_STORE_PASS_MAX + 1];
    if (wifi_store_load(ssid, sizeof(ssid), pass, sizeof(pass))) {
        wifi_driver_start_sta(ssid, pass);
    }
    web_server_start();

    struct tm timeinfo;
    local_time(&timeinfo);
    char strftime_buf[64];
    strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
    ESP_LOGI(TAG, "Current date/time: %s", strftime_buf);

    timers[0] = xTimerCreate("tick", pdMS_TO_TICKS(1000), pdTRUE,
                             (void *)(uintptr_t)TIMER_TICK, timer_callback);
    timers[1] = xTimerCreate("roll", pdMS_TO_TICKS((uint32_t)roll_minutes * 60000UL), pdTRUE,
                             (void *)(uintptr_t)TIMER_ROLL_DISP, timer_callback);
    timers[2] = xTimerCreate("idle", pdMS_TO_TICKS((uint32_t)idle_minutes * 60000UL), pdFALSE,
                             (void *)(uintptr_t)TIMER_NO_MOVEMENT, timer_callback);
    ramp_timer = xTimerCreate("ramp", pdMS_TO_TICKS(100), pdTRUE, NULL, ramp_timer_cb);
    const esp_timer_create_args_t duty_timer_args = {
        .callback = duty_timer_cb,
        .name = "duty_ramp",
    };
    ESP_ERROR_CHECK(esp_timer_create(&duty_timer_args, &duty_timer));

    presence_driver_start(presence_changed);

    xTimerStart(timers[0], 0);
    xTimerStart(timers[1], 0);
    if (presence_enabled && !hv_force && presence_driver_get() == 0) {
        ESP_LOGI(TAG, "No presence at init; starting idle timer");
        xTimerStart(timers[2], 0);
    }

    vTaskDelete(NULL);
}

void clock_start(void)
{
    state_lock = xSemaphoreCreateRecursiveMutex();
    configASSERT(state_lock);
    xTaskCreate(setup_task, "setup", 4096, NULL, 5, NULL);
}
