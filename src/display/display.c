#include <string.h>
#include <time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "nvs.h"

#include "defines.h"
#include "display.h"
#include "psu_driver.h"
#include "pwm_driver.h"
#include "shift_reg_driver.h"

static const char *TAG = "Display";

#define TUBE_COUNT 4
#define SYMBOL_COUNT 12
#define SYMBOL_LEFT_DOT 10
#define SYMBOL_RIGHT_DOT 11

typedef enum {
    DISPLAY_CMD_SHOW_TIME,
    DISPLAY_CMD_POISON,
    DISPLAY_CMD_RANDOM,
    DISPLAY_CMD_FADE,
    DISPLAY_CMD_FADE_TEST,
    DISPLAY_CMD_SAVE_RESULT,
} display_cmd_t;

typedef enum {
    MODE_SHOW,
    MODE_POISON,
    MODE_RANDOM,
    MODE_FADE,
} display_mode_t;

typedef struct {
    display_cmd_t cmd;
    bool poison_override;
    display_poison_cfg_t poison;
    bool save_ok;
    display_after_t after;
} display_msg_t;

static QueueHandle_t command_queue;
static SemaphoreHandle_t settings_lock;
static volatile display_mode_t shown_mode = MODE_SHOW;

static display_poison_cfg_t poison_cfg = {
    .digit_duration_ms = 50,
    .run_duration_ms = 1050,
    .offset = 3,
    .inverse_direction = false,
};
static display_random_cfg_t random_cfg = {
    .digit_duration_ms = 200,
    .run_duration_ms = -1,
};
static int32_t fade_ms = 500;
static bool fade_enabled = true;

static int symbols[TUBE_COUNT];
static bool command_pending;
static display_msg_t pending_command;

static int32_t clamp_digit_ms(int32_t value, int32_t fallback, int32_t max_ms)
{
    if (value < 1) {
        return fallback;
    }
    if (value > max_ms) {
        return max_ms;
    }
    return value;
}

static int32_t clamp_run_ms(int32_t value)
{
    if (value < -1) {
        return -1;
    }
    return value;
}

static void sanitize_poison(display_poison_cfg_t *cfg)
{
    cfg->digit_duration_ms = clamp_digit_ms(cfg->digit_duration_ms, 50, 1000);
    cfg->run_duration_ms = clamp_run_ms(cfg->run_duration_ms);
    if (cfg->offset > 12) {
        cfg->offset = 12;
    }
}

static void sanitize_random(display_random_cfg_t *cfg)
{
    cfg->digit_duration_ms = clamp_digit_ms(cfg->digit_duration_ms, 200, 5000);
    cfg->run_duration_ms = clamp_run_ms(cfg->run_duration_ms);
}

static int32_t sanitize_fade(int32_t value)
{
    if (value < 1) {
        return 1;
    }
    if (value > 5000) {
        return 5000;
    }
    return value;
}

static esp_err_t settings_save_unlocked(void)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open("display_cfg", NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Could not open display settings: %s", esp_err_to_name(err));
        return err;
    }

#define DISPLAY_NVS_SET(call) do { if (err == ESP_OK) err = (call); } while (0)
    DISPLAY_NVS_SET(nvs_set_i32(handle, "p_digit", poison_cfg.digit_duration_ms));
    DISPLAY_NVS_SET(nvs_set_u8(handle, "p_off", poison_cfg.offset));
    DISPLAY_NVS_SET(nvs_set_i32(handle, "p_run", poison_cfg.run_duration_ms));
    DISPLAY_NVS_SET(nvs_set_u8(handle, "p_inv", poison_cfg.inverse_direction ? 1 : 0));
    DISPLAY_NVS_SET(nvs_set_i32(handle, "r_digit", random_cfg.digit_duration_ms));
    DISPLAY_NVS_SET(nvs_set_i32(handle, "r_run", random_cfg.run_duration_ms));
    DISPLAY_NVS_SET(nvs_set_i32(handle, "fade", fade_ms));
    DISPLAY_NVS_SET(nvs_set_u8(handle, "fade_on", fade_enabled ? 1 : 0));
    DISPLAY_NVS_SET(nvs_commit(handle));
#undef DISPLAY_NVS_SET

    nvs_close(handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Could not save display settings: %s", esp_err_to_name(err));
    }
    return err;
}

void display_settings_load(void)
{
    bool dirty = false;
    nvs_handle_t handle;
    xSemaphoreTake(settings_lock, portMAX_DELAY);
    if (nvs_open("display_cfg", NVS_READONLY, &handle) != ESP_OK) {
        dirty = true;
    } else {
        int32_t value = 0;
        uint8_t small = 0;
        if (nvs_get_i32(handle, "p_digit", &value) == ESP_OK) {
            poison_cfg.digit_duration_ms = value;
        } else {
            dirty = true;
        }
        if (nvs_get_u8(handle, "p_off", &small) == ESP_OK) {
            poison_cfg.offset = small;
        } else {
            dirty = true;
        }
        if (nvs_get_i32(handle, "p_run", &value) == ESP_OK) {
            poison_cfg.run_duration_ms = value;
        } else {
            dirty = true;
        }
        if (nvs_get_u8(handle, "p_inv", &small) == ESP_OK) {
            poison_cfg.inverse_direction = small != 0;
        } else {
            dirty = true;
        }
        if (nvs_get_i32(handle, "r_digit", &value) == ESP_OK) {
            random_cfg.digit_duration_ms = value;
        } else {
            dirty = true;
        }
        if (nvs_get_i32(handle, "r_run", &value) == ESP_OK) {
            random_cfg.run_duration_ms = value;
        } else {
            dirty = true;
        }
        if (nvs_get_i32(handle, "fade", &value) == ESP_OK) {
            fade_ms = value;
        } else {
            dirty = true;
        }
        if (nvs_get_u8(handle, "fade_on", &small) == ESP_OK) {
            fade_enabled = small != 0;
        }
        nvs_close(handle);
    }
    sanitize_poison(&poison_cfg);
    sanitize_random(&random_cfg);
    fade_ms = sanitize_fade(fade_ms);
    if (dirty) {
        esp_err_t err = settings_save_unlocked();
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Could not persist sanitized display settings");
        }
    }
    xSemaphoreGive(settings_lock);
}

void display_get_poison(display_poison_cfg_t *out)
{
    xSemaphoreTake(settings_lock, portMAX_DELAY);
    *out = poison_cfg;
    xSemaphoreGive(settings_lock);
}

esp_err_t display_set_settings(const display_settings_t *settings)
{
    if (!settings) {
        return ESP_ERR_INVALID_ARG;
    }

    xSemaphoreTake(settings_lock, portMAX_DELAY);
    poison_cfg = settings->poison;
    random_cfg = settings->random;
    fade_enabled = settings->fade_enabled;
    if (fade_enabled) {
        fade_ms = settings->fade_ms;
    }
    sanitize_poison(&poison_cfg);
    sanitize_random(&random_cfg);
    fade_ms = sanitize_fade(fade_ms);
    esp_err_t err = settings_save_unlocked();
    xSemaphoreGive(settings_lock);
    return err;
}

esp_err_t display_set_poison(const display_poison_cfg_t *cfg)
{
    xSemaphoreTake(settings_lock, portMAX_DELAY);
    poison_cfg = *cfg;
    sanitize_poison(&poison_cfg);
    esp_err_t err = settings_save_unlocked();
    xSemaphoreGive(settings_lock);
    return err;
}

void display_get_random(display_random_cfg_t *out)
{
    xSemaphoreTake(settings_lock, portMAX_DELAY);
    *out = random_cfg;
    xSemaphoreGive(settings_lock);
}

esp_err_t display_set_random(const display_random_cfg_t *cfg)
{
    xSemaphoreTake(settings_lock, portMAX_DELAY);
    random_cfg = *cfg;
    sanitize_random(&random_cfg);
    esp_err_t err = settings_save_unlocked();
    xSemaphoreGive(settings_lock);
    return err;
}

int32_t display_get_fade_ms(void)
{
    int32_t value;
    xSemaphoreTake(settings_lock, portMAX_DELAY);
    value = fade_ms;
    xSemaphoreGive(settings_lock);
    return value;
}

esp_err_t display_set_fade_ms(int32_t value)
{
    xSemaphoreTake(settings_lock, portMAX_DELAY);
    fade_ms = sanitize_fade(value);
    esp_err_t err = settings_save_unlocked();
    xSemaphoreGive(settings_lock);
    return err;
}

bool display_get_fade_enabled(void)
{
    bool enabled;
    xSemaphoreTake(settings_lock, portMAX_DELAY);
    enabled = fade_enabled;
    xSemaphoreGive(settings_lock);
    return enabled;
}

esp_err_t display_set_fade_enabled(bool enabled)
{
    xSemaphoreTake(settings_lock, portMAX_DELAY);
    fade_enabled = enabled;
    esp_err_t err = settings_save_unlocked();
    xSemaphoreGive(settings_lock);
    return err;
}

static void copy_poison(display_poison_cfg_t *out)
{
    xSemaphoreTake(settings_lock, portMAX_DELAY);
    *out = poison_cfg;
    xSemaphoreGive(settings_lock);
}

static void copy_random(display_random_cfg_t *out)
{
    xSemaphoreTake(settings_lock, portMAX_DELAY);
    *out = random_cfg;
    xSemaphoreGive(settings_lock);
}

static int tube_symbol_bit(int tube, int symbol)
{
    int base = tube * 12;
    if (symbol <= 0) {
        return base + 1;
    }
    if (symbol < 10) {
        return base + (11 - symbol);
    }
    if (symbol == SYMBOL_LEFT_DOT) {
        return base + 11;
    }
    return base;
}

static uint64_t pack_symbols(const int *tube_symbols)
{
    uint64_t bits = 0;
    for (int tube = 0; tube < TUBE_COUNT; tube++) {
        bits |= 1ULL << tube_symbol_bit(tube, tube_symbols[tube]);
    }
    return bits;
}

static void render_symbols(const int *tube_symbols)
{
    shift_reg_driver_write(pack_symbols(tube_symbols));
}

static void step_symbol(int *symbol, bool inverse)
{
    int step = inverse ? -1 : 1;
    *symbol = (*symbol + step + SYMBOL_COUNT) % SYMBOL_COUNT;
}

static void current_digits(int out[TUBE_COUNT])
{
    time_t now;
    struct tm timeinfo;
    time(&now);
    localtime_r(&now, &timeinfo);
    out[0] = timeinfo.tm_min % 10;
    out[1] = timeinfo.tm_min / 10;
    out[2] = timeinfo.tm_hour % 10;
    out[3] = timeinfo.tm_hour / 10;
}

static bool wait_until_command(int64_t deadline_us, display_msg_t *msg)
{
    if (command_pending) {
        *msg = pending_command;
        command_pending = false;
        return true;
    }

    for (;;) {
        int64_t remaining_us = deadline_us - esp_timer_get_time();
        if (remaining_us <= 0) {
            return false;
        }

        uint64_t remaining_ms = ((uint64_t)remaining_us + 999ULL) / 1000ULL;
        TickType_t wait_ticks = pdMS_TO_TICKS(remaining_ms);
        if (wait_ticks < 1) {
            wait_ticks = 1;
        }
        if (xQueueReceive(command_queue, msg, wait_ticks) == pdTRUE) {
            return true;
        }
    }
}

static bool wait_for_command(int ms, display_msg_t *msg)
{
    if (ms < 1) {
        ms = 1;
    }
    return wait_until_command(esp_timer_get_time() + (int64_t)ms * 1000, msg);
}

static void hold_pending(const display_msg_t *msg)
{
    pending_command = *msg;
    command_pending = true;
}

static void show_digits(const int digits[TUBE_COUNT])
{
    memcpy(symbols, digits, sizeof(symbols));
    render_symbols(symbols);
    shown_mode = MODE_SHOW;
}

static bool crossfade(const int previous[TUBE_COUNT], const int next[TUBE_COUNT])
{
    int shown[TUBE_COUNT];
    int rendered[TUBE_COUNT];
    int32_t duration;
    xSemaphoreTake(settings_lock, portMAX_DELAY);
    duration = fade_ms;
    xSemaphoreGive(settings_lock);

    shown_mode = MODE_FADE;
    memcpy(rendered, previous, sizeof(rendered));

    int64_t start_us = esp_timer_get_time();
    int64_t duration_us = (int64_t)(duration > 0 ? duration : 1) * 1000;
    int64_t end_us = start_us + duration_us;
    int64_t next_slot_us = start_us;
    uint32_t mix_accumulator = 0;

    while (next_slot_us < end_us) {
        next_slot_us += 1000;
        if (next_slot_us > end_us) {
            next_slot_us = end_us;
        }

        display_msg_t msg;
        if (wait_until_command(next_slot_us, &msg)) {
            memcpy(symbols, rendered, sizeof(symbols));
            hold_pending(&msg);
            return false;
        }

        int64_t now_us = esp_timer_get_time();
        int64_t elapsed_us = now_us - start_us;
        if (elapsed_us >= duration_us) {
            break;
        }

        uint32_t next_weight = (uint32_t)(((uint64_t)elapsed_us << 16) / (uint64_t)duration_us);
        mix_accumulator += next_weight;
        bool slot_next = mix_accumulator >= (1u << 16);
        if (slot_next) {
            mix_accumulator -= (1u << 16);
        }

        for (int tube = 0; tube < TUBE_COUNT; tube++) {
            shown[tube] = (previous[tube] == next[tube] || slot_next) ? next[tube] : previous[tube];
        }
        if (memcmp(shown, rendered, sizeof(shown)) != 0) {
            render_symbols(shown);
            memcpy(rendered, shown, sizeof(rendered));
        }

        int64_t after_render_us = esp_timer_get_time();
        if (after_render_us - next_slot_us >= 1000) {
            next_slot_us = after_render_us;
        }
    }
    memcpy(symbols, next, sizeof(symbols));
    render_symbols(symbols);
    return true;
}

static void run_fade(void)
{
    int previous[TUBE_COUNT];
    int next[TUBE_COUNT];
    bool enabled;
    xSemaphoreTake(settings_lock, portMAX_DELAY);
    enabled = fade_enabled;
    xSemaphoreGive(settings_lock);
    memcpy(previous, symbols, sizeof(previous));
    current_digits(next);
    if (!enabled) {
        show_digits(next);
        return;
    }
    if (crossfade(previous, next)) {
        show_digits(next);
    }
}

static int alternate_digit(int symbol)
{
    if (symbol < 0 || symbol > 9) {
        return 0;
    }
    return (symbol + 1) % 10;
}

static void run_fade_test(void)
{
    int previous[TUBE_COUNT];
    int alternate[TUBE_COUNT];
    int live[TUBE_COUNT];
    bool match = true;
    memcpy(previous, symbols, sizeof(previous));
    for (int tube = 0; tube < TUBE_COUNT; tube++) {
        alternate[tube] = alternate_digit(previous[tube]);
    }
    if (!crossfade(previous, alternate)) {
        return;
    }
    display_msg_t msg;
    if (wait_for_command(1000, &msg)) {
        hold_pending(&msg);
        return;
    }
    current_digits(live);
    for (int tube = 0; tube < TUBE_COUNT; tube++) {
        if (symbols[tube] != live[tube]) {
            match = false;
            break;
        }
    }
    if (!match && !crossfade(symbols, live)) {
        return;
    }
    show_digits(live);
}

#define SAVE_FLASH_COUNT 3
#define SAVE_FLASH_TOTAL_MS 1000
#define SAVE_FLASH_HALF_MS (SAVE_FLASH_TOTAL_MS / (SAVE_FLASH_COUNT * 2))

static void play_save_flash(bool ok)
{
    bool hv_was_on = psu_driver_is_enabled();
    int time_digits[TUBE_COUNT];
    if (!hv_was_on) {
        psu_driver_enable();
    }
    current_digits(time_digits);
    for (int flash = 0; flash < SAVE_FLASH_COUNT; flash++) {
        if (ok) {
            uint64_t bits = pack_symbols(time_digits);
            bits |= 1ULL << tube_symbol_bit(0, SYMBOL_RIGHT_DOT);
            shift_reg_driver_write(bits);
        } else {
            int lit[TUBE_COUNT] = {8, 8, 8, 8};
            render_symbols(lit);
        }
        vTaskDelay(pdMS_TO_TICKS(SAVE_FLASH_HALF_MS));
        if (ok) {
            render_symbols(time_digits);
        } else {
            shift_reg_driver_write(0);
        }
        vTaskDelay(pdMS_TO_TICKS(SAVE_FLASH_HALF_MS));
    }
    render_symbols(symbols);
    if (!hv_was_on) {
        psu_driver_disable();
    }
}

static void poison_phases(int32_t next_change[TUBE_COUNT], const display_poison_cfg_t *cfg)
{
    int32_t digit_ms = cfg->digit_duration_ms > 0 ? cfg->digit_duration_ms : 1;
    uint32_t span = digit_ms > 1 ? (uint32_t)digit_ms : 1;
    for (int tube = 0; tube < TUBE_COUNT; tube++) {
        next_change[tube] = tube * (int)cfg->offset * digit_ms + (int32_t)(esp_random() % span);
    }
}

static void run_poison(display_poison_cfg_t cfg)
{
    bool returning = false;
    int32_t elapsed = 0;
    int32_t now = 0;
    int32_t next_change[TUBE_COUNT];
    int target[TUBE_COUNT];
    sanitize_poison(&cfg);
    poison_phases(next_change, &cfg);
    shown_mode = MODE_POISON;
    render_symbols(symbols);
    for (;;) {
        int32_t soonest = next_change[0];
        for (int tube = 1; tube < TUBE_COUNT; tube++) {
            if (next_change[tube] < soonest) {
                soonest = next_change[tube];
            }
        }
        int32_t wait = soonest - now;
        if (wait > 0) {
            display_msg_t msg;
            if (wait_for_command(wait, &msg)) {
                if (msg.cmd == DISPLAY_CMD_POISON) {
                    if (msg.poison_override) {
                        cfg = msg.poison;
                    } else {
                        copy_poison(&cfg);
                    }
                    sanitize_poison(&cfg);
                    poison_phases(next_change, &cfg);
                    returning = false;
                    elapsed = 0;
                    now = 0;
                    continue;
                }
                if (msg.cmd == DISPLAY_CMD_SAVE_RESULT) {
                    play_save_flash(msg.save_ok);
                    if (msg.after == DISPLAY_AFTER_POISON) {
                        copy_poison(&cfg);
                        sanitize_poison(&cfg);
                        poison_phases(next_change, &cfg);
                        returning = false;
                        elapsed = 0;
                        now = 0;
                    } else if (msg.after == DISPLAY_AFTER_RANDOM || msg.after == DISPLAY_AFTER_FADE_TEST) {
                        display_msg_t follow = {
                            .cmd = msg.after == DISPLAY_AFTER_RANDOM ? DISPLAY_CMD_RANDOM
                                                                    : DISPLAY_CMD_FADE_TEST,
                        };
                        hold_pending(&follow);
                        return;
                    } else if (msg.after == DISPLAY_AFTER_SHOW) {
                        returning = true;
                        current_digits(target);
                    }
                    continue;
                }
                if (msg.cmd == DISPLAY_CMD_RANDOM || msg.cmd == DISPLAY_CMD_FADE_TEST) {
                    hold_pending(&msg);
                    return;
                }
                returning = true;
                current_digits(target);
                continue;
            }
            now += wait;
            elapsed += wait;
        }
        if (!returning && cfg.run_duration_ms >= 0 && elapsed >= cfg.run_duration_ms) {
            returning = true;
            current_digits(target);
        }
        if (returning) {
            int latest[TUBE_COUNT];
            current_digits(latest);
            memcpy(target, latest, sizeof(target));
        }
        bool changed = false;
        bool all_match = returning;
        int32_t step = cfg.digit_duration_ms > 0 ? cfg.digit_duration_ms : 1;
        for (int tube = 0; tube < TUBE_COUNT; tube++) {
            if (now < next_change[tube]) {
                if (returning && symbols[tube] != target[tube]) {
                    all_match = false;
                }
                continue;
            }
            next_change[tube] += step;
            if (!returning) {
                step_symbol(&symbols[tube], cfg.inverse_direction);
                changed = true;
            } else if (symbols[tube] != target[tube]) {
                step_symbol(&symbols[tube], cfg.inverse_direction);
                all_match = false;
                changed = true;
            }
        }
        if (changed) {
            render_symbols(symbols);
        }
        if (all_match) {
            shown_mode = MODE_SHOW;
            return;
        }
    }
}

static void random_phases(int32_t next_change[TUBE_COUNT], int32_t digit_ms)
{
    uint32_t span = digit_ms > 1 ? (uint32_t)digit_ms : 1;
    for (int tube = 0; tube < TUBE_COUNT; tube++) {
        next_change[tube] = (int32_t)(esp_random() % span);
    }
}

static void run_random(void)
{
    display_random_cfg_t cfg;
    bool returning = false;
    int32_t elapsed = 0;
    int32_t now = 0;
    int32_t next_change[TUBE_COUNT];
    int target[TUBE_COUNT];
    copy_random(&cfg);
    random_phases(next_change, cfg.digit_duration_ms);
    shown_mode = MODE_RANDOM;
    render_symbols(symbols);
    for (;;) {
        int32_t soonest = next_change[0];
        for (int tube = 1; tube < TUBE_COUNT; tube++) {
            if (next_change[tube] < soonest) {
                soonest = next_change[tube];
            }
        }
        int32_t wait = soonest - now;
        if (wait > 0) {
            display_msg_t msg;
            if (wait_for_command(wait, &msg)) {
                if (msg.cmd == DISPLAY_CMD_RANDOM) {
                    copy_random(&cfg);
                    random_phases(next_change, cfg.digit_duration_ms);
                    returning = false;
                    elapsed = 0;
                    now = 0;
                    continue;
                }
                if (msg.cmd == DISPLAY_CMD_SAVE_RESULT) {
                    play_save_flash(msg.save_ok);
                    if (msg.after == DISPLAY_AFTER_RANDOM) {
                        copy_random(&cfg);
                        random_phases(next_change, cfg.digit_duration_ms);
                        returning = false;
                        elapsed = 0;
                        now = 0;
                    } else if (msg.after == DISPLAY_AFTER_POISON || msg.after == DISPLAY_AFTER_FADE_TEST) {
                        display_msg_t follow = {
                            .cmd = msg.after == DISPLAY_AFTER_POISON ? DISPLAY_CMD_POISON
                                                                    : DISPLAY_CMD_FADE_TEST,
                        };
                        hold_pending(&follow);
                        return;
                    } else if (msg.after == DISPLAY_AFTER_SHOW) {
                        returning = true;
                        current_digits(target);
                    }
                    continue;
                }
                if (msg.cmd == DISPLAY_CMD_POISON || msg.cmd == DISPLAY_CMD_FADE_TEST) {
                    hold_pending(&msg);
                    return;
                }
                returning = true;
                current_digits(target);
                continue;
            }
            now += wait;
            elapsed += wait;
        }
        if (!returning && cfg.run_duration_ms >= 0 && elapsed >= cfg.run_duration_ms) {
            returning = true;
            current_digits(target);
        }
        if (returning) {
            int latest[TUBE_COUNT];
            current_digits(latest);
            memcpy(target, latest, sizeof(target));
        }
        bool changed = false;
        bool all_match = returning;
        for (int tube = 0; tube < TUBE_COUNT; tube++) {
            if (now < next_change[tube]) {
                if (symbols[tube] >= 10 || (returning && symbols[tube] != target[tube])) {
                    all_match = false;
                }
                continue;
            }
            int32_t step = cfg.digit_duration_ms > 0 ? cfg.digit_duration_ms : 1;
            next_change[tube] += step;
            if (symbols[tube] >= 10) {
                step_symbol(&symbols[tube], false);
                all_match = false;
                changed = true;
            } else if (!returning) {
                symbols[tube] = (int)(esp_random() % 10);
                changed = true;
            } else if (symbols[tube] != target[tube]) {
                symbols[tube] = (int)(esp_random() % 10);
                all_match = false;
                changed = true;
            }
        }
        if (changed) {
            render_symbols(symbols);
        }
        if (all_match) {
            shown_mode = MODE_SHOW;
            return;
        }
    }
}

static void display_task(void *arg)
{
    (void)arg;
    for (;;) {
        display_msg_t msg;
        if (command_pending) {
            msg = pending_command;
            command_pending = false;
        } else if (xQueueReceive(command_queue, &msg, portMAX_DELAY) != pdTRUE) {
            continue;
        }
        switch (msg.cmd) {
        case DISPLAY_CMD_SHOW_TIME: {
            int digits[TUBE_COUNT];
            current_digits(digits);
            show_digits(digits);
            break;
        }
        case DISPLAY_CMD_FADE:
            if (shown_mode == MODE_SHOW) {
                run_fade();
            }
            break;
        case DISPLAY_CMD_FADE_TEST:
            run_fade_test();
            break;
        case DISPLAY_CMD_SAVE_RESULT:
            play_save_flash(msg.save_ok);
            if (msg.after == DISPLAY_AFTER_SHOW) {
                int digits[TUBE_COUNT];
                current_digits(digits);
                show_digits(digits);
            } else if (msg.after == DISPLAY_AFTER_POISON) {
                display_poison_cfg_t cfg;
                copy_poison(&cfg);
                run_poison(cfg);
            } else if (msg.after == DISPLAY_AFTER_RANDOM) {
                run_random();
            } else if (msg.after == DISPLAY_AFTER_FADE_TEST) {
                run_fade_test();
            }
            break;
        case DISPLAY_CMD_POISON: {
            display_poison_cfg_t cfg;
            if (msg.poison_override) {
                cfg = msg.poison;
            } else {
                copy_poison(&cfg);
            }
            run_poison(cfg);
            break;
        }
        case DISPLAY_CMD_RANDOM:
            run_random();
            break;
        }
    }
}

static void post_message(const display_msg_t *msg)
{
    if (command_queue) {
        xQueueOverwrite(command_queue, msg);
    }
}

void display_request_show(void)
{
    display_msg_t msg = {.cmd = DISPLAY_CMD_SHOW_TIME};
    post_message(&msg);
}

void display_request_fade(void)
{
    display_msg_t msg = {.cmd = DISPLAY_CMD_FADE};
    post_message(&msg);
}

void display_request_fade_test(void)
{
    display_msg_t msg = {.cmd = DISPLAY_CMD_FADE_TEST};
    post_message(&msg);
}

void display_request_poison(const display_poison_cfg_t *override_or_null)
{
    display_msg_t msg = {.cmd = DISPLAY_CMD_POISON};
    if (override_or_null) {
        msg.poison_override = true;
        msg.poison = *override_or_null;
        sanitize_poison(&msg.poison);
    }
    post_message(&msg);
}

void display_request_random(void)
{
    display_msg_t msg = {.cmd = DISPLAY_CMD_RANDOM};
    post_message(&msg);
}

void display_request_save_result(bool ok, display_after_t after)
{
    display_msg_t msg = {
        .cmd = DISPLAY_CMD_SAVE_RESULT,
        .save_ok = ok,
        .after = after,
    };
    post_message(&msg);
}

bool display_is_showing_time(void)
{
    return shown_mode == MODE_SHOW;
}

uint64_t display_format_time(struct tm timeinfo)
{
    int digits[TUBE_COUNT] = {
        timeinfo.tm_min % 10,
        timeinfo.tm_min / 10,
        timeinfo.tm_hour % 10,
        timeinfo.tm_hour / 10,
    };
    return pack_symbols(digits);
}

void display_init(void)
{
    settings_lock = xSemaphoreCreateMutex();
    command_queue = xQueueCreate(1, sizeof(display_msg_t));
    configASSERT(settings_lock);
    configASSERT(command_queue);
    shift_reg_driver_init();
    pwm_driver_init(GPIO_SHIFT_REG_OUTPUT_ENABLE);
    xTaskCreate(display_task, "display", 4096, NULL, 5, NULL);
    ESP_LOGI(TAG, "Display initialized");
}

void display_set_brightness(uint32_t percent)
{
    pwm_driver_set_duty(percent);
}

void display_set_brightness_counts(uint32_t duty)
{
    pwm_driver_set_duty_counts(duty);
}

void display_clear(void)
{
    memset(symbols, 0, sizeof(symbols));
    shift_reg_driver_write(0);
}
