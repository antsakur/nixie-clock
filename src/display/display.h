#pragma once

#include <stdint.h>
#include <time.h>

void display_init(void);
void display_set_brightness(uint32_t percent);
void display_clear(void);
void display_show_bitmap(uint64_t data);
uint64_t display_format_time(struct tm timeinfo);
void display_show_time(struct tm timeinfo);
void display_roll(struct tm timeinfo);
void display_waiting_frame(uint32_t index);
void display_test_loop(void);
void display_test_task(void *pvParameters);
