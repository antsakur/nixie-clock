#pragma once

#include <stdint.h>

void pwm_driver_init(int gpio_num);
void pwm_driver_set_duty(uint32_t duty_cycle_percent);
void pwm_driver_set_duty_counts(uint32_t duty);
