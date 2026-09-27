#pragma once

#include <stdbool.h>

typedef void (*gpio_presence_cb_t)(bool present);

void gpio_driver_init(void);
void gpio_driver_presence_start(gpio_presence_cb_t callback);
int gpio_driver_presence_get(void);
