#pragma once

#include <stdbool.h>

typedef void (*presence_cb_t)(bool present);

void presence_driver_init(void);
void presence_driver_start(presence_cb_t callback);
int presence_driver_get(void);
