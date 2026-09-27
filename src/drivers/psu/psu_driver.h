#pragma once

#include <stdbool.h>

void psu_driver_init(void);
void psu_driver_enable(void);
void psu_driver_disable(void);
bool psu_driver_is_enabled(void);
