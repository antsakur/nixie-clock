#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "clock.h"
#include "display.h"
#include "presence_driver.h"
#include "psu_driver.h"

void app_main(void)
{
    display_init();
    psu_driver_init();
    presence_driver_init();

    display_clear();
    psu_driver_enable();
    clock_start();
}
