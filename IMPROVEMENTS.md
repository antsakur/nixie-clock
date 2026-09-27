# Improvement backlog

Saved so we can circle back. Items not marked deferred are implemented in the current tree.

## P0 — Build / project layout

- [x] Restore ESP-IDF `main` component (`main/CMakeLists.txt`)
- [x] Move `Kconfig.projbuild` into `main/`
- [x] Register sources under `src/`
- [x] `.gitignore` (`build/`, `sdkconfig`, `sdkconfig.old`)
- [x] `sdkconfig.defaults` without real Wi-Fi secrets
- [x] README (target, flash, main vs test)

## P1 — Driver split

- [x] PSU, shift registers, PWM, GPIO/presence, Wi-Fi, web server, display, clock
- [x] Implementations in `.c` files, headers with declarations only
- [x] Include guards on all headers

## P2 — Correctness / RTOS

- [x] Timer stop/reset from the GPIO task (not `*FromISR`)
- [x] ISR: `xQueueSendFromISR` + `pxHigherPriorityTaskWoken`
- [x] No long `esp_rom_delay_us` in timer callbacks
- [x] Roll / waiting / digit-test use `vTaskDelay`
- [x] Display updates when hour/minute change (1 s tick)
- [x] No shared unlocked `now` / `timeinfo` for rendering
- [x] Shift-register write waits on both RMT channels; dummy write at init
- [x] `nvs_flash_init` erase + retry
- [x] Web server starts only after Wi-Fi connect
- [x] Wi-Fi reconnects after max retries; password not logged

## P3 — Hardware / product

- [x] PSU GPIO: known off level, no spurious pulls
- [x] Presence debounce
- [x] Waiting animation uses `&&`

## Deferred (optional later)

- [ ] PWM night dimming
- [ ] Web UI controls (brightness, timezone, force-on)
- [ ] JSON `/api/status`
- [ ] Host unit tests for `display_format_time()`
