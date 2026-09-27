# Nixie clock

ESP-IDF firmware for a four-tube HH:MM nixie clock on ESP32-C6.

Tubes are driven through shift registers. Time comes from SNTP over Wi-Fi.
A presence input blanks the HV supply after idle time. A small HTTP page
reports status. Every few minutes the digits roll to reduce cathode poisoning.

## Build

ESP-IDF v6.x, target `esp32c6`.

```bash
idf.py set-target esp32c6
idf.py menuconfig   # User Configuration: board, Wi-Fi, NTP, main vs test
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

Wi-Fi credentials belong in local `sdkconfig` (gitignored) or menuconfig, not in git.
`sdkconfig.defaults` only has placeholders.

## Program selection

In menuconfig, **User Configuration → Program selection**:

- **Main program** — clock: Wi-Fi, NTP, occupancy, web status, anti-poisoning roll
- **Test program** — walks all digits in a loop (no Wi-Fi)

## Layout

| Path | Role |
|---|---|
| `main/` | IDF component: `app_main`, Kconfig |
| `src/clock/` | Setup, timers, occupancy policy |
| `src/display/` | Digit encoding, show time, roll, waiting animation |
| `src/drivers/` | PSU, PWM (OE), shift registers, presence GPIO, Wi-Fi |
| `src/web/` | HTTP status page |
