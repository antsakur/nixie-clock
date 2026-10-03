# Nixie clock

ESP-IDF firmware for a four-tube HH:MM nixie clock on ESP32-C6.

Tubes are driven through shift registers. Time comes from SNTP over Wi-Fi.
A presence input blanks the HV supply after idle time. A small HTTP page
reports status. Every few minutes the digits roll to reduce cathode poisoning.

## Build

ESP-IDF v6.x, target `esp32c6`.

```bash
idf.py set-target esp32c6
idf.py menuconfig   # User Configuration: board, NTP, main vs test
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

## Wi-Fi setup

Credentials are entered after boot, not at compile time, and are stored in NVS.

1. On first boot (or if join fails), the clock opens a setup network named `NixieClock-XXXX`.
2. Join it from a phone and open `http://nixie.local/` (or `http://192.168.4.1/` if the name does not resolve).
3. Enter home SSID and password, then Save.
4. After the clock joins your network, open `http://nixie.local/` again. The same name works on both networks.
5. On the status page, **Forget Wi-Fi** clears flash and returns to the setup AP.

## Program selection

In menuconfig, **User Configuration → Program selection**:

- **Main program** — clock: Wi-Fi, NTP, occupancy, web status, anti-poisoning roll
- **Test program** — walks all digits in a loop (no Wi-Fi)

## Hardware

ESP32-C6 DevKitM-1 vs DevKitC-1 (module, flash, pinout): [docs/esp32-c6-devkits.md](docs/esp32-c6-devkits.md).

## Layout

| Path | Role |
|---|---|
| `main/` | IDF component: `app_main`, Kconfig |
| `src/clock/` | Setup, timers, occupancy policy |
| `src/display/` | Digit encoding, show time, roll, waiting animation |
| `src/drivers/` | PSU, PWM (OE), shift registers, presence GPIO, Wi-Fi |
| `src/web/` | HTTP status page |
