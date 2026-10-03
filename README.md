# Nixie clock

ESP-IDF firmware for a four-tube HH:MM nixie clock on ESP32-C6.

Tubes are driven through shift registers. Time comes from SNTP over Wi-Fi.
A presence input blanks the HV supply after idle time. A small HTTP page
reports status. Every few minutes the digits roll to reduce cathode poisoning.

## Build

ESP-IDF v6.x, target `esp32c6`.

```bash
idf.py set-target esp32c6
idf.py menuconfig   # User Configuration: board and SNTP servers
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

## Wi-Fi setup

Credentials are entered after boot and stored in NVS. They are not compile-time settings.

1. On first boot, or after a join failure, the clock opens a setup network named `NixieClock-XXXX`.
2. Join it and open `http://nixie.local/` (or `http://192.168.4.1/` if the name does not resolve).
3. Enter the home SSID and password, then press **Join**.
4. The page tells you to join that home network. Open `http://nixie.local/` there for the configuration page. The same name works on both networks.
5. Under **Network**, **Forget current network** clears the saved credentials and returns to the setup AP.

## Hardware

ESP32-C6 DevKitM-1 vs DevKitC-1 (module, flash, pinout): [docs/esp32-c6-devkits.md](docs/esp32-c6-devkits.md).

## Layout

| Path | Role |
|---|---|
| `main/` | IDF component: `app_main`, Kconfig |
| `src/clock/` | Setup, timers, occupancy policy |
| `src/display/` | Digit encoding, show time, roll, waiting animation |
| `src/drivers/` | PSU, PWM (OE), shift registers, presence GPIO, Wi-Fi |
| `src/web/` | HTTP setup and configuration pages |
