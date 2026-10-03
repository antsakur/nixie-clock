# ESP32-C6 DevKitM-1 vs DevKitC-1

Espressif entry-level boards for the same SoC family. They differ by **module**, **flash**, **footprint**, and **header pinout**, not by radio or CPU architecture.

Official user guides:

- [ESP32-C6-DevKitM-1](https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32c6/esp32-c6-devkitm-1/user_guide.html)
- [ESP32-C6-DevKitC-1](https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32c6/esp32-c6-devkitc-1/user_guide.html)

In this firmware, map kits in menuconfig **User Configuration → Board**:

| DevKit | Kconfig |
|---|---|
| ESP32-C6-DevKitC-1 | `ESP32_C6_WROOM_1` |
| ESP32-C6-DevKitM-1 | `ESP32_C6_MINI_1` |

Wrong board setting uses the wrong GPIOs (UART, HV PSU, shift registers). See `src/defines.h`.

---

## Differences

| | **ESP32-C6-DevKitM-1** | **ESP32-C6-DevKitC-1** |
|---|---|---|
| Module | ESP32-C6-**MINI-1**(U) | ESP32-C6-**WROOM-1**(U) |
| Typical flash (as kit docs) | **4 MB** in the chip package (`ESP32-C6FH4`) | **8 MB** SPI flash on the module |
| Module size | Smaller MINI (~13.2 × 16.6 mm class) | Larger WROOM (~18 × 25.5 mm class) |
| Antenna | PCB (`-1`) or U.FL (`-1U`) | Same: PCB vs U.FL |
| Header layout | Mini-oriented (e.g. **GPIO14** brought out; not the same J1 map as C) | Classic DevKitC (**GPIO10**, **GPIO11** on J1; extra NC pins) |
| Name | **M** = MINI module | **C** = WROOM / DevKitC-style |

The **U** SKUs change antenna only (connector vs PCB).

---

## Shared specifications

Both kits:

| Item | Spec |
|---|---|
| SoC | ESP32-C6, 32-bit RISC-V HP core (up to **160 MHz**) + LP core |
| Wi-Fi | 2.4 GHz **Wi-Fi 6** (802.11ax) |
| Bluetooth | **Bluetooth 5 (LE)** |
| 802.15.4 | Zigbee 3.0 / Thread 1.3 |
| USB | Two Type-C ports: native **USB Serial/JTAG** (USB 2.0 full speed, 12 Mbps) and **USB-UART bridge** (up to ~3 Mbps) |
| Power | USB and/or 5 V / 3.3 V headers; 5 V → 3.3 V LDO |
| Debug | BOOT, RST, RGB LED (typically **GPIO8**), current-measure jumper **J5** |
| I/O | Most GPIOs on headers (ADC, UART, I2C, SPI, USB D+/D− on GPIO13 / GPIO12) |

---

## Kit details

### ESP32-C6-DevKitM-1

- Module: ESP32-C6-MINI-1 or MINI-1U
- Flash: **4 MB** in-package on the kit Espressif documents (some MINI part numbers exist with more flash)
- Chip: **ESP32-C6FH4**
- Use when matching a small MINI production module

### ESP32-C6-DevKitC-1 (user guide v1.2)

- Module: ESP32-C6-WROOM-1 or WROOM-1U
- Flash: **8 MB** SPI
- Chip: ESP32-C6 with flash on the module
- Use when you want extra flash and the usual DevKitC header map

---

## Firmware pin map (this project)

| Signal | WROOM-1 / DevKitC-1 | MINI-1 / DevKitM-1 |
|---|---|---|
| Presence | GPIO0 | GPIO0 |
| USB-C CC1 / CC2 | GPIO2 / GPIO1 | — |
| UART RX / TX | GPIO6 / GPIO7 | GPIO4 / GPIO5 |
| HV PSU enable | GPIO11 | GPIO7 |
| Shift-reg latch | GPIO18 | GPIO15 |
| Shift-reg data | GPIO19 | GPIO18 |
| Shift-reg clock | GPIO20 | GPIO19 |
| Shift-reg output enable (PWM) | GPIO21 | GPIO20 |
