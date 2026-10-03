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
- [x] Web server starts on boot (setup AP and/or STA)
- [x] Wi-Fi reconnects after max retries; password not logged
- [x] SoftAP + web form + NVS for SSID/password (not compile-time)

## P3 — Hardware / product

- [x] PSU GPIO: known off level, no spurious pulls
- [x] Presence debounce
- [x] Waiting animation uses `&&`

## Deferred (optional later)

- [x] Change the Wi-Fi setup page title to "Wi-Fi setup"
- [x] On the joined page, remove the Forget Wi-Fi button and the Wi-Fi network-name field. Change the heading to "Join network <network_name>". Rename Address to Assigned IP-address. Move the instruction directly under the title, and tell the user to join <recently_joined_network_name> to open the configuration page.
- [x] After a successful home Wi-Fi join, keep the setup network up until the phone leaves it, then switch to station only. Show the success text and home IP on the setup page while it is still up. Wait until the setup station list has been empty for a few seconds before shutting the access point down, so a brief channel-switch drop does not close the page. Leave station reconnect enabled across that mode change.
- [x] Use pool.ntp.org as the main NTP server, and time.google.com as a backup when the first server is unavailable.
- [x] Create a configuration page served on the home network. It sets day brightness, timezone, and force-on.
- [x] Remove the HV PSU status from the joined web page.
- [x] PWM night dimming
- [x] Web UI controls (brightness, timezone, force-on)
- [x] On the setup network, after a successful join, show the instructions page ("Join network <name>" and the assigned IP). The configuration page is being shown there instead. The configuration page belongs on the home network only.
- [x] Add a cathode-poisoning routine, similar to `display_roll`. It runs every digit in order on all tubes, including the left and right dots. Configurable parameters: rolling frequency (how fast the numbers change), offset between adjacent digits, how long the routine runs, and `inverse_direction`. The routine can run for a long time, so it must yield the display to other tasks that also write to it. When the routine finishes, show the current time.

  Shared display rule for this item, the boot animation, random mode, and fading: `shift_reg_driver_write` is the only hardware write, and it already takes `write_mutex`. That mutex stops torn 48-bit frames. It does not stop two tasks from alternating frames. Today the only exclusion is the `rolling` flag in `src/clock/clock.c`, which makes the 1 s tick skip `display_show_time` while `display_roll` blocks. One display task must own the shift register, and everyone else posts a mode request.

  Recommended owner, used by the display items below:

  - One `display_task` blocks on a command queue.
  - Commands: `SHOW_TIME`, `POISON`, `RANDOM`, `FADE`.
  - The task is the only caller of `shift_reg_driver_write`, apart from `display_clear` at boot.
  - A new command replaces the running mode. Poison and random do not snap the tubes to a new pattern, and they do not write the clock themselves if a newer command arrived during the run.
  - `vTaskDelay` between frames yields the CPU. The queue is what yields the display.

  Mode transitions. The display task remembers the symbol on each tube. A new mode's first frame is that symbol. It never restarts from a fixed pattern such as digit 0 with the offset already applied.

  - Entering poison or random from `SHOW_TIME` or `FADE`: the first frame is the digits already on the tubes. Poison then advances each tube by one symbol per frame. Random holds that first frame for one `digit_duration_ms`, then starts choosing new digits. The run ends by returning to the time that is current when the return starts.
  - Entering `SHOW_TIME` or `FADE` from random: each tube keeps receiving a new random digit until that tube equals its target digit, then that tube holds. The others keep changing. When all four match, the mode is `SHOW_TIME`. Do not run the duty-cycle fade on top of this landing.
  - Entering poison from any mode, including random: start from the symbols the previous mode left. Each later frame changes a tube by at most one symbol, in `inverse_direction`. A tube on a dot steps to the next symbol; it does not jump to a digit. `offset` is not applied by adding it to the symbol index. Tube `t` waits `t * offset` frames before its first step, so the tubes drift apart without skipping.
  - Entering `SHOW_TIME` or `FADE` from poison: keep stepping one symbol per frame in `inverse_direction` until each tube shows its target digit, then hold that tube. This is the smooth return. A tube that has arrived does not keep cycling. When all four match, the mode is `SHOW_TIME`. The duty-cycle fade is not used for this return.
  - The target is the clock time sampled when the return starts. If the minute changes before every tube has arrived, sample again. A tube that no longer matches resumes (another random digit, or the next poison step).
  - `FADE`'s duty-cycle crossfade runs only when the current mode is already `SHOW_TIME` and the minute tick changes a digit. A request for `FADE` during poison or random means "return smoothly to the new time," which is the return phase above.

  Configurable parameters of the display modes are stored in NVS namespace `display_cfg`, separate from `clock_cfg`. Load them when the clock starts. If a key is missing, use the default below and write that default back. The configuration page reads and writes these values. A posted command uses the stored parameters unless the caller overrides one field for that run.

  - Poison: `digit_duration_ms` default 50, how long one digit stays on. `offset` default 3, symbol steps between adjacent tubes. `run_duration_ms` default 1050, how long the mode runs, which is one full sweep at the defaults. `-1` runs until another display command replaces it. `inverse_direction` default false. False counts 0→9→left dot→right dot. True counts the other way.
  - Random: `digit_duration_ms` default 200, how long one random digit stays on. Must be greater than 0. `run_duration_ms` default -1, how long the mode runs. `-1` runs until another display command replaces it.
  - Fade: `fade_duration_ms` default 200, length of the digit crossfade. Minimum 20, maximum 1000.
  - `SHOW_TIME` has no stored parameters.

  Each tube is 12 consecutive bits, high to low: left dot, digits 1–9, digit 0, right dot. See the masks at the top of `src/display/display.c`. A symbol index `0..11` maps to that order as `0,1,2,3,4,5,6,7,8,9,left dot,right dot`.

  Add `display_poison(const display_poison_cfg_t *)` in `src/display/display.h` and implement it in `src/display/display.c`. The display task runs it. The struct fields are the poison parameters above, loaded from NVS.

  Frame `n` does not recompute an absolute symbol from a fixed start. Each tube keeps the symbol the previous mode left. The step is `inverse_direction ? -1 : +1`. After tube `t` has waited `t * offset` frames, each following frame adds that step, modulo 12. Pack four 12-bit fields, write once, `vTaskDelay(digit_duration_ms)`. A positive `run_duration_ms` ends the free-running phase; the return phase then steps until the tubes show the new time. `run_duration_ms` of `-1` has no free-running end. A request for `SHOW_TIME` or `FADE` starts the return phase instead of cutting to the new digits. Another poison or random command replaces this run and starts from the symbols now on the tubes.

  `display_roll` stays until the boot-animation item removes the old helpers. The 5-minute timer in `setup_task` (`TIMER_ROLL_DISP`, 300000 ms) should post `POISON` with the stored poison parameters. Because the clock is showing, that run starts from the current digits and, after `run_duration_ms`, returns to the time that is then current.

  Sweep length is `12 + 3 * offset` steps so every tube visits all 12 symbols. One sweep at 50 ms and offset 3 is 1050 ms, which is the default `run_duration_ms`.

  Flaws:

  - "Frequency" is not a unit. Implement it as `digit_duration_ms`. A Hz value would be `digit_duration_ms = 1000 / Hz`.
  - A short `run_duration_ms` can end before every cathode has been lit, which misses the point of the routine. Prefer `cycles` (full sweeps) over a raw duration. Duration can still cap a long run. Changing `digit_duration_ms` or `offset` does not recompute a stored `run_duration_ms`, so a saved run length can stop before a full sweep.
  - Dots are not digits. The requested sequence must include both dots or those cathodes stay unused. The symbol order above does that.
  - `display_roll` walks bits with `MINUTE_LOW_1 >> i` and reuses that pattern on every tube. That matches today's wiring only because each tube's 12 bits have the same order. The new routine should index symbols, not shift one tube's mask into the others.
  - Returning to the clock by writing the target digits in one frame skips every symbol in between. The return phase steps one symbol at a time so a tube cannot jump from, for example, 2 to 7 or from a dot to the time digit.

  Ways to share the display:

  - Display task and command queue, as above. Use this.
  - Keep a `busy` flag like `rolling`. Other writers skip. Simple, but the tick drops time updates and nothing can preempt a long run.
  - Call `vTaskDelay` only. That yields the CPU and still lets the tick overwrite the frame. That is the flaw in the current roll path if `rolling` is forgotten.

- [x] Remove `display_test_loop` and `display_test_task`. The cathode-poisoning routine covers that digit exercise. Remove `display_waiting_frame` and use the cathode-poisoning routine as the startup animation. Rename that animation to something that describes it, such as a booting animation.

  After the poison routine exists:

  - Delete `display_test_loop`, `display_test_task`, and `display_waiting_frame` from `src/display/display.c` and `src/display/display.h`.
  - In `main/app_main.c`, delete the `#else` branch that starts `display_test_task`. The test-program Kconfig item removes the `#ifdef` itself.
  - Rename `waiting_animation_task` in `src/clock/clock.c` to `boot_animation_task`. It keeps the existing event-group handshake: set `ANIMATION_TASK_DONE` when `MAIN_TASK_SETUP_DONE` is set. `setup_task` then requests `SHOW_TIME`, and poison returns by stepping to that time.
  - Each loop calls one poison step (`digit_duration_ms` about 100, to match today's `vTaskDelay(100)`; `offset` 1 or the current dot chase's visual spacing; `inverse_direction` false) instead of `display_waiting_frame`. The boot run uses `run_duration_ms = -1` for that invocation only. Setup does not cut to the clock digits. It requests `SHOW_TIME`, and poison steps from the symbols on the tubes until they match the time, then the mode is `SHOW_TIME`. Stored poison parameters are not changed by the `-1` override.

  Flaw: today's waiting loop stops when setup finishes, which is about a second, not a fixed poison duration. A fixed-duration poison run can end before Wi-Fi init, or run long after it. Keep "until setup is done" as the boot duration, and use poison only as the frame generator.

  The old dot chase moves one dot across tubes. A poison step lights a digit or a dot on every tube. That looks different. If the boot animation should stay a single moving dot, this item changes behavior on purpose.

  `display_test_loop` holds each of 12 patterns for 500 ms with all tubes on the same symbol (`offset` 0). Poison with `digit_duration_ms = 500` and `offset = 0` replaces it. No separate test function.

- [x] Remove the test-program build switch. Drop the `BUILD_TEST_PROGRAM` / `BUILD_MAIN_PROGRAM` choice in `main/Kconfig.projbuild`, the `BUILD_MAIN_PROGRAM` define in `src/defines.h`, and the `#ifdef BUILD_MAIN_PROGRAM` branch in `main/app_main.c`. The firmware always builds the main program.

  - Delete the `choice BUILD_TEST_PROGRAM` block from `main/Kconfig.projbuild`.
  - Delete the `#if !CONFIG_BUILD_TEST_PROGRAM` block at the bottom of `src/defines.h`.
  - In `main/app_main.c`, always call `clock_start()`. Remove the unused `TAG` that exists only for the test branch.
  - Let `idf.py build` regenerate sdkconfig so `CONFIG_BUILD_TEST_PROGRAM` / `CONFIG_BUILD_MAIN_PROGRAM` disappear. Do not hand-edit unrelated sdkconfig lines.
  - Leave `HW/ESP32_workspace/nixie1.0` alone. It is the old tree, not the firmware that is flashed.

  No behavioral alternative. The test program and the poison routine would duplicate the same digit walk.

- [x] Rename the shift-register GPIO pins so each name includes `SHIFT_REG`. `GPIO_SRDATA` becomes `GPIO_SHIFT_REG_DATA`, `GPIO_SRCLK` becomes `GPIO_SHIFT_REG_CLOCK`, `GPIO_RCLK` becomes `GPIO_SHIFT_REG_LATCH`, and `GPIO_OUTPUT_EN` becomes `GPIO_SHIFT_REG_OUTPUT_ENABLE`. Apply those names everywhere the pins are used, including the driver, log text, and the devkit pin table.

  In all three board branches of `src/defines.h`:

  - `GPIO_SRDATA` becomes `GPIO_SHIFT_REG_DATA`
  - `GPIO_SRCLK` becomes `GPIO_SHIFT_REG_CLOCK`
  - `GPIO_RCLK` becomes `GPIO_SHIFT_REG_LATCH`
  - `GPIO_OUTPUT_EN` becomes `GPIO_SHIFT_REG_OUTPUT_ENABLE`

  Update every use in `src/drivers/shift_reg/shift_reg_driver.c`, including the statics `SRDATA_*` and `SRCLK_*` so the driver matches the pin names (`SHIFT_REG_DATA_*`, `SHIFT_REG_CLOCK_*`). Update `pwm_driver_init` in `src/display/display.c`, which takes the output-enable pin. Update the init log. Update the pin table in `docs/esp32-c6-devkits.md`. GPIO numbers stay the same.

  Do not rename `GPIO_SRCLK` to a serial-clock abbreviation. `SRCLK` on a 74HC595 is the shift clock, not a serial clock.

- [x] Add a random digit mode. Each tube shows a random digit. Parameters: `digit_duration_ms` (how long each digit is held) and `run_duration_ms` (how long the mode runs). The run may also continue until the mode changes. A long run must yield the display to other tasks that write to it. When a finite run finishes, show the current time. Both parameters are stored in NVS and loaded at start.

  Uses the display task and command queue from the cathode-poisoning plan. Add `display_random(const display_random_cfg_t *)`, run by that task. The struct is filled from the stored random parameters.

  - `digit_duration_ms`: how long each random digit is held before the next frame. Replaces the 1–10 level. Must be greater than 0. Default 200.
  - `run_duration_ms`: how long the mode runs. `-1` runs until another display command replaces it. A positive value is a finite run. Default -1.
  - Each frame after the first, each tube that has not yet landed gets an independent symbol in `0..9` (not the dots). The first frame is the symbols already on the tubes. A tube left on a dot by poison steps one symbol per frame until it is on a digit, then randomizes. Map digits with the same bit function `display_format_time` already uses. Write the 48-bit word, then `vTaskDelay(digit_duration_ms)`.
  - When `run_duration_ms` elapses, or when `SHOW_TIME` or `FADE` is requested, each tube that is not yet the target digit keeps taking a new random digit until it matches, then holds. The target is the clock time at the start of that return, updated if the minute changes before every tube has landed. When all four match, the mode is `SHOW_TIME`.
  - A `run_duration_ms` of `-1` has no timed end. It returns only when `SHOW_TIME` or `FADE` is requested, by the same landing. A new poison command starts from the digits on the tubes instead of landing.

  Flaw: nothing in the request starts or stops the mode. Without a caller it is dead code. The configuration page is the place to start it (hold time, run length, start, stop). That page already posts settings from `src/web/web_server.c`. Add the controls there, or the mode cannot be reached.

  Flaw: "random digit" does not include dots, so this mode does not prevent dot cathode poisoning. The poison routine does.

  Flaw: infinite mode plus "when finished, show the time" needs an explicit stop. A `SHOW_TIME` or `FADE` request is that stop, and the landing above is how the tubes get there. A poison request does not land; it continues from the digits on the tubes.

  `esp_random()` is the RNG. No extra seeding.

- [x] Add a fading effect when a digit changes. For a short transition, the previous digit and the next digit on that tube are both driven. At the start the next digit is on only briefly and the previous digit takes most of the time; through the transition the next digit's on-time ramps up and the previous digit's on-time ramps down. Parameter: duration of the transition.

  Do not turn both cathodes on in the same frame. They share one anode resistor, so both-on does not crossfade: current rises only slightly and the lower-voltage cathode hogs it. The eye can still see a blend if the two cathodes alternate faster than persistence of vision, with a changing duty.

  Implementation in the display task, triggered when the minute tick's new time differs per tube from the time on the tubes. The transition length is the stored `fade_duration_ms`.

  - Fixed slice, 10 ms. Over `fade_duration_ms`, slice `k` of `N = fade_duration_ms / 10` shows the next digit when `(k * 10) < next_duty` and the previous digit otherwise. `next_duty` goes from 10% to 100% across the slices. Last slice is only the next digit.
  - Tubes whose digit did not change stay on the current symbol for every slice.
  - Dots stay off, matching `display_format_time`.
  - Default `fade_duration_ms` is 200 (20 slices) when the NVS key is missing. Minimum 20, maximum 1000. Reject a saved value outside that range and use 200.
  - The 1 s tick must not call `display_show_time` during the fade. While the mode is `SHOW_TIME`, a minute change posts `FADE`. A `FADE` or `SHOW_TIME` request during poison or random starts that mode's return phase instead of this crossfade.

  Ways:

  - Duty-cycled alternation, as above. This is the one that matches "on-time ramps".
  - Both bits set for the whole transition. Simpler, and it does not ramp. Reject this.
  - PWM on output enable while both are on. That dims the whole tube, not one digit against the other. Reject this.

  Flaw: a 10 ms slice for 200 ms is 20 full shift-register writes. That is fine. A duration below one slice cannot ramp; clamp it.

  Flaw: `display_roll` and poison also change digits. The request only describes a digit change, which is the clock tick. Do not fade the poison walk unless asked. Fading every poison step would hide the cathode exercise and stretch the sweep by `fade_duration_ms` per step.

