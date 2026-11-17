# EBiCS-ClaudesFork

A fork of [EBiCS/EBiCS_Firmware](https://github.com/EBiCS/EBiCS_Firmware), the open source FOC firmware for
Lishui e-bike controllers (STM32F103). This fork adds a **simple setup app with a step-by-step first-setup
wizard**, fixes a number of **firmware bugs**, adds **two more display protocols**, an experimental
**sensorless** mode, **speed + motor temperature on one wire**, and **custom battery voltages with soft current ramps**.

> Use at your own risk, like upstream EBiCS. Unlocking the chip erases the original Lishui firmware for good.
> Test with the wheel lifted before riding.

## Contents

- [What's different from upstream](#whats-different-from-upstream)
- [What you need](#what-you-need)
- [Wiring the ST-Link](#wiring-the-st-link)
- [First setup, step by step](#first-setup-step-by-step)
- [Changing settings later](#changing-settings-later)
- [Display protocols](#display-protocols)
- [Troubleshooting](#troubleshooting)
- [For developers](#for-developers)

## What's different from upstream

### 1. EBiCS Setup app (new)

`EBiCS Setup.bat` starts a Python app (`setup_app/`) that replaces the Java configurator for normal use:

| Page | What it does |
|---|---|
| **First setup** | Wizard: check connection, unlock chip, a few bike questions, build + install firmware, motor calibration, motor direction, gear ratio measurement, sensor test |
| **Settings** | Basic / Advanced / Expert tabs, every setting explained (hover), import of old Java `.ini` files, profiles |
| **Install firmware** | Build and flash with one click, or re-flash the last build |
| **Live data** | Battery voltage, speed, assist level, throttle, brake, motor and battery current, PAS and hall counters, calibration state, hall angle check - read live from the controller over the ST-Link |
| **Tools** | Connection check, chip lock state, unlock, show the generated `config.h` |

The motor calibration runs **over the ST-Link** (a small "setup mailbox" in the firmware): no USB-UART adapter,
no debug display, no holding brake + throttle at power-up. The app shows the progress and tells you when to wait
for the wheel to stop.

The old Java configurator still works (labels and tooltips improved, batch files can retry flashing), but it isn't needed.

### 2. Firmware bug fixes

| Problem | Effect | Fix |
|---|---|---|
| No2, KingMeter (KM5S) and Bafang drivers copied up to 64 bytes into a 32 byte buffer | Overwrote other variables. With the No2 (S866) display the motor control interrupt stopped: **throttle and walk assist did nothing** | Buffers sized for the full 64 byte DMA buffer |
| KM5S checksum loop used an unchecked length byte from the display; handshake table index unchecked | Out of bounds reads, possible endless loop -> watchdog reset | Length and index bounded |
| Linker scripts gave the code all 32 KB flash, but the last 2 KB hold the emulated EEPROM | A bigger firmware would overwrite the hall angles / KV (bootloader layouts didn't even fit the chip) | App flash limited to 30 KB (26 KB with bootloader) |
| EEPROM variable table listed only 9 of 15 variables | KV and Kunteng settings lost when the EEPROM page fills up | Complete table |
| Speed value for No2/KingMeter displays overflowed 16 bit at low speed | Display showed random speeds like **99.9 km/h** | Clamped |
| `map()` divided by zero when the input range was empty | HardFault (controller stops) | Guarded |
| Autodetect saved hall angles even if not all 6 hall positions were seen | Rough motor after a bad calibration | Refuses to save, reports which halls are missing |
| KV detection stopped without saving when its 8 bit counter wrapped | KV not stored | Stops and saves before the wrap |
| Hall order check compared int16 -1 with 0xFFFF | Check was always true | Uses the EEPROM read status |
| Bafang battery percentage not clamped | Wrong battery display | Clamped to 0-100 % |
| PLL could shift by a negative count | Undefined behaviour | Clamped |
| Watchdog reset flag never cleared | Every later boot reported a watchdog reset | Latched once and cleared |
| Kunteng build referenced an undeclared variable | **Kunteng firmware did not compile** | Fixed |

### 3. New display protocols

- **Tongsheng VLCD5 / VLCD6 / XH18** (`DISPLAY_TYPE_TONGSHENG`, 9600 baud)
- **VESC UART** (`DISPLAY_TYPE_VESC`, 115200 baud): live values for VESC displays and apps (via a UART-BLE bridge)
  and current commands. Commands time out after 1 s and never override the brake. It is *not* a full VESC: VESC Tool
  cannot configure the motor.

Both are new and not yet tested on many devices - please report results.

### 4. Sensorless motors (experimental)

Tick **"Sensorless motor (no hall sensors)"** in the app. This builds the firmware with the flux observer from the
EBiCS [Sensorless_VESC](https://github.com/EBiCS/EBiCS_Firmware/tree/Sensorless_VESC) branch (`Src/sensorless.c`):
open loop start, then the observer estimates the rotor angle; its speed feeds display speed, speed limit and
standstill detection. The motor parameters (inductance, resistance, flux linkage, observer gain, angle offset) are in
**Settings > Expert**; the defaults are the values tuned in that branch. The atan2 of the branch was replaced by an
overflow-safe version (error < 0.06 deg). Hall calibration and the gearing measurement are skipped for sensorless motors.
Test with the wheel lifted and low current first.

### 5. Speed sensor and motor temperature on one wire

Many hub motors have one wire for the speed sensor *and* the temperature sensor (NTC between the signal and GND, the
speed sensor pulls the signal to ~0 V when the magnet passes). Connect that wire to an analog input, **AD1 (PB1)** or
**AD2 (PB0)**, with a pull-up resistor to 3.3 V, then in **Settings > Advanced**:

- *Speed/temperature wire on*: AD1 or AD2
- tick *use it for speed* and/or *use it for motor temperature*
- *NTC pull-up resistor* (ohm, measure it; NTC 10k B3950) and *speed pulse threshold* (ADC, default 100)
- *Motor temp derating start / maximum*: current is reduced smoothly between the two temperatures

Readings below the threshold count as speed pulses, readings above it are averaged for the temperature. Live data shows
the raw ADC value and the motor temperature for tuning.

### 6. Custom battery voltages and soft current ramps

- *Full charge voltage* and *Cutoff voltage* can be set freely (or choose "custom" battery) - e.g. for 13S at 54.0 V.
- **Regen** is ramped down to zero when the battery approaches the full charge voltage (no overcharging).
- **Motor current** is ramped down smoothly when the battery approaches the cutoff (instead of a hard stop), in every
  mode: PAS, throttle, walk assist, display and VESC commands.
- Both ramps use the same width, *Soft voltage ramp width* (default 1.0 V). Battery bars follow the two voltages.

## What you need

**Hardware**
- A Lishui controller supported by EBiCS (STM32F103C6, "old generation" pinout - see `Documentation/`)
- An ST-Link V2 (the cheap USB stick clones work)
- A few jumper wires, a multimeter is very helpful
- The controller connected to motor and battery

**Software (Windows)**
1. [STM32CubeProgrammer](https://www.st.com/en/development-tools/stm32cubeprog.html) - unlock, flash, live data
2. An ARM compiler: [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html) **or**
   [System Workbench for STM32](https://www.st.com/en/development-tools/sw4stm32.html) (Ac6). The app finds them by
   itself (it even unpacks System Workbench's compiler zip).
3. [Python 3](https://www.python.org/downloads/) - during installation tick **"Add python.exe to PATH"**
4. This repository: *Code -> Download ZIP* and unpack, or `git clone https://github.com/ionrise/EBiCS-ClaudesFork.git`

The **Home** page of the app shows with green ticks which of these it found.

## Wiring the ST-Link

The Lishui programming plug (white, 4 pins) usually has these colours - **check with a multimeter, colours vary**:

| Plug wire | Signal | ST-Link V2 pin |
|---|---|---|
| **black** (next to the arrow on the plug) | **SWDIO** - yes, black is *not* GND here | SWDIO (2) |
| **yellow** | **GND** | GND (4) |
| green | SWCLK | SWCLK (6) |
| red | 3.3 V | leave it **unconnected** when the battery powers the controller |

- GND: beep test against battery minus (thick black wire).
- Power the controller **from the battery**, switched on. The 3.3 V that cheap ST-Links report proves nothing.
- If the controller doesn't answer: check GND first, then swap SWDIO and SWCLK.

## First setup, step by step

1. Wire the ST-Link, plug it into USB, switch the controller on, **lift the wheel**.
2. Double-click **`EBiCS Setup.bat`** and click **Start first setup**.
3. **Connect** - the page checks every 2 seconds until the controller answers.
4. **Unlock** - a new controller is read-protected. Tick the box and click *Unlock chip* (this erases the original
   firmware), then switch the controller off and on.
5. **Your bike** - display protocol, battery, wheel size, speed limit, motor and battery current, pedal sensor,
   torque sensor, throttle, EU legal mode. Hover over a question for help.
6. **Install** - builds the firmware for your answers and flashes it (about a minute).
7. **Motor calibration** - confirm the wheel can spin freely, click *Start calibration*:
   - *Finding the hall sensor positions* - the motor turns slowly and may click (about 6 s)
   - *Measuring the motor speed* - the wheel spins up to full speed (can take a few minutes)
   - *Wait until the wheel has stopped* - the app waits until the hall sensors stop changing
   - The six hall angles are checked (they should be about 60 degrees apart)
8. **Motor direction** - tell the app whether the wheel turned forward; if not it reverses the direction.
9. **Gearing** - mark the tyre, click *Start measuring*, turn the wheel **backwards** by hand exactly 3 turns, click
   *Done*. This sets "pole pairs x gear ratio", which makes the displayed speed and the speed limit correct.
10. **Update** - if steps 8 or 9 changed something, the firmware is installed again (the calibration is kept).
11. **Test** - change the assist level on the display, twist the throttle, pull a brake, turn the pedals. Each gets a tick.

Then lower the wheel and do a careful first ride.

## Changing settings later

Settings -> change -> **Save + install firmware**. Your settings are stored in `setup_app/my_bike.json` (not in git).
Use *Export profile* to keep a copy. Old Java configurator files can be imported (*Import old Java settings*).

## Display protocols

| In the app | Displays | Notes |
|---|---|---|
| KingMeter 5S / KM5S / 901U | KM5S, EN06, many multi-protocol displays ("5S") | **Recommended**, most complete |
| KingMeter J-LCD / 618U | J-LCD | |
| Bafang LCD | Bafang 1200 baud displays, "Blaupunkt" | |
| Bafang 850C / 860C | Bafang color displays | |
| Kunteng | KT-LCD3 / LCD5 / LCD8 | |
| UART No.2 | S866 and other "protocol 2" displays | Assist level, light, walk assist |
| Tongsheng | VLCD5, VLCD6, XH18 | New |
| VESC UART | VESC displays / apps | New, telemetry + current command |
| EBiCS app | EBiCS BLE module + Flutter app | |
| No display | Debug text on the UART at 56000 baud | |

**Multi-protocol displays:** set the display to **5S / KM5S** and choose KM5S in the app. Protocols without a public
specification (APT, KDS ...) are not supported.

## Troubleshooting

| Symptom | Try |
|---|---|
| "ST-Link NOT found" | Other USB port, no hub; LED on the ST-Link must light |
| "controller does not answer" | Controller switched on from the battery? GND connected (yellow on Lishui plugs)? Swap SWDIO/SWCLK |
| Unlock fails | Same as above, then switch the controller off and on and retry |
| Calibration fails, "not all hall positions found" | Hall plug (5 thin wires), wheel must turn freely; switch off/on and retry |
| Motor runs rough after calibration | Run the calibration again; leave the PLL off (Expert tab) |
| Display shows nothing / error 30 | Wrong protocol or display TX/RX swapped; on multi-protocol displays choose 5S |
| Throttle does nothing | Live data: does the throttle value rise? Is "Brake" off? Battery above the cutoff? |
| Wrong speed on the display | Run the gearing step again (Settings -> "Pole pairs x gear ratio") |
| Walk assist too weak | Settings -> "Walk assist current" |
| Sensorless motor doesn't start / runs rough | Expert tab: adjust angle offset in small steps, check inductance/resistance; low current first |
| Speed jumps when the motor gets warm | Raise *speed pulse threshold* (Live data shows the raw value) |
| Motor stops early when the battery is low | Lower *Cutoff voltage* or the *Soft voltage ramp width* |

## For developers

- Firmware sources: `Src/`, `Inc/`. `Inc/config.h` is generated by the app.
- Build by hand: `Start_Compiling.bat` (or let the app do it; it uses the same makefiles in `make/`).
- Flash the last build again: `Flash_Only.bat`.
- Setup mailbox (`Src/main.c`): `ui8_setup_cmd` (write 1 to start hall + KV calibration while the motor is off),
  `ui8_setup_state` (0 idle, 1 hall scan, 2 KV, 3 done, 4 failed), `ui8_setup_progress`, `ui8_setup_halls_found`,
  `ui8_hall_calibrated`, `ui32_hall_events`, `ui32_pas_events`, `ui32_setup_magic` = `0xEB1C5002`.
  The app reads/writes them with `STM32_Programmer_CLI` in hot-plug mode, addresses come from the ELF.
- New display drivers: `Src/display_tongsheng.c`, `Src/display_vesc.c`.

---

*Original upstream README below.*

This project has reached a certain maturity, use is always at your own risk. The authors do not accept any liability for damage to property or personal injury!  

The [master]([https://github.com/ShalokShalom/EBiCS_Firmware/wiki](https://github.com/EBiCS/EBiCS_Firmware/tree/master) branch supports sensored motors, Bafang-, Kunteng- Kingmeter-Displays, and debugging.

User-specific settings can be done in a user-friendly GUI.

Throttle, torque-simulation, and torque-sensor modes are available.

The [sensorless_VESC](https://github.com/EBiCS/EBiCS_Firmware/tree/Sensorless_VESC) branch has adapted the VESC's sensorless control. It was recently optimized for a Fischer ETH 1606 bike. 
Of course, it will work with other motors, too.

A short tutorial can be found in the [Wiki](https://github.com/EBiCS/EBiCS_Firmware/wiki).

Helpful links:

[vector-control-for-dummies](https://www.switchcraft.org/learning/2016/12/16/vector-control-for-dummies "vector-control-for-dummies")

[space-vector-pwm-intro](https://www.switchcraft.org/learning/2017/3/15/space-vector-pwm-intro "space-vector-pwm-intro")

[UM1052 - STM32F PMSM single/dual FOC SDK v4.3](https://www.st.com/content/ccc/resource/technical/document/user_manual/5e/5e/d2/cb/07/35/45/a6/CD00298474.pdf/files/CD00298474.pdf/jcr:content/translations/en.CD00298474.pdf "UM1052 - STM32F PMSM single/dual FOC SDK v4.3")

[RM0008 Reference Manual STM32F103xx](https://www.st.com/content/ccc/resource/technical/document/reference_manual/59/b9/ba/7f/11/af/43/d5/CD00171190.pdf/files/CD00171190.pdf/jcr:content/translations/en.CD00171190.pdf)

[System Workbench for STM32](https://www.st.com/en/development-tools/sw4stm32.html)

[Sensorless approach VESC](http://cas.ensmp.fr/~praly/Telechargement/Journaux/2010-IEEE_TPEL-Lee-Hong-Nam-Ortega-Praly-Astolfi.pdf)

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.
