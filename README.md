# EBiCS-ClaudesFork

Fork of [EBiCS/EBiCS_Firmware](https://github.com/EBiCS/EBiCS_Firmware) with a simple setup app,
firmware bug fixes and two more display protocols.

## Quick start: EBiCS Setup

1. Install [STM32CubeProgrammer](https://www.st.com/en/development-tools/stm32cubeprog.html),
   an ARM compiler ([STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html) or System Workbench)
   and [Python 3](https://www.python.org/downloads/) (tick "Add python.exe to PATH").
2. Wire an ST-Link V2 to the controller's programming plug: SWDIO, SWCLK, GND
   (Lishui plug: BLACK = SWDIO, YELLOW = GND, GREEN = SWCLK - check with a multimeter).
   Power the controller from the battery and lift the wheel.
3. Double-click **`EBiCS Setup.bat`** and click **Start first setup**.

The wizard checks the connection, unlocks the chip (erases the original firmware), asks a few questions
about the bike, builds and installs the firmware, calibrates the motor over the ST-Link (hall angles and KV,
it tells you when to wait for the wheel to stop), checks the motor direction, measures the gearing while you
turn the wheel by hand, and tests display, throttle, brake and pedal sensor. Settings, Live data and Tools
pages are there for later changes. The old Java configurator still works but is no longer needed.

### What changed compared to upstream

- **Bug fixes**: RX buffer overflows in the No2, KingMeter and Bafang drivers (No2: throttle/push assist dead),
  KM5S checksum loop/handshake out of bounds, flash layout overlapping the emulated EEPROM, incomplete EEPROM
  variable table (KV lost), map() division by zero, Bafang battery % wrap, negative shift in the PLL,
  watchdog flag never cleared, Kunteng build error, wheel time overflow (99.9 km/h on the display),
  autodetect saving incomplete hall data.
- **New display protocols**: Tongsheng VLCD5 / VLCD6 / XH18, and a minimal VESC UART mode
  (live values + current command) for VESC displays and apps.
- **Setup mailbox**: calibration can be started and followed over SWD, no UART or debug display needed.

---

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
