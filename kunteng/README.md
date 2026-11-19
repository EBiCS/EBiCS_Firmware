# KT firmware (Kunteng / STM8S105)

Open source firmware for Kunteng (KT) e-bike controllers with an STM8S105 chip: S06S, S12S, KT36/48 and
similar boards. Based on [casainho's BMSBattery_S_controllers_firmware](https://github.com/casainho/BMSBattery_S_controllers_firmware)
(GPL v3), with bug fixes, runtime settings and motor control improvements. It is installed and configured
with the **EBiCS Setup** apps (PC and Android) over the SWIM pin of an ST-Link V2. No ST Visual Programmer needed.

Displays: Kunteng KT-LCD3 / LCD5 / LCD8, KingMeter 901U / KM5S, Bafang LCD and 850C / 860C (setting
"Display"). Both apps show live data of the running controller over the ST-Link (no restart).

> Not tested on a real KT controller yet. It builds, and the motor code runs in the STM8 simulator (see
> below), but test with the wheel lifted first and start with a low current.

## Using it

PC: start `EBiCS Setup.bat`, choose **Kunteng / KT** on the start screen, then follow the 4 steps on
the home page (check connection, unlock, settings, install). Android: choose **Kunteng / KT** on the
app's start screen.

Wiring: ST-Link V2 **3.3V, SWIM, GND, RST** to the 4 pin programming header of the KT board. Power the
controller from the battery while connected.

Unlocking removes the read-out protection of the chip. That erases the original KT firmware for good:
it cannot be read out first.

## Settings at runtime

The settings (battery, current, PAS, assist levels, motor angles ...) are not compiled in. The apps write
a 32 byte settings block into the data EEPROM (address 0x4080, layout in [settings.h](settings.h)), the
firmware reads it at power up. So one prebuilt firmware works for every bike (the Android app cannot run
a compiler), and changing a setting takes a second ("Write settings only"), without reinstalling.
Without a valid block the firmware uses the defaults in [main.h](main.h).

## Building

```
python kunteng/build.py               # -> kunteng/build/main.ihx
python kunteng/build.py -DNAME=VALUE  # change a compile time default
```

Needs SDCC 4.x (`winget install SDCC.SDCC` on Windows, `apt install sdcc` on Linux). The STM8 standard
peripheral library (SDCC version) is included in `StdPeriphLib`.

## Testing without hardware

```
python kunteng/sim/run_sim.py
```

Runs the real firmware in ucsim, the STM8 simulator that comes with SDCC, and checks: the settings block
is used, speed and interpolation from a hall sensor sequence, the hall position learning with misplaced
sensors, the defaults without a settings block, the phase current limit, the Kunteng / KingMeter / Bafang
display protocols (answers and checksums) and the learned values loaded from the EEPROM. About 3 minutes.

```
python tools/kt_motor_sim.py
```

Motor simulator: a Python port of the motor control (16 bit integer overflows are reported) runs against a
PMSM motor and bike model, once as casainho's original and once as this firmware: start, locked rotor, hall
mounting errors, current steps, speed limit, throttle release, regen and brake with a direct drive, broken hall
wire, coasting. About 1 minute.

The flashing code of the apps is tested against a simulated ST-Link + STM8 (`python setup_app/test_swim.py`,
`flutter test` in `android_app`).

## Changes against casainho's firmware

See [CHANGES.md](CHANGES.md).
