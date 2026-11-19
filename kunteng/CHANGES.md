# Changes against casainho's BMSBattery_S_controllers_firmware

## Bug fixes

| Where | Bug | Effect | Fix |
|---|---|---|---|
| motor.c, ebike_app.c | Motor current stored as `int8_t` in 0.125 A steps | Wrapped to negative above 15.9 A. The current controller then saw 0 A and **raised** the duty cycle (S12S limit is 25 A). The display value wrapped too. | `int16_t`, display byte saturates at 255 |
| motor.c ISR | Invalid hall state (0 or 7) did `return` from the PWM interrupt without clearing its flag | With a broken hall wire the interrupt re-entered forever, main loop and watchdog starved: reset loop | Ignore the state, continue the interrupt |
| ebike_app.c | Throttle error limits defined but never checked | Throttle shorted to 5 V or a lost ground read as **full throttle** | Above `ADC_THROTTLE_MAX_ERROR`: throttle ignored, error 01 shown, 1 s in range to recover |
| ebike_app.c | Cruise control: `(v > c - 20) \|\| (v < c + 20)` | Always true: cruise control locked after 8 s at any throttle movement | `&&` (with int16 math) |
| ebike_app.c | `10 + a \| b` (max speed from the display) | Parsed as `(10 + a) \| b`: wrong speed limit above 41 km/h | `10 + (a \| b)` |
| adc.c | Pin masks and Schmitt trigger channels joined with `\|\|` | Evaluated to 1: only pin 0 / channel 1 configured | `\|`, one call per channel |
| adc.c | Current offset calibration: delay loop missing in the averaging loop | 16 "averaged" samples taken back to back right after power up | Wait 10 ms per sample |
| motor.c | Over voltage reported as error 0x91 | Display showed "battery empty" for an over voltage | Own state and error 05, recovers below 4.2 V/cell - 0.8 V |
| motor.c | Hardware over current trip latched until power off | One spike stopped the ride | Retry after 1 s with the throttle released, max 3 times per power up |
| motor.c | `counter << 8` with 16 bit int | Interpolation angle overflowed when the counter passed 255 | Clamped (a 32 bit divide is too slow for the ISR) |
| motor.c | Current filter started at a `<< 5` constant for a `>> 4` filter | Wrong current for the first second | Starts at the measured offset |
| uart.h | `getchar` prototype | Did not compile with current SDCC | Matches SDCC's stdio.h |

## Improvements

- **Runtime settings** from a settings block in the data EEPROM (settings.c/.h), written by the EBiCS Setup
  apps. Battery thresholds for any cell count 7S ... 16S (was 7S/10S/13S only).
- **9 bit PWM**: duty cycle 0 ... 508 and 9 bit compare values (was 8 bit shifted left by one).
- **Interpolation**: angle advanced by a fixed step every PWM cycle. The step comes from the sum of the last
  6 hall sectors, updated at every hall edge (was once per electrical revolution, with a division in every
  PWM interrupt). The angle never runs more than one sector past the last hall edge when the motor slows down.
- **Hall position learning**: at steady speed the time in each hall state is proportional to its angle.
  The real angle of every hall edge is learned (filtered, max ±28°) and used instead of exact 60° steps.
  Smoother and quieter on motors with badly placed sensors. Can be switched off in the settings.
- **FOC angle correction from both zero crossings** of the phase B current (was one): corrects twice as
  fast, and an offset error of the current sensor no longer shifts the angle (it cancels out).
- **Phase current limit**: phase current ≈ battery current / duty cycle. At low duty cycle (starting,
  climbing slowly) the phase current can be several times the battery current; limited to 2.5 × the
  battery current setting. Protects the MOSFETs. Can be switched off in the settings.
- **Current controller as PI every 10 ms** (was an integrator every 100 ms with a 1.6 s current filter).
  Same integral rate, plus a proportional part that damps it.
- Build script (`build.py`) without make, works on Windows and Linux.
- Simulator test (`sim/run_sim.py`).

Removed: the 360° interpolation mode (was off by default, replaced by the new interpolation).
