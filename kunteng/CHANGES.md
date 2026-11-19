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

## Found with the motor simulator (tools/kt_motor_sim.py)

| Problem | Effect (simulated) | Fix |
|---|---|---|
| Broken hall wire: angle froze, PWM stayed on | 40+ A phase current spikes for 2 s until the blocked motor protection | PWM off after 0.5 ms of invalid hall states, error 24, restarts once the halls are valid and the throttle is released |
| Brake set the duty cycle target to 0 once, the controllers restored it | Direct drive: brake + throttle kept pushing (13 A); regen did nothing | The brake overrides the target every 10 ms; without a known back-EMF constant the PWM goes off while braking |
| FOC angle correction also ran with regen current (opposite sign) | Regen current swung -17 ... +2 A, angle through ±90° | Correction only while motoring (> 1 A) |
| Duty cycle far below the back-EMF (power on while rolling, restart) shorts the motor: phase current flows, battery current ~0 | Direct drive braked with 7 A phase current, the current controller saw 0 A and stayed there | Phase B current peak per revolution; phase current without battery current raises the duty cycle |
| Regen ramped the duty cycle down to 0 | Below half the back-EMF the battery current falls again, at 0 the motor is shorted | Duty cycle floor at half the back-EMF while braking (back-EMF constant learned at ~0 A) |
| Speed limiter as an integrator from below the limit | Duty cycle held back to +2 per 100 ms, then 1.1 km/h over the limit | Soft limit: current reduced over the last 1 km/h (filtered speed), hard limiter only above the limit |
| PWM on with no current requested (geared motor) | Motor kept spinning at no load speed in the freewheel | PWM off after 400 ms, restart at the duty cycle matching the spinning motor |

Simulated comparison, original vs this firmware (same motor model): current step 3 -> 12 A settles in 0.2 s
(original 1.7 s and then oscillating 5 ... 15 A); torque ripple with misplaced halls 0.75 A rms (5.9 A);
speed limit 24.8 ... 24.9 km/h at a 25 km/h limit (24.7 ... 26.1); regen spike after releasing the throttle
-1.3 A (-9.3 A).

Known limits: a direct drive motor restarted at speed before the back-EMF constant was ever learned (no
moment at ~0 A since power on) still sees a short braking current spike (~8 A phase, simulated). Regen current
stays below the setting when the winding inductance limits it (sine commutation without field angle control).

## Displays, saved learning, more settings, live data

- **Display protocols** (display.c), chosen in the settings: Kunteng KT-LCD3/5/8 (as before), KingMeter
  901U / KM5S (with the 901U handshake) and Bafang LCD (1200 baud) / 850C / 860C (9600 baud), ported from
  EBiCS. Receiving and sending are interrupt driven: the main loop never waits for the UART (the KT status
  used to block it for 12.5 ms every 150 ms). KingMeter and Bafang assist levels are mapped to the 5 levels;
  the KingMeter speed limit and wheel circumference are used.
- **Learned values kept**: hall edge offsets and the back-EMF constant are saved in the data EEPROM at
  standstill (once per stop, only when changed) and loaded at power on.
- **New settings** (settings block bytes 23 ... 26, 0 = old behaviour): display protocol, power ramp (time to
  full power), FOC sample delay compensation (signed; in the motor simulator +164 us made the angle worse and
  -40 us slightly better, so it is off and meant for tuning on the controller), motor constant (P1) for non-KT
  displays.
- **Live data** in both apps over SWIM without reset: battery, current, speed, duty cycle, throttle, cadence,
  assist level, states and errors, FOC correction, phase current peak, learned values.

More bugs of the original found on the way (simulator):

| Where | Bug | Fix |
|---|---|---|
| ebike_app.c | Standstill wheel period `36000 x circumference` overflowed 16 bits above 1.82 m (26": 74250 -> 8714 ms, the display showed 0.9 km/h) | 65000 ms |
| ebike_app.c | External speed sensor assumed connected at power on with the shortest period: ~850 km/h for 4 s | Not connected until the first pulse |

Removed: the 360° interpolation mode (was off by default, replaced by the new interpolation).
