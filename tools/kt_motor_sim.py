"""
KT (Kunteng, STM8S105) motor control simulator - host side, no hardware needed.

Runs a Python port of the KT firmware's motor control (kunteng/motor.c, settings.c, ebike_app.c current
mapping) against a PMSM motor + bike model, the way tools/motor_sim.py does for EBiCS:
  - PWM interrupt (64 us): hall decoding, sector timing, interpolation, FOC angle correction from the phase B
    current, battery / phase / regen current limits, duty cycle ramps, 9 bit SVM compare values
  - 10 ms task: current filter, PI current controller, hall position learning
  - 100 ms task: speed limiter, motor state machine (blocked motor), battery voltage protection
The STM8 has a 16 bit int: every int / uint16 / uint8 operation is checked, an overflow or a silent
truncation is recorded as a finding instead of wrapping unnoticed.

Two firmware versions run against the same motor:
  ORIGINAL = casainho's firmware (8 bit duty, interpolation once per revolution, one FOC sample, integrating
             current controller every 100 ms on an int8 current, no phase current limit)
  CURRENT  = kunteng/ in this repository

Hardware model (assumptions, the KT schematics are not public in detail):
  - geared hub (Q100 like) on a 36 V battery, freewheel; or a direct drive hub
  - total (battery) current measured on the low side shunt through an RC filter (0.5 ms): the ADC reads the
    average bus current. 8 bit = 0.5 A/step, 10 bit = 0.125 A/step, zero at 79 / 317
  - phase B current sensor: 0.121 A/step (8 bit), zero at 126, low pass 0.1 ms
  - ADC values are one PWM period old (conversion started at the start of the interrupt)

Usage:  python tools/kt_motor_sim.py            (all scenarios, prints a report)
Part of EBiCS / the KT firmware, GPL v3.
"""

import math
import sys
from collections import Counter

PWM_HZ = 15625.0
TS = 1.0 / PWM_HZ
PWM_PERIOD = 512               # TIM1 ARR+1, center aligned
FINDINGS = Counter()

# ------------------------------------------------------------------------------------------------ STM8 integer helpers


def finding(tag):
    FINDINGS[tag] += 1


def u8(x, tag=None):
    if tag and not 0 <= x <= 255:
        finding("uint8 overflow: " + tag)
    return x & 0xFF


def u16(x, tag=None):
    if tag and not 0 <= x <= 0xFFFF:
        finding("uint16 overflow: " + tag)
    return x & 0xFFFF


def i16(x, tag):
    """C int (16 bit on the STM8): signed overflow is undefined behaviour, record it."""
    if not -32768 <= x <= 32767:
        finding("int16 overflow: " + tag)
        x = ((x + 32768) % 65536) - 32768
    return x


def i8(x, tag):
    if not -128 <= x <= 127:
        finding("int8 overflow: " + tag)
        x = ((x + 128) % 256) - 128
    return x


def cdiv(a, b):
    if b == 0:
        finding("division by zero")
        return 0
    q = abs(a) // abs(b)
    return q if (a >= 0) == (b >= 0) else -q


# ------------------------------------------------------------------------------------------------ firmware tables
SVM = [127, 133, 138, 144, 149, 154, 160, 165, 170, 176, 181, 186, 191, 197, 202, 207, 212, 217, 222, 227, 231, 236,
       239, 240, 242, 243, 244, 245, 247, 248, 249, 250, 250, 251, 252, 253, 253, 254, 254, 254, 255, 255, 255, 255,
       255, 255, 254, 254, 254, 253, 253, 252, 251, 251, 250, 249, 248, 247, 246, 245, 243, 242, 241, 239, 238, 239,
       241, 242, 243, 245, 246, 247, 248, 249, 250, 251, 251, 252, 253, 253, 254, 254, 254, 255, 255, 255, 255, 255,
       255, 254, 254, 254, 253, 253, 252, 251, 250, 250, 249, 248, 247, 245, 244, 243, 242, 240, 239, 236, 231, 227,
       222, 217, 212, 207, 202, 197, 191, 186, 181, 176, 170, 165, 160, 154, 149, 144, 138, 133, 127, 122, 116, 111,
       106, 101, 95, 90, 85, 79, 74, 69, 64, 58, 53, 48, 43, 38, 33, 28, 24, 19, 16, 15, 13, 12, 11, 10, 8, 7, 6, 5,
       5, 4, 3, 2, 2, 1, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 2, 2, 3, 4, 4, 5, 6, 7, 8, 9, 10, 12, 13, 14, 16, 17, 16,
       14, 13, 12, 10, 9, 8, 7, 6, 5, 4, 4, 3, 2, 2, 1, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 2, 2, 3, 4, 5, 5, 6, 7, 8,
       10, 11, 12, 13, 15, 16, 19, 24, 28, 33, 38, 43, 48, 53, 58, 64, 69, 74, 79, 85, 90, 95, 101, 106, 111, 116, 122]
assert len(SVM) == 256
HALL_NOMINAL = [0, 170, 85, 127, 0, 212, 42, 0]
HALL_NEXT = [0, 5, 3, 1, 6, 4, 2, 0]
HALL_ORDER = [4, 6, 2, 3, 1, 5]            # forward sequence starting at the state 4 edge (0 deg)

PWM_CYCLES_COUNTER_MAX = 3125
PWM_CYCLES_SECOND = 15625
MOTOR_OVER_SPEED_ERPS = 520
SPEED_KP = 2                                # S06S
SPEED_OUT_MAX = 8160
CURRENT_KP_ORIG = 10
CURRENT_OUT_MAX_ORIG = 16320
RAMP_UP, RAMP_DOWN = 25, 10                 # S06S, PWM interrupts per 8 bit duty step
FOC_HIGH, FOC_LOW = 127, 125


class Firmware:
    def __init__(self, original=False, cells=10, current_a=15, regen_a=0, erps_interp=15, rotor_offset=202,
                 foc_adjust=137, phase_factor_x10=25, direct_drive=False, hall_learning=True, phase_limit=True,
                 offset8=79, offset10=317, foc_delay_us=0):
        self.orig = original
        self.foc_delay_x4 = max(-128, min(127, int(foc_delay_us / 4)))
        self.foc_adjust_eff = foc_adjust
        self.cells, self.current_a, self.regen_a = cells, current_a, regen_a
        self.erps_interp, self.rotor_offset, self.foc_adjust = erps_interp, rotor_offset, foc_adjust
        self.direct_drive = direct_drive
        self.hall_learning = hall_learning and not original
        self.phase_limit_on = phase_limit and not original
        self.vmax = (cells * 420) // 27
        self.vmin = (cells * 300) // 27
        self.phase_max = min(250, (current_a * 2 * phase_factor_x10) // 10)
        self.offset8, self.offset10 = offset8, offset10
        self.target_max = min(255, offset8 + current_a * 2)
        self.regen_target = max(0, offset8 - 4)              # motor_set_regen_current_max (4) at init
        self.duty_max = 254 if original else 508
        self.ramp_up = RAMP_UP if original else RAMP_UP // 2
        self.ramp_down = RAMP_DOWN if original else RAMP_DOWN // 2
        # state
        self.cnt = self.cnt6 = 0
        self.total = 0xFFFF
        self.erps = 0
        self.abs_angle = self.rotor_angle = 0
        self.correction = 127
        self.commutation = 1                  # 1 block, 2 sine 60
        self.motor_state = 1                  # STOP
        self.startup_counter = 0
        self.ctrl_state = 1                   # OK
        self.moe = True
        self.hall_last = 0
        self.half_flag = self.foc_flag = self.foc_flag2 = 0
        self.angle_x256 = 0
        self.step = 0
        self.limit = 42
        self.sector = [PWM_CYCLES_COUNTER_MAX] * 8
        self.period_sum = 6 * PWM_CYCLES_COUNTER_MAX
        self.valid = 0
        self.snap_ready = 0
        self.snap = [0] * 8
        self.snap_sum = 0
        self.last_total_learn = 0
        self.hall_off = [0] * 8
        self.hall_off_x16 = [0] * 8
        self.duty = 0
        self.duty_target = 0
        self.ru = self.rd = 0
        self.cur_acc = offset10 << 4 if not original else 10144   # original started at a <<5 constant
        self.cur_filt = 0
        self.vbat_acc = ((self.vmax + self.vmin) >> 1) << 6
        self.vbat_filt = (self.vmax + self.vmin) >> 1
        self.target_erps = 0
        self.target_current = 0               # 10 bit units (0.125 A)
        self.speed_out = self.duty_max
        self.err_last = 0
        self.frac = 0
        self.throttle_released = True
        self.brake = False
        self.oc_counter = self.oc_retries = 0
        self.ccr = (254, 254, 254)
        self.last_adc_id = 126
        self.hall_invalid = 0
        self.coasting = False
        self.coast_counter = 0
        self.hall_now = 0
        self.coast_k = 0          # learned back-EMF constant (duty per erps x256)
        self.erps_x10_f = 0
        self.zero_ticks = 0
        self.pb_run = 0
        self.adc8 = offset8
        self.pb_peak = 0
        self.brake_off = False

    def press_brake(self, on):
        """brake.c EXTI interrupt (edge)."""
        if on:
            self.ctrl_state |= 2
            self.regen_target = max(0, self.offset8 - self.regen_a * 2)
            self.duty_target = 0
            if not self.direct_drive:
                self.moe = False
                self.duty = 0
        else:
            self.ctrl_state &= ~2
            self.regen_target = self.offset8          # motor_set_regen_current_max (0)

    # ------------------------------------------------------------------ PWM interrupt
    def _reset_interp(self):
        self.sector = [PWM_CYCLES_COUNTER_MAX] * 8
        self.period_sum = 6 * PWM_CYCLES_COUNTER_MAX
        self.valid = 0
        self.step = 0

    def isr(self, hall, adc_total8, adc_phase_b8, adc_vbat8):
        # hall edges
        self.hall_now = hall
        if hall != self.hall_last and hall in (0, 7) and not self.orig:
            self.hall_invalid = min(255, self.hall_invalid + 1)
            if self.hall_invalid == 8:
                self.ctrl_state |= 128
                self.moe = False
        elif hall != self.hall_last and not self.orig:
            self.hall_invalid = 0
        if hall != self.hall_last and hall not in (0, 7):
            prev, self.hall_last = self.hall_last, hall
            if not self.orig:
                if 1 <= prev <= 6 and HALL_NEXT[prev] == hall and self.cnt6 < PWM_CYCLES_COUNTER_MAX:
                    self.period_sum = u16(self.period_sum - self.sector[prev] + self.cnt6, "period_sum")
                    self.sector[prev] = self.cnt6
                    self.valid = min(255, self.valid + 1)
                    if self.valid >= 6:
                        self.step = 0xFFFF // self.period_sum
                else:
                    self._reset_interp()
            if hall == 1:
                if self.half_flag:
                    self.half_flag = 0
                    self.total = self.cnt
                    self.cnt = 0
                    self.erps = PWM_CYCLES_SECOND // self.total if self.total else 0
                    if not self.orig and self.valid >= 12 and not self.snap_ready and \
                            self.total + (self.total >> 4) > self.last_total_learn and \
                            self.total < self.last_total_learn + (self.last_total_learn >> 4):
                        self.snap = list(self.sector)
                        self.snap_sum = self.period_sum
                        self.snap_ready = 1
                    self.last_total_learn = self.total
                if self.erps > self.erps_interp:
                    if self.commutation == 1:
                        self.commutation = 2
                        self.motor_state = 4
                else:
                    if self.commutation == 2:
                        self.commutation = 1
                        self.correction = 127
                self.foc_flag2 = 1
                if not self.orig:
                    self.pb_peak, self.pb_run = self.pb_run, 0
            elif hall == 6:
                self.half_flag = 1
                self.foc_flag = 1
            self.abs_angle = (HALL_NOMINAL[hall] + (self.hall_off[hall] & 0xFF) + self.rotor_offset) & 0xFF
            self.angle_x256 = self.abs_angle << 8
            nxt = HALL_NEXT[hall]
            self.limit = ((HALL_NOMINAL[nxt] - HALL_NOMINAL[hall]) & 0xFF) + 4 + self.hall_off[nxt] - self.hall_off[hall]
            self.cnt6 = 0
        elif hall in (0, 7) and self.orig:
            finding("invalid hall state: original firmware returns from the ISR without clearing the flag (reset loop)")

        # counters
        if self.cnt < PWM_CYCLES_COUNTER_MAX:
            self.cnt += 1
            self.cnt6 += 1
        else:
            self.cnt = self.cnt6 = 0
            self.half_flag = 0
            self.erps = 0
            self.total = 0xFFFF
            self.correction = 127
            self.commutation = 1
            self.hall_last = 0
            if not self.orig:
                self._reset_interp()
            if self.motor_state == 4:
                self.motor_state = 1

        # angle
        if self.commutation == 2:
            if self.orig:
                prod = self.cnt6 << 8
                if prod > 0xFFFF:
                    finding("uint16 overflow: original interpolation counter<<8")
                interp = u8((prod & 0xFFFF) // self.total)
                self.rotor_angle = (self.abs_angle + interp) & 0xFF
            elif self.step:
                self.angle_x256 = (self.angle_x256 + self.step) & 0xFFFF
                self.rotor_angle = self.angle_x256 >> 8
                if ((self.rotor_angle - self.abs_angle) & 0xFF) > self.limit:
                    self.rotor_angle = (self.abs_angle + self.limit) & 0xFF
                    self.angle_x256 = self.rotor_angle << 8
            else:
                self.rotor_angle = self.abs_angle
        else:
            self.rotor_angle = self.abs_angle
        index = (self.rotor_angle + self.correction) & 0xFF

        # peak phase B current per revolution
        self.pb_run = max(self.pb_run, abs(adc_phase_b8 - 126))
        # FOC correction
        rel = (self.rotor_angle - self.rotor_offset) & 0xFF
        if self.orig:
            if self.foc_flag and rel >= self.foc_adjust:
                self.foc_flag = 0
                if self.erps > self.erps_interp:
                    self.last_adc_id = adc_phase_b8
                    if adc_phase_b8 > FOC_HIGH:
                        self.correction = (self.correction + 1) & 0xFF
                    elif adc_phase_b8 < FOC_LOW:
                        self.correction = (self.correction - 1) & 0xFF
        elif self.erps > self.erps_interp and adc_total8 > self.offset8 + 2:
            if self.foc_flag and rel >= self.foc_adjust_eff:
                self.foc_flag = 0
                self.last_adc_id = adc_phase_b8
                if adc_phase_b8 > FOC_HIGH:
                    self.correction = (self.correction + 1) & 0xFF
                elif adc_phase_b8 < FOC_LOW:
                    self.correction = (self.correction - 1) & 0xFF
            if self.foc_flag2 and ((rel - 128) & 0xFF) >= self.foc_adjust_eff:
                self.foc_flag2 = 0
                if adc_phase_b8 > FOC_HIGH:
                    self.correction = (self.correction - 1) & 0xFF
                elif adc_phase_b8 < FOC_LOW:
                    self.correction = (self.correction + 1) & 0xFF
        else:
            self.foc_flag = self.foc_flag2 = 0

        # duty cycle controller
        cur = adc_total8
        self.adc8 = adc_total8
        if cur > self.target_max:
            if self.duty > 0:
                self.duty -= 1
        elif self.phase_limit_on and cur > self.offset8 and \
                u16((cur - self.offset8) << 8, "phase limit lhs") > self.phase_max * ((self.duty >> 1) & 0xFF):
            if self.duty > 0:
                self.duty -= 1
        elif cur < self.offset8 and adc_vbat8 >= self.vmax:
            if self.duty < self.duty_max:
                self.duty += 1
        elif cur < self.regen_target:
            if self.duty < self.duty_max:
                self.duty += 1
        else:
            if self.duty_target > self.duty:
                if self.ru >= self.ramp_up:
                    self.ru = 0
                    self.duty += 1
                else:
                    self.ru += 1
            elif self.duty_target < self.duty:
                if self.rd >= self.ramp_down:
                    self.rd = 0
                    self.duty -= 1
                else:
                    self.rd += 1

        # compare values
        def pwm(t):
            if self.orig:
                d = self.duty
                v = 127 + (((t - 127) * d) >> 8) if t > 127 else 127 - (((127 - t) * d) >> 8)
                return v << 1
            d = self.duty
            return 254 + (((t - 127) * d) >> 8) if t > 127 else 254 - (((127 - t) * d) >> 8)
        a = pwm(SVM[index])
        b = pwm(SVM[(index + 85) & 0xFF])
        c = pwm(SVM[(index + 171) & 0xFF])
        self.ccr = (a, c, b)                 # CCR1 = phase A, CCR2 = value_c, CCR3 = value_b
        if self.ctrl_state == 1 and not self.coasting:
            self.moe = True
        return self.ccr, self.moe

    # ------------------------------------------------------------------ 10 ms task (current controller, learning)
    def current_filter(self, adc10):
        self.cur_acc = u16(self.cur_acc - (self.cur_acc >> 4) + adc10, "current filter accumulator")
        self.cur_filt = self.cur_acc >> 4
        if self.orig:
            return i8(self.cur_filt - self.offset10, "original int8 motor current (wraps above 15.9 A)")
        return i16(self.cur_filt - self.offset10, "motor current")

    def soft_limit(self, target):
        if self.target_erps < 5:
            return target
        erps_x10 = self.erps_x10_f
        limit_x10 = min(self.target_erps, MOTOR_OVER_SPEED_ERPS) * 10
        band = limit_x10 // 25
        if erps_x10 >= limit_x10:
            return 0
        if erps_x10 + band <= limit_x10:
            return target
        return target * (limit_x10 - erps_x10) // band

    def task_10ms(self, adc10):
        if self.orig:
            return
        current = self.current_filter(adc10)
        self.learn()
        comp = int((self.erps * self.foc_delay_x4) / 977)
        self.foc_adjust_eff = (self.foc_adjust + max(-64, min(64, comp))) & 0xFF
        x10 = 156250 // self.total if self.total not in (0, 0xFFFF) else 0
        self.erps_x10_f = self.erps_x10_f - (self.erps_x10_f >> 3) + (x10 >> 3)
        if x10 == 0:
            self.erps_x10_f = 0
        self.zero_ticks = 0 if self.target_current else min(255, self.zero_ticks + 1)
        if self.zero_ticks >= 30 and self.erps > 30 and self.commutation == 2 and not self.coasting and \
                self.ctrl_state == 1 and abs(self.adc8 - self.offset8) <= 2 and self.duty and self.pb_peak < 24:
            k = u16((self.duty << 8) // self.erps, "emf constant")
            self.coast_k = k if self.coast_k == 0 else self.coast_k - (self.coast_k >> 3) + (k >> 3)
        target = self.soft_limit(self.target_current)
        # coasting (geared motor, no current requested)
        braking = bool(self.ctrl_state & 2)
        if self.direct_drive or target or braking:
            self.coast_counter = 0
            if self.coasting and not braking:
                d = (self.erps * self.coast_k) >> 8 if self.erps > 15 else 0
                self.duty = self.duty_target = min(self.duty_max, d)
                self.coasting = False
        elif self.coast_counter < 40:
            self.coast_counter += 1
        elif not self.coasting:
            self.coasting = True
            self.moe = False
            self.duty = self.duty_target = 0
        cur = max(0, current)
        err = max(-400, min(400, target - cur))
        self.frac = i16(self.frac + err * 2 + (err - self.err_last) * 16, "PI fraction")
        self.err_last = err
        delta = cdiv(self.frac, 16)
        self.frac -= delta * 16
        if target == 0:
            self.frac = 0
        out = max(0, min(self.duty_max, self.duty + delta))
        self.duty_target = min(out, self.speed_out, self.duty_max)
        if not self.coasting and self.pb_peak > 40 and current < 4 and self.erps > 30 and not braking:
            self.duty_target = min(self.duty_max, max(self.duty_target, self.duty + 16))
            self.frac = 0
        if braking:
            floor = ((self.erps * self.coast_k) >> 8) // 2
            self.duty_target = min(self.duty_max, floor)
            if not self.regen_a or not self.coast_k or self.erps <= 30:
                self.duty_target = self.duty = 0
                self.brake_off = True
                self.moe = False
            self.frac = 0
        elif self.brake_off:
            self.brake_off = False
            self.duty = min(self.duty_max, (self.erps * self.coast_k) >> 8) if self.erps > 15 else 0
            self.duty_target = self.duty

    def learn(self):
        if not self.snap_ready:
            return
        if not self.hall_learning or self.erps < 30 or self.snap_sum == 0:
            self.snap_ready = 0
            return
        pos, errs = 0, []
        for st in HALL_ORDER:
            errs.append((pos * 256 + (self.snap_sum >> 1)) // self.snap_sum - HALL_NOMINAL[st])
            pos += self.snap[st]
        self.snap_ready = 0
        mean = cdiv(sum(errs), 6)
        for st, e in zip(HALL_ORDER, errs):
            o = max(-20, min(20, e - mean))
            self.hall_off_x16[st] = i16(self.hall_off_x16[st] + o - (self.hall_off_x16[st] >> 4), "hall learning")
            self.hall_off[st] = self.hall_off_x16[st] >> 4

    # ------------------------------------------------------------------ 100 ms task
    def task_100ms(self, adc10, adc_vbat8):
        # over current recovery is not modelled (no hardware comparator in the model)
        if not self.orig and self.ctrl_state & 128 and self.hall_now not in (0, 7) and self.throttle_released:
            self.duty = self.duty_target = 0
            self.hall_invalid = 0
            self.ctrl_state &= ~128
        e = self.erps
        if self.motor_state == 1:
            if e < 5 and not self.throttle_released:
                self.startup_counter = 0
                self.motor_state = 2
            elif e > 4:
                self.motor_state = 4
        elif self.motor_state == 2:
            self.startup_counter += 1
            if self.startup_counter > 20:
                self.ctrl_state |= 32
                self.moe = False
                self.motor_state = 3
                self.startup_counter = 0
            if e > 4:
                self.motor_state = 4
        elif self.motor_state == 3:
            self.startup_counter += 1
            if self.startup_counter > 10:
                self.startup_counter = 11
                if self.throttle_released:
                    self.duty = self.duty_target = 0
                    self.ctrl_state &= ~32
                    self.motor_state = 1
        # battery
        self.vbat_acc = u16(self.vbat_acc - (self.vbat_acc >> 6) + adc_vbat8, "battery filter")
        self.vbat_filt = self.vbat_acc >> 6
        if self.vbat_filt < self.vmin:
            self.ctrl_state |= 8
            self.moe = False
        # speed limiter
        if self.target_erps < 5:
            self.speed_out = self.duty_max
        else:
            err = (MOTOR_OVER_SPEED_ERPS if self.target_erps > MOTOR_OVER_SPEED_ERPS else self.target_erps) - e
            if err >= 0 and not self.orig:
                self.speed_out = self.duty_max
            else:
                o = max(-SPEED_OUT_MAX, min(SPEED_OUT_MAX, i16(err * SPEED_KP, "speed controller")))
                o >>= 5 if self.orig else 4
                self.speed_out = max(0, min(self.duty_max, self.duty + o))
        if self.orig:
            current = self.current_filter(adc10)
            cur = max(0, current)
            err = self.target_current - cur
            o = max(-CURRENT_OUT_MAX_ORIG, min(CURRENT_OUT_MAX_ORIG, i16(err * CURRENT_KP_ORIG, "current controller")))
            o >>= 5
            cur_out = max(0, min(254, self.duty + o))
            self.duty_target = min(cur_out, self.speed_out, 254)


# ------------------------------------------------------------------------------------------------ motor + bike model
class Motor:
    """Surface PMSM (dq, electrical angle) driving a bike through a freewheel (geared) or directly."""

    def __init__(self, R=0.30, L=0.40e-3, lam=0.0083, erps_per_ms=None, direct_drive=False, mass=100.0,
                 locked=False, v=0.0, theta=0.3, slope=0.0):
        self.R, self.L, self.lam = R, L, lam
        self.id = self.iq = 0.0
        self.theta = theta
        self.dd = direct_drive
        self.mass = mass
        self.locked = locked
        self.slope = slope
        # wheel: erps = wheel_rps * 101 (casainho's motor characteristic 202/2), 26" wheel 2.0625 m
        self.perimeter = 2.0625
        self.k = 101.0 * 2 * math.pi / self.perimeter          # electrical rad/s per m/s
        self.v = v                                             # bike speed m/s
        self.w = self.k * v                                    # motor electrical rad/s
        self.motor_mass = 3.0                                  # geared motor inertia as equivalent bike mass (kg)

    def force_per_amp(self):
        return 1.5 * self.lam * self.k                         # N per A of iq

    def step(self, v_alpha, v_beta, dt, n=8):
        h = dt / n
        for _ in range(n):
            c, s = math.cos(self.theta), math.sin(self.theta)
            vd = v_alpha * c + v_beta * s
            vq = -v_alpha * s + v_beta * c
            did = (vd - self.R * self.id + self.w * self.L * self.iq) / self.L
            diq = (vq - self.R * self.iq - self.w * (self.L * self.id + self.lam)) / self.L
            self.id += did * h
            self.iq += diq * h
            self._mech(h)
            self.theta = (self.theta + self.w * h) % (2 * math.pi)

    def _mech(self, h):
        if self.locked:
            self.w = 0.0
            return
        f_m = self.force_per_amp() * self.iq
        f_load = (0.008 * self.mass * 9.81 + 0.5 * 1.2 * 0.5 * self.v * self.v) * (1 if self.v > 0 else 0) + \
            self.mass * 9.81 * self.slope
        w_wheel = self.k * self.v
        if self.dd or (self.w >= w_wheel - 1e-6 and f_m > 0):
            self.v = max(0.0, self.v + (f_m - f_load) / (self.mass + self.motor_mass) * h)
            self.w = self.k * self.v
        else:   # freewheel open: motor spins on its own (2 N friction), bike coasts
            self.v = max(0.0, self.v - f_load / self.mass * h)
            friction = 2.0 if self.w > 0 else 0.0
            self.w = max(0.0, self.w + (f_m - friction) / self.motor_mass * self.k * h)

    def phase_currents(self):
        c, s = math.cos(self.theta), math.sin(self.theta)
        ia = self.id * c - self.iq * s
        ib_ = self.id * s + self.iq * c
        return ia, -0.5 * ia + math.sqrt(3) / 2 * ib_, -0.5 * ia - math.sqrt(3) / 2 * ib_


class Hardware:
    """Halls (with mounting errors), current sensing, battery."""

    def __init__(self, motor, vbat0=39.0, r_bat=0.25, hall_offset_deg=0.0, hall_errors_deg=(0, 0, 0, 0, 0, 0),
                 rotor_offset=202, phase_b_sign=1, offset8=79, offset10=317):
        self.m = motor
        self.vbat0, self.r_bat = vbat0, r_bat
        self.vbat = vbat0
        # the firmware assumes: true rotor angle at the state 4 edge = rotor_offset (in 1/256 turns)
        self.edge0 = rotor_offset / 256 * 2 * math.pi + math.radians(hall_offset_deg)
        self.edges = [self.edge0 + i * math.pi / 3 + math.radians(e) for i, e in enumerate(hall_errors_deg)]
        self.sign_b = phase_b_sign
        self.offset8, self.offset10 = offset8, offset10
        self.i_dc_f = 0.0
        self.i_b_f = 0.0
        self.broken_hall = False

    def hall(self):
        if self.broken_hall:
            return 7
        th = self.m.theta % (2 * math.pi)
        rel = [(th - e) % (2 * math.pi) for e in self.edges]
        k = min(range(6), key=lambda i: rel[i])      # the last edge passed
        return HALL_ORDER[k]

    def measure(self, duties, moe, dt):
        ia, ib, ic = self.m.phase_currents()
        i_dc = sum(d * i for d, i in zip(duties, (ia, ib, ic))) if moe else 0.0
        a = dt / 0.5e-3
        self.i_dc_f += (i_dc - self.i_dc_f) * min(1.0, a)
        self.i_b_f += (ib - self.i_b_f) * min(1.0, dt / 0.1e-3)
        self.vbat = self.vbat0 - self.r_bat * i_dc
        adc8 = max(0, min(255, int(round(self.offset8 + self.i_dc_f / 0.5))))
        adc10 = max(0, min(1023, int(round(self.offset10 + self.i_dc_f / 0.125))))
        adcb = max(0, min(255, int(round(126 + self.sign_b * self.i_b_f / 0.121))))
        adcv = max(0, min(255, int(self.vbat / 0.272)))
        return adc8, adc10, adcb, adcv, i_dc


def run(fw, motor, hw, seconds, target_fn, erps_max=340, log_every=0.01, brake_fn=None, event_fn=None):
    """target_fn(t) -> (target current A, throttle pressed)."""
    rows = []
    n = int(seconds / TS)
    duties = (0.5, 0.5, 0.5)
    adc8, adc10, adcb, adcv, i_dc = hw.offset8, hw.offset10, 126, int(hw.vbat0 / 0.272), 0.0
    next_log = 0.0
    t10 = t100 = 0.0
    peak_phase = 0.0
    fw.target_erps = erps_max
    for k in range(n):
        t = k * TS
        if event_fn:
            event_fn(t, fw, motor, hw)
        amps, pressed = target_fn(t)
        fw.throttle_released = not pressed
        fw.target_current = int(amps * 8)
        ccr, moe = fw.isr(hw.hall(), adc8, adcb, adcv)
        duties = tuple(c / PWM_PERIOD for c in ccr)
        if moe:
            mean = sum(duties) / 3
            va, vb, vc = ((d - mean) * hw.vbat for d in duties)
            v_alpha = (2 / 3) * (va - 0.5 * vb - 0.5 * vc)
            v_beta = (1 / math.sqrt(3)) * (vb - vc)
            motor.step(v_alpha, v_beta, TS)
        else:
            # switches open: currents decay through the diodes (back-EMF below the battery voltage)
            motor.id *= 0.5
            motor.iq *= 0.5
            motor._mech(TS)
            motor.theta = (motor.theta + motor.w * TS) % (2 * math.pi)
        adc8, adc10, adcb, adcv, i_dc = hw.measure(duties, moe, TS)
        iph = math.hypot(motor.id, motor.iq)
        peak_phase = max(peak_phase, iph)
        if t >= t10:
            t10 += 0.01
            fw.task_10ms(adc10)
        if t >= t100:
            t100 += 0.1
            fw.task_100ms(adc10, adcv)
        if t >= next_log:
            next_log += log_every
            rows.append(dict(t=t, iq=motor.iq, id=motor.id, iph=iph, peak=peak_phase, idc=i_dc, idc_f=hw.i_dc_f,
                             kmh=motor.v * 3.6, erps=fw.erps, w_erps=motor.w / 2 / math.pi, duty=fw.duty,
                             duty_max=fw.duty_max, corr=fw.correction, comm=fw.commutation, moe=moe,
                             state=fw.motor_state, ctrl=fw.ctrl_state, offs=list(fw.hall_off), vbat=hw.vbat))
            peak_phase = 0.0
    return rows


def stats(rows, key, t0=0.0, t1=1e9):
    v = [r[key] for r in rows if t0 <= r["t"] <= t1]
    return (min(v), sum(v) / len(v), max(v)) if v else (0, 0, 0)


def gamma_deg(r):
    """current angle from the q axis: 0 = all torque, positive = d current (field weakening direction)."""
    return math.degrees(math.atan2(-r["id"], r["iq"])) if abs(r["iq"]) + abs(r["id"]) > 0.3 else 0.0


# ------------------------------------------------------------------------------------------------ scenarios
def both(fn):
    out = {}
    for name, orig in (("ORIGINAL", True), ("CURRENT", False)):
        FINDINGS.clear()
        out[name] = (fn(orig), dict(FINDINGS))
    return out


def main():
    print("KT motor simulator (geared hub, 36 V nominal / 39 V battery, S06S 15 A)")

    # 0. phase B sensor sign: the FOC correction must move towards Id = 0 (the hardware works, so one sign does)
    print("\n=== 0. convention check: steady 20 km/h, which phase B sign makes the FOC correction converge")
    sign = None
    for sgn in (1, -1):
        FINDINGS.clear()
        m = Motor(v=20 / 3.6)
        fw = Firmware(original=False, rotor_offset=202)
        hw = Hardware(m, hall_offset_deg=15, phase_b_sign=sgn)     # 15 deg mounting error to correct
        rows = run(fw, m, hw, 2.5, lambda t: (8, True), erps_max=600)
        g = [gamma_deg(r) for r in rows if r["t"] > 2.0]
        mg = sum(g) / len(g)
        print(f"  sign {sgn:+d}: current angle from q {mg:6.1f} deg, correction {rows[-1]['corr']}, "
              f"iq {stats(rows, 'iq', 2.0)[1]:.1f} A")
        if abs(mg) < 12 and sign is None:
            sign = sgn
    if sign is None:
        print("  neither sign converges - model mapping wrong")
        return 1
    print(f"  using phase B sign {sign:+d}")

    def hw_for(m, **kw):
        return Hardware(m, phase_b_sign=sign, **kw)

    # 1. start from standstill, full throttle, flat road
    print("\n=== 1. start from standstill, full throttle (15 A), 100 kg, flat")

    def s1(orig):
        m = Motor(v=0.0)
        fw = Firmware(original=orig)
        return run(fw, m, hw_for(m), 8.0, lambda t: (15, True))
    for name, (rows, f) in both(s1).items():
        t25 = next((r["t"] for r in rows if r["kmh"] >= 20), None)
        print(f"  {name:8s}: peak phase current first 0.5 s {stats(rows, 'peak', 0, 0.5)[2]:5.1f} A, "
              f"battery {stats(rows, 'idc_f', 0.5, 3)[1]:5.1f} A avg, 20 km/h after "
              f"{t25 if t25 is None else round(t25, 2)} s, speed at 8 s {rows[-1]['kmh']:.1f} km/h  findings: {f or '-'}")

    # 2. locked rotor (wheel blocked, e.g. starting on a steep hill against the brake)
    print("\n=== 2. locked rotor, full throttle: phase current and the blocked motor protection")

    def s2(orig):
        m = Motor(locked=True)
        fw = Firmware(original=orig)
        return run(fw, m, hw_for(m), 3.0, lambda t: (15, True))
    for name, (rows, f) in both(s2).items():
        print(f"  {name:8s}: phase current {stats(rows, 'iph', 0.2, 1.8)[1]:5.1f} A avg / {stats(rows, 'peak', 0, 2)[2]:5.1f} A peak, "
              f"battery {stats(rows, 'idc_f', 0.2, 1.8)[1]:4.1f} A, duty {stats(rows, 'duty', 0.2, 1.8)[1]:.0f}/"
              f"{rows[0]['duty_max']}, PWM off at {next((round(r['t'], 1) for r in rows if not r['moe']), '-')} s  "
              f"findings: {f or '-'}")

    # 3. steady state with a hall mounting error: FOC correction, current angle, efficiency
    print("\n=== 3. 22 km/h steady, hall sensors mounted 20 deg off: does the FOC correction find the angle")

    def s3(orig):
        m = Motor(v=22 / 3.6)
        fw = Firmware(original=orig)
        return run(fw, m, hw_for(m, hall_offset_deg=20), 6.0, lambda t: (10, True), erps_max=600)
    for name, (rows, f) in both(s3).items():
        t_ok = next((r["t"] for r in rows if abs(gamma_deg(r)) < 8 and r["t"] > 0.2), None)
        g_end = sum(gamma_deg(r) for r in rows[-50:]) / 50
        print(f"  {name:8s}: current angle within 8 deg after {t_ok if t_ok is None else round(t_ok, 2)} s, "
              f"at the end {g_end:5.1f} deg, iq {stats(rows, 'iq', 5)[1]:.1f} A, id {stats(rows, 'id', 5)[1]:.1f} A  "
              f"findings: {f or '-'}")

    # 3b. FOC sample delay compensation. The phase B current is measured with a delay (here: 1 PWM period ADC +
    # 0.1 ms filter = 164 us). Compensating by +164 us (the textbook direction) made the angle worse in this model,
    # -40 us made it flatter over the speed: the sign and size depend on the real sensor, so it is a setting (off).
    print("\n=== 3b. current angle at 3 speeds, hall mounting error 10 deg, with delay compensation 0 / +164 / -40 us")
    for delay in (0, 164, -40):
        line = []
        for kmh in (10, 18, 26):
            FINDINGS.clear()
            m = Motor(v=kmh / 3.6, mass=5000)
            fw = Firmware(foc_delay_us=delay)
            rows = run(fw, m, hw_for(m, hall_offset_deg=10), 4.0, lambda t: (8, True), erps_max=600)
            g = sum(gamma_deg(r) for r in rows[-80:]) / 80
            line.append(f"{kmh} km/h {g:6.1f} deg")
        print(f"  delay {delay:3d} us: " + ", ".join(line))

    # 4. misplaced hall sensors: torque ripple, learning
    print("\n=== 4. hall sensors not 60 deg apart (+8, -6, +5, -7, +4, -4 deg), 20 km/h, 10 A")
    errs = (8, -6, 5, -7, 4, -4)

    def s4(orig):
        m = Motor(v=20 / 3.6)
        fw = Firmware(original=orig)
        return run(fw, m, hw_for(m, hall_errors_deg=errs), 8.0, lambda t: (10, True), erps_max=600, log_every=0.0005)
    for name, (rows, f) in both(s4).items():
        def ripple(t0, t1):
            v = [r["iq"] for r in rows if t0 <= r["t"] <= t1]
            avg = sum(v) / len(v)
            return math.sqrt(sum((x - avg) ** 2 for x in v) / len(v))
        print(f"  {name:8s}: iq ripple (rms) first second {ripple(0.5, 1.5):.2f} A, last second {ripple(7, 8):.2f} A, "
              f"learned offsets {[rows[-1]['offs'][s] for s in HALL_ORDER]}  findings: {f or '-'}")

    # 5. current step at mid speed: controller response
    print("\n=== 5. current step 3 A -> 12 A at 18 km/h (bike speed held), response of the current control")

    def s5(orig):
        m = Motor(v=18 / 3.6, mass=5000)            # heavy: speed stays nearly constant
        fw = Firmware(original=orig)
        return run(fw, m, hw_for(m), 4.0, lambda t: (3 if t < 2 else 12, True), erps_max=600, log_every=0.005)
    for name, (rows, f) in both(s5).items():
        after = [r for r in rows if r["t"] >= 2]
        t90 = next((round(r["t"] - 2, 2) for r in after if r["idc_f"] >= 0.9 * 12), None)
        over = max(r["idc_f"] for r in after)
        tail = stats(rows, "idc_f", 3.5, 4)
        print(f"  {name:8s}: 90 % after {t90} s, max {over:.1f} A, last 0.5 s {tail[0]:.1f} .. {tail[2]:.1f} A  "
              f"findings: {f or '-'}")

    # 6. speed limit
    print("\n=== 6. speed limit 25 km/h (340 erps), full throttle from 15 km/h, downhill 1 %")

    def s6(orig):
        m = Motor(v=15 / 3.6, slope=-0.01)
        fw = Firmware(original=orig)
        return run(fw, m, hw_for(m), 14.0, lambda t: (15, True), erps_max=340)
    for name, (rows, f) in both(s6).items():
        sp = stats(rows, "kmh", 8, 14)
        print(f"  {name:8s}: speed 8 ... 14 s {sp[0]:.1f} .. {sp[2]:.1f} km/h (limit 25.0), max {stats(rows, 'kmh')[2]:.1f}, "
              f"battery {stats(rows, 'idc_f', 8, 14)[1]:.1f} A  findings: {f or '-'}")

    # 7. throttle released at speed: regen spike, freewheel
    print("\n=== 7. 25 km/h, throttle released: current when the duty cycle ramps down (geared, freewheel)")

    def s7(orig):
        m = Motor(v=24 / 3.6)
        fw = Firmware(original=orig)
        return run(fw, m, hw_for(m), 3.0, lambda t: (12 if t < 1 else 0, t < 1), erps_max=600, log_every=0.002)
    for name, (rows, f) in both(s7).items():
        after = [r for r in rows if r["t"] > 1]
        print(f"  {name:8s}: battery current after release min {min(r['idc'] for r in after):.1f} A, "
              f"phase iq min {min(r['iq'] for r in after):.1f} A, duty at 3 s {rows[-1]['duty']}  findings: {f or '-'}")

    # 8. direct drive hub, regen braking with 6 A
    print("\n=== 8. direct drive, 25 km/h, brake lever with 6 A regen")

    def s8(orig):
        m = Motor(v=25 / 3.6, direct_drive=True)
        fw = Firmware(original=orig, regen_a=6, direct_drive=True)

        def ev(t, fw_, m_, hw_):
            if t >= 2.0 and not fw_.brake:
                fw_.brake = True
                fw_.press_brake(True)
        # ride with a little current first, throttle off at 1 s, brake at 2 s
        return run(fw, m, hw_for(m), 4.5, lambda t: (3, True) if t < 1.0 else (0, False), erps_max=600,
                   event_fn=ev, log_every=0.005)
    for name, (rows, f) in both(s8).items():
        tail = stats(rows, "idc", 2.5, 4.5)
        print(f"  {name:8s}: battery current while braking {tail[0]:.1f} .. {tail[2]:.1f} A, avg {tail[1]:.1f} (limit -6), "
              f"speed 25 -> {rows[-1]['kmh']:.1f} km/h  findings: {f or '-'}")

    print("\n=== 8b. direct drive, 20 km/h, throttle held (12 A) AND brake pulled at 0.5 s")

    def s8b(orig):
        m = Motor(v=20 / 3.6, direct_drive=True)
        fw = Firmware(original=orig, regen_a=6, direct_drive=True)

        def ev(t, fw_, m_, hw_):
            if t >= 0.5 and not fw_.brake:
                fw_.brake = True
                fw_.press_brake(True)
        return run(fw, m, hw_for(m), 3.0, lambda t: (12, True), erps_max=600, event_fn=ev, log_every=0.005)
    for name, (rows, f) in both(s8b).items():
        tail = stats(rows, "idc", 1.5, 3.0)
        print(f"  {name:8s}: battery current with brake + throttle {tail[0]:.1f} .. {tail[2]:.1f} A "
              f"(should be <= 0: brake wins), speed at 3 s {rows[-1]['kmh']:.1f} km/h  findings: {f or '-'}")

    print("\n=== 8d. direct drive, power on while rolling at 25 km/h, no throttle: does the motor brake?")

    def s8d(orig):
        m = Motor(v=25 / 3.6, direct_drive=True)
        fw = Firmware(original=orig, direct_drive=True)
        return run(fw, m, hw_for(m), 3.0, lambda t: (0, False), erps_max=600, log_every=0.005)
    for name, (rows, f) in both(s8d).items():
        tail = [r for r in rows if r["t"] > 2]
        print(f"  {name:8s}: phase current after 2 s {sum(abs(r['iq']) for r in tail) / len(tail):.1f} A avg "
              f"(braking torque), speed 25 -> {rows[-1]['kmh']:.1f} km/h, duty {rows[-1]['duty']}  findings: {f or '-'}")

    print("\n=== 8c. direct drive, 20 km/h, 8 A, brake from 1.0 to 1.5 s (no regen set), then 8 A again")

    def s8c(orig):
        m = Motor(v=20 / 3.6, direct_drive=True)
        fw = Firmware(original=orig, regen_a=0, direct_drive=True)

        def ev(t, fw_, m_, hw_):
            want = 1.0 <= t < 1.5
            if want != fw_.brake:
                fw_.brake = want
                fw_.press_brake(want)
        return run(fw, m, hw_for(m), 2.5, lambda t: (8, True), erps_max=600, event_fn=ev, log_every=0.002)
    for name, (rows, f) in both(s8c).items():
        br = [r for r in rows if 1.05 <= r["t"] < 1.5]
        after = [r for r in rows if 1.5 <= r["t"] < 1.8]
        print(f"  {name:8s}: while braking max battery {max(r['idc'] for r in br):.1f} A, after release phase current "
              f"{min(r['iq'] for r in after):.1f} .. {max(r['iq'] for r in after):.1f} A  findings: {f or '-'}")

    # 9. hall wire breaks while riding
    print("\n=== 9. hall wire breaks at 20 km/h for 1 s (all halls read 1 = state 7), throttle released at 2.5 s")

    def s9(orig):
        m = Motor(v=20 / 3.6)
        fw = Firmware(original=orig)

        def ev(t, fw_, m_, hw_):
            hw_.broken_hall = 1.0 <= t < 2.0
        return run(fw, m, hw_for(m), 4.0, lambda t: (10, True) if (t < 2.5 or t > 3.0) else (0, False),
                   erps_max=600, event_fn=ev, log_every=0.002)
    for name, (rows, f) in both(s9).items():
        during = [r for r in rows if 1.0 < r["t"] < 2.0]
        after = [r for r in rows if r["t"] > 3.5]
        print(f"  {name:8s}: phase current while broken max {max(r['iph'] for r in during):.1f} A, "
              f"PWM off after {next((round(r['t'] - 1, 3) for r in during if not r['moe']), '-')} s, "
              f"running again at the end: {'yes' if after[-1]['moe'] and after[-1]['iq'] > 3 else 'no'} "
              f"(iq {after[-1]['iq']:.1f} A)  findings: {f or '-'}")

    # 11. coasting: idle current of a geared motor with no current requested
    print("\n=== 11. geared, 20 km/h, throttle released for 3 s, then 10 A again")

    def s11(orig):
        m = Motor(v=20 / 3.6)
        fw = Firmware(original=orig)
        return run(fw, m, hw_for(m), 5.0, lambda t: (10, True) if (t < 1 or t > 4) else (0, False),
                   erps_max=600, log_every=0.002)
    for name, (rows, f) in both(s11).items():
        idle = stats(rows, "idc", 2.0, 4.0)
        re = [r for r in rows if r["t"] > 4]
        print(f"  {name:8s}: battery current while coasting {idle[1]:.2f} A avg, motor at 4 s "
              f"{[r['w_erps'] for r in rows if abs(r['t'] - 4) < 0.002][0]:.0f} erps, after re-apply max phase "
              f"{max(r['iph'] for r in re):.1f} A, min battery {min(r['idc'] for r in re):.1f} A  findings: {f or '-'}")

    # 10. high current S12S: int8 wrap of the original
    print("\n=== 10. S12S 25 A start: the original int8 current wraps above 15.9 A")

    def s10(orig):
        m = Motor(v=0.0, R=0.15)
        fw = Firmware(original=orig, current_a=25)
        return run(fw, m, hw_for(m), 4.0, lambda t: (25, True))
    for name, (rows, f) in both(s10).items():
        print(f"  {name:8s}: battery current {stats(rows, 'idc_f', 1, 4)[1]:.1f} A avg, max {stats(rows, 'idc_f', 0, 4)[2]:.1f} A "
              f"(limit 25)  findings: {f or '-'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
