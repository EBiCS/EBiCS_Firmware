"""
EBiCS motor control simulator (host side, no hardware needed).

Runs a bit-exact Python port of the firmware's fast loop against a PMSM motor model:
  - FOC_calculation(), PI_control(), runPIcontrol() incl. battery current limit, svpwm()   (Src/FOC.c, Src/main.c)
  - CMSIS q31 clarke / park / inverse park / sin_cos / sqrt with the library's saturation behaviour
  - hall angle interpolation and 6-step fallback of the injected-ADC interrupt               (Src/main.c)
  - sensorless observer, atan2, open loop start, stall detection                           (Src/sensorless.c)
Every C int32 / int64 operation is checked: an overflow is recorded as a finding instead of silently wrapping.
The motor sees the real PWM duty cycles (uint16 switch times!) and is sampled by a 12 bit ADC model.

Usage:  python tools/motor_sim.py            (runs all scenarios, prints a report)
Part of EBiCS, GPL v3 or later.
"""

import math
import sys
from collections import Counter

# ----------------------------------------------------------------------------------------------- firmware constants
T_PWM = 2028                 # _T
U_MAX = 2000                 # _U_MAX
SQRT3_16 = 28                # _SQRT3
F_FOC = 64e6 / (2 * T_PWM)   # one FOC run per PWM period (center aligned, CC4 when counting down)
TS = 1.0 / F_FOC
CAL_I = 38                   # mA per ADC count (CAL_I = 38LL<<8)
CAL_V = 25                   # mV per ADC count of the battery voltage
ADC_OFFSET = 2048
P_IQ, I_IQ, P_ID, I_ID = 500, 20, 500, 20
SIXSTEPTHRESHOLD = 3500
DEG_30 = 357913941

FINDINGS = Counter()
FIXED_FIRMWARE = True     # False = behaviour before the fixes (for comparison)


def finding(tag):
    FINDINGS[tag] += 1


I32_MIN, I32_MAX = -2 ** 31, 2 ** 31 - 1
I64_MIN, I64_MAX = -2 ** 63, 2 ** 63 - 1


def w32(x, tag):
    """C int32 result: record overflow, wrap like the CPU does."""
    if x < I32_MIN or x > I32_MAX:
        finding("int32 overflow: " + tag)
        x = ((x + 2 ** 31) % 2 ** 32) - 2 ** 31
    return x


def w64(x, tag):
    if x < I64_MIN or x > I64_MAX:
        finding("int64 overflow: " + tag)
        x = ((x + 2 ** 63) % 2 ** 64) - 2 ** 63
    return x


def sat32(x):
    return max(I32_MIN, min(I32_MAX, x))


def asr(x, n):
    return x >> n          # Python >> on ints is arithmetic, like ARM GCC for signed values


def cdiv(a, b):
    """C integer division (truncates toward zero); Cortex-M3 returns 0 for /0 (no trap)."""
    if b == 0:
        finding("division by zero")
        return 0
    q = abs(a) // abs(b)
    return q if (a >= 0) == (b >= 0) else -q


# ----------------------------------------------------------------------------------------------- CMSIS q31 helpers
def clarke_q31(ia, ib):
    p1 = (ia * 0x24F34E8B) >> 30
    p2 = (ib * 0x49E69D16) >> 30
    return ia, sat32(p1 + p2)


def park_q31(a, b, s, c):
    p1 = (a * c) >> 31; p2 = (b * s) >> 31; p3 = (a * s) >> 31; p4 = (b * c) >> 31
    return sat32(p1 + p2), sat32(p4 - p3)


def inv_park_q31(d, q, s, c):
    p1 = (d * c) >> 31; p2 = (q * s) >> 31; p3 = (d * s) >> 31; p4 = (q * c) >> 31
    return sat32(p1 - p2), sat32(p4 + p3)


def sin_cos_q31(theta):
    a = math.pi * theta / 2 ** 31
    return sat32(round(math.sin(a) * 2 ** 31)), sat32(round(math.cos(a) * 2 ** 31))


def sqrt_q31(x):
    if x <= 0:
        return 0
    return sat32(int(math.sqrt(x / 2 ** 31) * 2 ** 31))


# ----------------------------------------------------------------------------------------------- firmware state
class PI:
    def __init__(self, gp, gi, limit_out, limit_i, shift=10, max_step=5000):
        self.gp, self.gi, self.lo, self.li, self.shift, self.max_step = gp, gi, limit_out, limit_i, shift, max_step
        self.setpoint = self.recent = self.integral = self.out = 0


def PI_control(pi, moe=True):
    err = w32(pi.setpoint - pi.recent, "PI error")
    p = w32(err * pi.gp, "PI proportional (error*gain_p)")
    pi.integral = w32(pi.integral + w32(err * pi.gi, "PI error*gain_i"), "PI integral")
    lim = w32(pi.li << pi.shift, "PI limit_i<<shift")
    pi.integral = max(-lim, min(lim, pi.integral))
    if not moe:
        pi.integral = 0
    tot = w32(p + pi.integral, "PI p+i")
    if tot > pi.out + pi.max_step:
        pi.out += pi.max_step
    elif tot < pi.out - pi.max_step:
        pi.out -= pi.max_step
    else:
        pi.out = tot
    lo = pi.lo << pi.shift
    pi.out = max(-lo, min(lo, pi.out))
    if not moe:
        pi.out = 0
    return asr(pi.out, pi.shift)


class Firmware:
    def __init__(self, ph_current_max, battery_current_max_ma=1_000_000, regen_max_ma=0, vbat_raw=1440,
                 sensorless=False, obs=None):
        self.ph_current_max = ph_current_max
        self.bc_max = battery_current_max_ma
        self.regen_max = regen_max_ma
        self.vbat_raw = vbat_raw
        self.pi_iq = PI(P_IQ, I_IQ, U_MAX, U_MAX)
        self.pi_id = PI(P_ID, I_ID, U_MAX, 1800)
        self.iq_fil = self.id_fil = 0
        self.i_q = self.i_d = 0
        self.u_d = self.u_q = self.u_abs = 0
        self.i_q_setpoint = 0
        self.i_d_setpoint = 0
        self.bat_acc = 0
        self.battery_current = 0
        self.bc_flag = 0
        self.brake = 0
        self.switchtime = [1014, 1014, 1014]
        self.st_raw = [1014, 1014, 1014]
        self.sensorless = sensorless
        self.obs = Sensorless(obs) if sensorless else None
        self.dyn_state = 1
        self.uq_fil = 0
        self.moe = True

    # --- runPIcontrol() of main.c
    def runPIcontrol(self):
        """Src/main.c runPIcontrol() after the fix (FIXED_FIRMWARE) or as before (legacy)."""
        if not FIXED_FIRMWARE:
            return self.runPIcontrol_legacy()
        power = w32(w32(self.u_q * self.i_q, "u_q*i_q") - w32(self.u_d * self.i_d, "u_d*i_d"), "power")
        self.bat_acc = w32(self.bat_acc - asr(self.bat_acc, 8), "battery current accumulator")
        self.bat_acc = w32(self.bat_acc + asr(w32(power * 7, "power*7"), 14) * CAL_I, "battery current accumulator")
        self.battery_current = asr(self.bat_acc, 8)
        sp = self.i_q_setpoint
        if self.sensorless:
            sp = self.obs.q_setpoint(sp)
        self.uq_fil = w32(self.uq_fil - asr(self.uq_fil, 8) + abs(self.u_q), "u_q filter")
        uq = asr(self.uq_fil, 8)
        self.bc_flag = 0
        if uq > 20:
            lim_ma = max(0, min(131000, self.regen_max if self.brake else self.bc_max))
            lim = cdiv(w32(lim_ma * 16384, "battery limit*16384"), 7 * CAL_I * uq)
            if sp > lim:
                sp, self.bc_flag = lim, 1
            if sp < -lim:
                sp, self.bc_flag = -lim, 1
        self.pi_iq.recent = self.i_q
        self.pi_iq.setpoint = sp
        self._pi_and_limit()

    def _pi_and_limit(self):
        uq = PI_control(self.pi_iq)
        self.pi_id.recent = self.i_d
        self.pi_id.setpoint = self.i_d_setpoint
        ud = -PI_control(self.pi_id)
        s = w32(w32(ud * ud, "u_d^2") + w32(uq * uq, "u_q^2"), "u_d^2+u_q^2")
        s = w32(s << 1, "(u_d^2+u_q^2)<<1")
        self.u_abs = asr(sqrt_q31(s), 16) + 1
        if self.u_abs > U_MAX:
            self.u_q = cdiv(w32(uq * U_MAX, "u_q*_U_MAX"), self.u_abs)
            self.u_d = cdiv(w32(ud * U_MAX, "u_d*_U_MAX"), self.u_abs)
            self.u_abs = U_MAX
        else:
            self.u_q, self.u_d = uq, ud

    def runPIcontrol_legacy(self):
        """runPIcontrol() as it was before the simulation found its problems."""
        self.bat_acc = w32(self.bat_acc - asr(self.bat_acc, 8), "battery current accumulator")
        self.bat_acc = w32(self.bat_acc + w32(asr(w32(self.i_q * self.u_abs, "i_q*u_abs"), 11) * CAL_I, "battery current"),
                           "battery current accumulator")
        self.battery_current = asr(self.bat_acc, 8)
        if self.battery_current > self.bc_max:
            self.bc_flag = 1
        if self.battery_current < -self.regen_max:
            self.bc_flag = 1
        sp_power = w32(asr(w32(self.i_q_setpoint * self.u_abs, "i_q_setpoint*u_abs"), 11) * CAL_I, "setpoint power")
        if not self.brake:
            if sp_power < (self.bc_max * 7) >> 3:
                self.bc_flag = 0
        else:
            if sp_power > (-self.regen_max * 7) >> 3:
                self.bc_flag = 0
        if not self.bc_flag:
            self.pi_iq.recent = self.i_q
            self.pi_iq.setpoint = self.i_q_setpoint
        else:
            self.pi_iq.recent = asr(self.battery_current, 6)
            self.pi_iq.setpoint = asr(self.bc_max, 6) if not self.brake else asr(-self.regen_max, 6)
        self._pi_and_limit()

    # --- FOC_calculation() of FOC.c
    def foc(self, ia, ib, theta):
        alpha, beta = clarke_q31(ia, ib)
        s, c = sin_cos_q31(theta)
        if s == -2 ** 31:
            s = -2 ** 31 + 1
        d, q = park_q31(alpha, beta, s, c)
        self.iq_fil = w32(self.iq_fil - asr(self.iq_fil, 4) + q, "i_q filter")
        self.i_q = asr(self.iq_fil, 4)
        self.id_fil = w32(self.id_fil - asr(self.id_fil, 4) + d, "i_d filter")
        self.i_d = asr(self.id_fil, 4)
        self.runPIcontrol()
        if self.sensorless:
            self.obs.startup(self, self.i_q_setpoint)
        ua, ub = inv_park_q31(self.u_d, self.u_q, -s, c)
        if self.sensorless:
            self.obs.observe(ua, ub, alpha, beta, self.vbat_raw)
        self.svpwm(ua, ub)

    def svpwm(self, ua, ub):
        U_alpha = asr(w32(w32(SQRT3_16 * T_PWM, "_SQRT3*_T") * ua, "svpwm _SQRT3*_T*u_alpha"), 4)
        U_beta = w32(-T_PWM * ub, "svpwm -_T*u_beta")
        X = U_beta
        Y = asr(w32(U_alpha + U_beta, "svpwm Y"), 1)
        Z = asr(w32(U_beta - U_alpha, "svpwm Z"), 1)
        st = list(self.st_raw)
        if (Y >= 0 and Z < 0 and X > 0) or (Y < 0 and Z >= 0 and X <= 0):
            st[0] = asr(T_PWM + X - Z, 12) + (T_PWM >> 1); st[1] = st[0] + asr(Z, 11); st[2] = st[1] - asr(X, 11)
        if (Y >= 0 and Z >= 0) or (Y < 0 and Z < 0):
            st[0] = asr(T_PWM + Y - Z, 12) + (T_PWM >> 1); st[1] = st[0] + asr(Z, 11); st[2] = st[0] - asr(Y, 11)
        if (Y < 0 and Z >= 0 and X > 0) or (Y >= 0 and Z < 0 and X <= 0):
            st[0] = asr(T_PWM + Y - X, 12) + (T_PWM >> 1); st[2] = st[0] - asr(Y, 11); st[1] = st[2] + asr(X, 11)
        self.st_raw = st
        for k in range(3):
            if st[k] < 0 or st[k] > T_PWM:
                finding("switchtime outside 0.._T (uint16 wraps -> wrong duty)")
        self.switchtime = [v & 0xFFFF for v in st]    # uint16_t switchtime[3]

    def dyn_adc_state(self):
        s0, s1, s2 = self.switchtime
        if s2 > s0 and s2 > s1:
            self.dyn_state = 1
        if s0 > s1 and s0 > s2:
            self.dyn_state = 2
        if s1 > s0 and s1 > s2:
            self.dyn_state = 3


# ----------------------------------------------------------------------------------------------- sensorless.c port
LUT_ATAN = [round(math.atan(i / 100) / (math.pi / 4) * 16384) for i in range(101)]
LUT_ATAN[100] = 16383


def lut_ratio(num, den):
    r = (num * 1000) // den
    idx, frac = r // 10, r % 10
    if idx >= 100:
        return LUT_ATAN[100]
    return LUT_ATAN[idx] + ((LUT_ATAN[idx + 1] - LUT_ATAN[idx]) * frac) // 10


def atan2_q31(y, x):
    ax, ay = abs(x), abs(y)
    if not ax and not ay:
        return 0
    while ax >= (1 << 21) or ay >= (1 << 21):
        ax >>= 1; ay >>= 1
    if not ax and not ay:
        return 0
    ang = lut_ratio(ay, ax) if ay <= ax else 32768 - lut_ratio(ax, ay)
    if x < 0:
        ang = 65536 - ang
    q = (ang << 15) & 0xFFFFFFFF
    q = q - 2 ** 32 if q >= 2 ** 31 else q
    r = -q if y < 0 else q
    return ((r + 2 ** 31) % 2 ** 32) - 2 ** 31


def change_frame(d, q, frm, to):
    """vector in the firmware dq frame at angle frm -> same stator vector in the frame at angle to (sensorless.c)"""
    s1, c1 = sin_cos_q31(frm)
    s2, c2 = sin_cos_q31(to)
    a = ((d * c1) >> 31) + ((q * s1) >> 31)
    b = ((q * c1) >> 31) - ((d * s1) >> 31)
    return ((a * c2) >> 31) - ((b * s2) >> 31), ((a * s2) >> 31) + ((b * c2) >> 31)


class Sensorless:
    STARTUP_STEP, STARTUP_CYCLES, STARTUP_UD_MAX, STALL = 2684354, 4000, 250, 10000

    def __init__(self, p):
        self.L, self.R, self.lam, self.gamma, self.offset = p["L"], p["R"], p["lam"], p["gamma"], p["offset"]
        self.obs_cal_v = 15 << 8
        self.dT = p.get("dT", 13)
        self.x1 = self.x2 = 0
        self.teta = 0
        self.running = 0
        self.startup_counter = 0
        self.erps_counter = self.STALL
        self.erps_cum = self.STALL << 4
        self.erps_filtered = self.STALL
        self.angle_old = 0
        self.sin_delay, self.cos_delay = 0, 2 ** 31 - 1
        self.slow_div = 0
        self.passive = p.get("passive", False)   # test mode: observer runs alongside, FOC uses the true angle
        self.exact_dt = p.get("exact_dt", FIXED_FIRMWARE)  # firmware: x += x_dot * 17012 >> 28 (one FOC period)
        self.obs_angle = 0
        self.handover = self.HANDOVER
        self.d0 = self.q0 = 0
        if FIXED_FIRMWARE:
            self.erps_counter = 0

    HANDOVER, RAMP = 1000, 1000

    def startup(self, fw, target):
        """Src/sensorless.c sensorless_startup(): I/f open loop + bumpless hand-over (fixed) or voltage ramp (legacy)."""
        if self.passive:
            return
        if not FIXED_FIRMWARE:
            if self.running:
                return
            if target > 20 or target < -20:
                fw.u_d = min(self.startup_counter >> 4, 250)
                step = self.STARTUP_STEP if target > 0 else -self.STARTUP_STEP
                self.teta = ((self.teta + step + 2 ** 31) % 2 ** 32) - 2 ** 31
                self.startup_counter += 1
                if self.startup_counter > self.STARTUP_CYCLES:
                    self.running, self.startup_counter = 1, 0
            return
        if self.running:
            if self.handover < self.HANDOVER:
                self.handover += 1
                fw.i_d_setpoint = cdiv(self.d0 * (self.HANDOVER - self.handover), self.HANDOVER)
            else:
                fw.i_d_setpoint = 0
            return
        if target > 20 or target < -20:
            self.i_start = min(abs(target), fw.ph_current_max)
            fw.i_d_setpoint = cdiv(self.i_start * self.startup_counter, self.RAMP) if self.startup_counter < self.RAMP else self.i_start
            step = self.STARTUP_STEP if target > 0 else -self.STARTUP_STEP
            self.teta = ((self.teta + step + 2 ** 31) % 2 ** 32) - 2 ** 31
            self.startup_counter += 1
            if self.startup_counter > self.STARTUP_CYCLES:
                old, new = self.teta, self.obs_angle
                d, q = change_frame(fw.u_d, fw.u_q, old, new)
                fw.pi_id.integral = fw.pi_id.out = -(d << fw.pi_id.shift)
                fw.pi_iq.integral = fw.pi_iq.out = q << fw.pi_iq.shift
                fw.id_fil, fw.iq_fil = change_frame(fw.id_fil, fw.iq_fil, old, new)
                self.d0, self.q0 = change_frame(fw.i_d_setpoint, 0, old, new)
                self.handover = 0
                self.teta = new
                self.running, self.startup_counter = 1, 0

    def q_setpoint(self, sp):
        if self.passive:
            return sp
        if not self.running:
            return 0
        if self.handover < self.HANDOVER:
            return cdiv(self.q0 * (self.HANDOVER - self.handover) + sp * self.handover, self.HANDOVER)
        return sp

    def observer(self, va, vb, ia, ib):
        L = (3 * self.L) >> 1
        R = (3 * self.R) >> 1
        lam2 = self.lam * self.lam
        L_ia = w64(L * ia, "obs L*i_alpha") >> 16
        L_ib = w64(L * ib, "obs L*i_beta") >> 16
        R_ia = w64(R * ia, "obs R*i_alpha") >> 9
        R_ib = w64(R * ib, "obs R*i_beta") >> 9
        ea = self.x1 - L_ia
        eb = self.x2 - L_ib
        err = w64(lam2 - w64(w64(ea * ea, "obs e_alpha^2") + w64(eb * eb, "obs e_beta^2"), "obs e^2 sum"), "obs err")
        err = max(-lam2, min(lam2, err))
        x1d = w64(-R_ia + va + (w64(ea * err, "obs e_alpha*err") >> self.gamma), "obs x1_dot")
        x2d = w64(-R_ib + vb + (w64(eb * err, "obs e_beta*err") >> self.gamma), "obs x2_dot")
        if self.exact_dt:
            k = round(2 ** 28 / F_FOC)
            self.x1 = w64(self.x1 + (w64(x1d * k, "obs x1_dot*k") >> 28), "obs x1")
            self.x2 = w64(self.x2 + (w64(x2d * k, "obs x2_dot*k") >> 28), "obs x2")
        else:
            self.x1 = w64(self.x1 + (x1d >> self.dT), "obs x1")
            self.x2 = w64(self.x2 + (x2d >> self.dT), "obs x2")
        return self.x1 - L_ia, self.x2 - L_ib

    def observe(self, ua, ub, ialpha, ibeta, vbat_raw):
        ia_c, ib_c = park_q31(ialpha, ibeta, self.sin_delay, self.cos_delay)
        va = w64(ua * vbat_raw * self.obs_cal_v, "obs v_alpha") >> 11
        vb = w64(-ub * vbat_raw * self.obs_cal_v, "obs v_beta") >> 11
        ea, eb = self.observer(va, vb, -ia_c * (CAL_I << 8), -ib_c * (CAL_I << 8))
        self.obs_angle = ((atan2_q31(ea, -eb) + self.offset + 2 ** 31) % 2 ** 32) - 2 ** 31
        if self.running or self.passive:
            self.teta = self.obs_angle
        if self.erps_counter < self.STALL:
            self.erps_counter += 1
        else:
            self.running = 0
            self.erps_filtered = self.STALL
        wrapped = (self.angle_old > (1 << 25) and self.teta < -(1 << 25)) or (self.angle_old < -(1 << 25) and self.teta > (1 << 25))
        if wrapped and self.erps_counter > 15:
            self.erps_cum = self.erps_cum - (self.erps_cum >> 4) + self.erps_counter
            self.erps_filtered = self.erps_cum >> 4
            self.erps_counter = 0
        self.angle_old = self.teta
        self.slow_div += 1
        if self.slow_div >= 1000:                       # sensorless_slow() from the 16 Hz loop
            self.slow_div = 0
            d = cdiv(59652323 << 4, self.erps_filtered + 1)
            self.sin_delay, self.cos_delay = sin_cos_q31(d)


# ----------------------------------------------------------------------------------------------- motor model
class Motor:
    """Surface PMSM in the rotor dq frame; electrical angle theta, electrical speed w (rad/s)."""

    def __init__(self, R=0.15, L=0.25e-3, lam=0.010, J=None, load=0.0, w=0.0, theta=0.0):
        self.R, self.L, self.lam, self.J, self.load = R, L, lam, J, load
        self.id = self.iq = 0.0
        self.w, self.theta = w, theta

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
            if self.J:
                torque_e = 1.5 * self.lam * self.iq           # per pole pair, electrical units
                self.w += (torque_e - self.load * (1 if self.w > 0 else -1 if self.w < 0 else 0)) / self.J * h
            self.theta = (self.theta + self.w * h) % (2 * math.pi)

    def phase_currents(self):
        c, s = math.cos(self.theta), math.sin(self.theta)
        ia = self.id * c - self.iq * s
        ib_ = self.id * s + self.iq * c
        return ia, -0.5 * ia + math.sqrt(3) / 2 * ib_, -0.5 * ia - math.sqrt(3) / 2 * ib_

    def emf_phase_peak(self):
        return abs(self.w) * self.lam


def adc_counts(i, sign):
    raw = ADC_OFFSET + round(sign * i * 1000 / CAL_I)
    if raw < 0 or raw > 4095:
        finding("phase current ADC saturated (12 bit)")
    return max(0, min(4095, raw)) - ADC_OFFSET


def leg_voltages(switchtime, vbat):
    """Center aligned PWM mode 1: output high while CNT < CCR. CCR > ARR = always high."""
    return [vbat * min(1.0, st / T_PWM) for st in switchtime]


# ----------------------------------------------------------------------------------------------- hall emulation
class Halls:
    """Hall sector events and the firmware's angle interpolation / 6-step fallback (ADC interrupt in main.c)."""

    def __init__(self, offset_q31):
        self.offset = offset_q31       # firmware angle = rotor angle + offset (found by autodetect)
        self.sector = None
        self.t_since = 0.0
        self.timertics = 20000
        self.overflow = 1
        self.hall_angle = 0
        self.direction = 1

    def update(self, theta, dt):
        sector = int(((theta % (2 * math.pi)) / (math.pi / 3))) % 6
        self.t_since += dt
        if self.sector is None:
            self.sector = sector
            self.hall_angle = self._q31(sector * math.pi / 3 + (math.pi / 6))
        elif sector != self.sector:
            fwd = (sector - self.sector) % 6 == 1
            self.direction = 1 if fwd else -1
            tics = int(self.t_since * 64e6 / 129)
            if tics > 20:
                self.timertics = min(tics, 65535)
            self.t_since = 0.0
            edge = (sector if fwd else self.sector) * math.pi / 3
            self.hall_angle = self._q31(edge)
            self.sector = sector
            self.overflow = 0

    def _q31(self, a):
        q = int(a / math.pi * 2 ** 31)
        return ((q + self.offset + 2 ** 31) % 2 ** 32) - 2 ** 31

    def angle(self, iq_setpoint):
        recent = min(int(self.t_since * 64e6 / 129), 65535)
        sixstep = self.timertics > (SIXSTEPTHRESHOLD * 6) >> 2
        if recent < self.timertics + (self.timertics >> 2) and not self.overflow and not sixstep:
            inc = (cdiv(10923 * recent, self.timertics) * self.direction) << 16
            return ((self.hall_angle + inc + 2 ** 31) % 2 ** 32) - 2 ** 31
        self.overflow = 1
        sgn = 1 if iq_setpoint >= 0 else -1
        return ((self.hall_angle + sgn * DEG_30 * self.direction + 2 ** 31) % 2 ** 32) - 2 ** 31


# ----------------------------------------------------------------------------------------------- simulation loop
def ang_err_deg(a_q31, b_rad):
    a = a_q31 / 2 ** 31 * 180.0
    b = math.degrees(b_rad)
    return (a - b + 180) % 360 - 180


def run(motor, fw, seconds, setpoint_fn, vbat=36.0, current_sign=-1, angle_mode="ideal", angle_offset=math.pi,
        halls=None, log_every=0.01, brake_fn=None, coast=False):
    """angle_mode: 'ideal' (perfect rotor angle + offset), 'halls', 'sensorless'."""
    n = int(seconds / TS)
    rows = []
    next_log = 0.0
    off_q31 = int(angle_offset / math.pi * 2 ** 31)
    zero_since = 0.0
    for k in range(n):
        t = k * TS
        fw.i_q_setpoint = setpoint_fn(t)
        if brake_fn:
            fw.brake = brake_fn(t)
        ia, ib, ic = motor.phase_currents()
        fw.dyn_adc_state()
        ma, mb = adc_counts(ia, current_sign), adc_counts(ib, current_sign)
        if angle_mode == "ideal":
            theta = ((int(motor.theta / math.pi * 2 ** 31) + off_q31 + 2 ** 31) % 2 ** 32) - 2 ** 31
        elif angle_mode == "halls":
            halls.update(motor.theta, TS)
            theta = halls.angle(fw.i_q_setpoint)
        else:
            theta = fw.obs.teta
        if coast and FIXED_FIRMWARE:
            if fw.i_q_setpoint or fw.brake:
                zero_since = t
            elif t - zero_since > 0.1:
                fw.moe = False
            if fw.i_q_setpoint and not fw.moe:
                fw.moe = True
        if not fw.moe:
            # all switches open: current only through the body diodes when the line-line back-EMF exceeds the battery
            e_ll = math.sqrt(3) * motor.emf_phase_peak()
            i_dc = max(0.0, (e_ll - vbat) / (2 * motor.R)) * 0.9
            motor.iq = -i_dc * vbat / (1.5 * max(motor.emf_phase_peak(), 1e-6)) * (1 if motor.w > 0 else -1)
            motor.id = 0.0
            motor.theta = (motor.theta + motor.w * TS) % (2 * math.pi)
            if t >= next_log:
                next_log += log_every
                rows.append(dict(t=t, iq_true=motor.iq, id_true=0.0, w=motor.w, sp=0, iq_fw=0, id_fw=0, u_abs=0,
                                 ibat_fw=0, ibat_true=-i_dc, bc=0, iph_peak=abs(motor.iq)))
            continue
        fw.foc(ma, mb, theta)
        legs = leg_voltages(fw.switchtime, vbat)
        mean = sum(legs) / 3
        va, vb, vc = (x - mean for x in legs)
        v_alpha = (2 / 3) * (va - 0.5 * vb - 0.5 * vc)
        v_beta = (2 / 3) * (math.sqrt(3) / 2) * (vb - vc)
        i_dc = sum((st_min / T_PWM) * i for st_min, i in zip([min(s, T_PWM) for s in fw.switchtime], (ia, ib, ic)))
        motor.step(v_alpha, v_beta, TS)
        if t >= next_log:
            next_log += log_every
            row = dict(t=t, iq_true=motor.iq, id_true=motor.id, w=motor.w, sp=fw.i_q_setpoint * CAL_I / 1000,
                       iq_fw=fw.i_q * CAL_I / 1000, id_fw=fw.i_d * CAL_I / 1000, u_abs=fw.u_abs,
                       ibat_fw=fw.battery_current / 1000, ibat_true=i_dc, bc=fw.bc_flag,
                       iph_peak=math.hypot(motor.id, motor.iq))
            if fw.sensorless:
                row["obs_err"] = ang_err_deg(fw.obs.teta, motor.theta + angle_offset)
                row["obs_running"] = fw.obs.running
            if angle_mode == "halls":
                row["hall_err"] = ang_err_deg(theta, motor.theta + angle_offset)
            rows.append(row)
    return rows


def amps_to_units(a):
    return int(round(a * 1000 / CAL_I))


def summary(rows, t0=None, keys=("iq_true", "id_true", "iq_fw", "u_abs", "ibat_fw", "ibat_true")):
    sel = [r for r in rows if t0 is None or r["t"] >= t0]
    out = {}
    for k in keys:
        vals = [r[k] for r in sel if k in r]
        if vals:
            out[k] = (min(vals), sum(vals) / len(vals), max(vals))
    return out


def fmt(s):
    return ", ".join(f"{k} {v[1]:.1f} [{v[0]:.1f}..{v[2]:.1f}]" for k, v in s.items())


# ----------------------------------------------------------------------------------------------- scenarios
def main():
    report = []

    def section(title):
        print("\n=== " + title); report.append(title)

    lam, R, L = 0.010, 0.15, 0.25e-3
    w_noload = (36.0 / math.sqrt(3)) / lam

    # 0. which current sign / angle convention gives a stable current loop (the real hardware works, so one must)
    section("0. convention check (current sign x angle offset), locked rotor, 10 A")
    best = None
    for sign in (-1, 1):
        for off in (0.0, math.pi):
            FINDINGS.clear()
            m = Motor(R, L, lam, w=0.0)
            fw = Firmware(amps_to_units(60))
            rows = run(m, fw, 0.15, lambda t: amps_to_units(10), current_sign=sign, angle_offset=off)
            s = summary(rows, 0.1, ("iq_true", "id_true", "iq_fw"))
            stable = abs(abs(s["iq_true"][1]) - 10) < 1.5 and abs(s["id_true"][1]) < 1.5 and s["iq_true"][2] - s["iq_true"][0] < 2
            print(f"  sign {sign:+d} offset {math.degrees(off):5.0f}: {fmt(s)}  -> {'STABLE' if stable else 'unstable'}")
            if stable and best is None:
                best = (sign, off, 1 if s['iq_true'][1] > 0 else -1)
    if not best:
        print("  no stable convention found - model mapping wrong"); return
    SIGN, OFF, DIR = best   # DIR: electrical direction a positive setpoint drives the motor
    print(f"  using current sign {SIGN:+d}, angle offset {math.degrees(OFF):.0f} deg")

    # 1. locked rotor, current up to the maximum the ADC can measure
    section("1. locked rotor, current steps (hall/ideal angle)")
    for amps in (10, 20, 30, 40, 50, 60, 70):
        FINDINGS.clear()
        m = Motor(R, L, lam, w=0.0)
        fw = Firmware(amps_to_units(amps))
        rows = run(m, fw, 0.2, lambda t, a=amps: amps_to_units(a), current_sign=SIGN, angle_offset=OFF)
        s = summary(rows, 0.15, ("iq_true", "id_true", "iq_fw", "u_abs"))
        print(f"  {amps:3d} A: {fmt(s)}  findings: {dict(FINDINGS) or '-'}")

    # 2. constant speed (dyno), step to max current: voltage limit, d current, battery current estimate
    section("2. dyno at fixed speed, step to 30 A (battery limit off)")
    for frac in (0.0, 0.25, 0.5, 0.75, 0.9, 1.05):
        FINDINGS.clear()
        w = DIR * frac * w_noload
        m = Motor(R, L, lam, w=w)
        fw = Firmware(amps_to_units(30))
        rows = run(m, fw, 0.25, lambda t: 0 if t < 0.05 else amps_to_units(30), current_sign=SIGN, angle_offset=OFF)
        s = summary(rows, 0.2)
        print(f"  {frac * 100:5.0f}% speed: {fmt(s)}  findings: {dict(FINDINGS) or '-'}")

    # 3. battery current limit at high phase current
    section("3. battery current limit 15 A, phase current request 40 A, at 60% speed")
    FINDINGS.clear()
    m = Motor(R, L, lam, w=DIR * 0.6 * w_noload)
    fw = Firmware(amps_to_units(40), battery_current_max_ma=15000)
    rows = run(m, fw, 0.4, lambda t: 0 if t < 0.05 else amps_to_units(40), current_sign=SIGN, angle_offset=OFF)
    s = summary(rows, 0.3, ("iq_true", "ibat_fw", "ibat_true", "u_abs", "bc"))
    print(f"  {fmt(s)}  findings: {dict(FINDINGS) or '-'}")
    tail = [r for r in rows if r["t"] > 0.3]
    print(f"  battery current true: {min(r['ibat_true'] for r in tail):.1f} .. {max(r['ibat_true'] for r in tail):.1f} A,"
          f" limit flag toggles: {sum(1 for a, b in zip(tail, tail[1:]) if a['bc'] != b['bc'])}")

    # 4. regen braking at speed
    section("4. regen: brake with -20 A at 70% speed, regen battery limit 8 A")
    FINDINGS.clear()
    m = Motor(R, L, lam, w=DIR * 0.7 * w_noload)
    fw = Firmware(amps_to_units(30), regen_max_ma=8000)
    rows = run(m, fw, 0.4, lambda t: 0 if t < 0.05 else -amps_to_units(20), current_sign=SIGN, angle_offset=OFF,
               brake_fn=lambda t: 1 if t >= 0.05 else 0)
    s = summary(rows, 0.3, ("iq_true", "ibat_fw", "ibat_true", "u_abs", "bc"))
    print(f"  {fmt(s)}  findings: {dict(FINDINGS) or '-'}")

    # 5. full current start from standstill with the hall interpolation / 6-step, free spinning wheel
    section("5. hall motor: 30 A start from standstill, free acceleration (hall interpolation + 6-step)")
    FINDINGS.clear()
    m = Motor(R, L, lam, J=2e-5, load=0.002, w=0.0, theta=0.3)
    fw = Firmware(amps_to_units(30))
    h = Halls(int(OFF / math.pi * 2 ** 31))
    rows = run(m, fw, 1.0, lambda t: amps_to_units(30), current_sign=SIGN, angle_offset=OFF, angle_mode="halls", halls=h)
    for r in rows[::10]:
        print(f"  t {r['t']:.2f}s  w {r['w']:8.0f} rad/s  iq {r['iq_true']:6.1f} A  id {r['id_true']:6.1f} A  hall angle err {r['hall_err']:6.1f} deg  u_abs {r['u_abs']}")
    print(f"  findings: {dict(FINDINGS) or '-'}")

    # 6. coasting above the no-load speed with no current requested (direct drive downhill / full battery)
    section("6. no current requested at 90..130% of no-load speed (direct drive coasting)")
    for frac in (0.9, 1.0, 1.05, 1.15, 1.3):
        FINDINGS.clear()
        m = Motor(R, L, lam, w=DIR * frac * w_noload)
        fw = Firmware(amps_to_units(30))
        rows = run(m, fw, 0.4, lambda t: 0, current_sign=SIGN, angle_offset=OFF, coast=True)
        tail = [r for r in rows if r["t"] > 0.3]
        print(f"  {frac * 100:4.0f}%: battery current {sum(r['ibat_true'] for r in tail) / len(tail):6.1f} A, "
              f"PWM {'off (coasting)' if not fw.moe else 'on'}  findings: {dict(FINDINGS) or '-'}")

    # 7. sensorless with physical parameters converted to firmware units
    # observer offset +90 deg: atan2(e_alpha, -e_beta) adds +90, so the firmware frame is rotor + 180 deg (like the hall
    # autodetect finds), which turns the motor in the direction of the open loop start
    obs = dict(L=round(L * 2 ** 16 / 1.5), R=round(R * 2 ** 9 / 1.5), lam=round(lam * 256000), gamma=13, offset=2 ** 30)
    OFF, DIR = math.pi, 1
    section(f"7. sensorless observer accuracy (passive), firmware values L {obs['L']} R {obs['R']} flux {obs['lam']} offset +90 deg")
    for frac in (0.1, 0.5, 0.9):
        for amps in (5, 30, 50):
            FINDINGS.clear()
            m = Motor(R, L, lam, w=DIR * frac * w_noload)
            fw = Firmware(amps_to_units(amps), sensorless=True, obs=dict(obs, passive=True))
            rows = run(m, fw, 0.2, lambda t, a=amps: amps_to_units(a), current_sign=SIGN, angle_offset=OFF)
            e = [r["obs_err"] for r in rows if r["t"] > 0.12]
            print(f"  speed {frac * 100:3.0f}% {amps:3d} A: angle error {sum(e) / len(e):6.1f} deg [{min(e):.1f}..{max(e):.1f}]  findings: {dict(FINDINGS) or '-'}")

    section("8. sensorless closed loop at fixed speed")
    for frac in (0.2, 0.5, 0.8):
        for amps in (10, 30, 50):
            FINDINGS.clear()
            m = Motor(R, L, lam, w=DIR * frac * w_noload)
            fw = Firmware(amps_to_units(amps), sensorless=True, obs=obs)
            fw.obs.running = 1
            fw.obs.handover = fw.obs.HANDOVER
            rows = run(m, fw, 0.25, lambda t, a=amps: amps_to_units(a), current_sign=SIGN, angle_offset=OFF, angle_mode="sensorless")
            tail = [r for r in rows if r["t"] > 0.15]
            s = summary(tail, None, ("iq_true", "id_true"))
            print(f"  speed {frac * 100:3.0f}% {amps:3d} A: {fmt(s)}, observer running {tail[-1]['obs_running']}  findings: {dict(FINDINGS) or '-'}")

    section("9. sensorless start from standstill, peak phase current")
    for amps in (10, 30, 45):
        for load in (0.002, 0.03):
            FINDINGS.clear()
            m = Motor(R, L, lam, J=2e-5, load=load, w=0.0, theta=1.0)
            fw = Firmware(amps_to_units(amps), sensorless=True, obs=obs)
            rows = run(m, fw, 0.8, lambda t, a=amps: amps_to_units(a), current_sign=SIGN, angle_offset=OFF,
                       angle_mode="sensorless", log_every=TS)
            pk = max(r["iph_peak"] for r in rows)
            print(f"  {amps:3d} A load {load}: peak {pk:6.1f} A ({pk / amps:.2f}x), end speed {rows[-1]['w'] / w_noload * 100:+6.1f}% "
                  f"of no-load, observer running {rows[-1]['obs_running']}  findings: {dict(FINDINGS) or '-'}")


if __name__ == "__main__":
    sys.setrecursionlimit(10000)
    main()
