#!/usr/bin/env python3
"""Run the real KT firmware in the ucsim STM8 simulator (ships with SDCC) and check the motor code.

What is simulated: the CPU, timers (PWM interrupt every 64 us), GPIO pins and the data EEPROM.
The ADC is not modelled by ucsim, so the firmware is built with -DSIMULATION (skips only the ADC
wait loops) and the ADC result registers are written by the test.

Checks:
  1. the settings block written into the EEPROM is used (kt_settings in RAM)
  2. hall sensor sequence at constant speed -> speed (erps), sine wave mode, interpolation step
  3. hall sensors that are not 60 deg apart -> the hall learning finds their real positions
  4. without a settings block the compile time defaults are used

Usage: python kunteng/sim/run_sim.py
"""
import os
import re
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
FW = os.path.dirname(HERE)
sys.path.insert(0, FW)
sys.path.insert(0, os.path.join(os.path.dirname(FW), "setup_app"))
import build  # noqa: E402
import kunteng_core as kt  # noqa: E402

PWM_HZ = 15625
CPU_HZ = 16_000_000
# ucsim ignores the TIM1 repetition counter: the PWM interrupt comes at every counter turn (31.25 kHz)
# instead of every second one (15.625 kHz). The firmware counts PWM interrupts, so for the firmware
# time in the interrupt runs twice as fast. Hall timing is scaled by this to get the wanted erps.
SIM_ISR_SCALE = 2


def ucsim():
    sdcc = build.find_sdcc()
    p = os.path.join(os.path.dirname(sdcc), "ucsim_stm8.exe" if os.name == "nt" else "ucsim_stm8")
    if not os.path.isfile(p):
        p = os.path.join(os.path.dirname(sdcc), "sstm8" + (".exe" if os.name == "nt" else ""))
    return p


def symbols(map_file):
    syms = {}
    for line in open(map_file):
        m = re.match(r"\s+([0-9A-F]{8})\s+(_\w+)\s", line)
        if m:
            syms[m.group(2)[1:]] = int(m.group(1), 16)
    return syms


def struct_offsets():
    """Offsets inside struct_kt_settings (all uint8 except the uint16 array and ticks)."""
    names = ["cells", "motor_current_a", "regen_current_a", "throttle_type", "flags", "pas_magnets",
             "pas_max_cadence", "a1", "a2", "a3", "a4", "a5", "rotor_offset_angle", "foc_angle_adjust",
             "erps_interpolation", "default_max_speed", "default_wheel_size", "controller_type",
             "phase_current_factor_x10", "adc_battery_voltage_max", "adc_battery_voltage_min",
             "communications_battery_voltage"]
    off = {n: i for i, n in enumerate(names)}
    i = len(names)
    off["pack_volts"] = i
    i += 8
    off["pas_max_cadence_ticks"] = i
    off["pas_min_cadence_ticks"] = i + 2
    off["phase_current_max"] = i + 4
    off["from_eeprom"] = i + 5
    return off


class Sim:
    def __init__(self, ihx, syms):
        self.ihx, self.syms, self.cmds = ihx, syms, []
        self.dumps = []

    def cmd(self, c):
        self.cmds.append(c)

    def setmem(self, addr, data):
        vals = " ".join("0x%02x" % b for b in data)
        if 0x4000 <= addr < 0x4800:   # the EEPROM is write protected through the address space
            self.cmd("set memory eeprom_chip 0x%x %s" % (addr - 0x4000, vals))
            shown = addr - 0x4000
        else:
            self.cmd("set memory rom 0x%x %s" % (addr, vals))
            shown = addr
        # ucsim prints the written bytes like a dump: consume that output so it is not taken for a later dump
        self.dumps.append((None, shown, len(data)))

    def run_ms(self, ms):
        self.cmd("step %d us" % int(round(ms * 1000)))  # ucsim does not take fractional ms

    def dump(self, tag, name, n):
        a = self.syms[name] if isinstance(name, str) else name
        self.dumps.append((tag, a, n))
        self.cmd("dump rom 0x%x 0x%x 16" % (a, a + n - 1))

    def go(self, timeout=60):
        """Feed the commands one by one. ucsim drops input that arrives while a 'step' is still running,
        so after every step wait for its "Simulated ... ticks" line before sending the next command."""
        import queue
        import threading
        p = subprocess.Popen([ucsim(), "-t", "STM8S105", "-X", "16M", self.ihx], stdin=subprocess.PIPE,
                             stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True, bufsize=1)
        q, out = queue.Queue(), []

        def reader():
            for line in p.stdout:
                q.put(line)
            q.put(None)
        threading.Thread(target=reader, daemon=True).start()

        # ucsim buffers its output in a pipe, so it can't be watched: pace by the simulated time instead
        # (it simulates at roughly 0.15 ... 0.25 x real time on a normal PC, 0.06 is on the safe side)
        import time
        try:
            for c in self.cmds:
                p.stdin.write(c + "\n")
                p.stdin.flush()
                m = re.match(r"step (\d+) us", c)
                if m:
                    time.sleep(0.03 + int(m.group(1)) * 1e-6 / 0.06)
            time.sleep(2)  # the last dumps must not be cut off by kill
            p.stdin.write("kill\n")
            p.stdin.close()
            while True:
                line = q.get(timeout=timeout)
                if line is None:
                    break
                out.append(line)
        finally:
            p.kill()
        self.output = "".join(out)
        return parse_dumps(self.output, self.dumps)


def parse_dumps(out, dumps):
    """Dump results in command order, found by their start address (ucsim prints echo text out of order)."""
    lines = out.splitlines()
    res, pos = {}, 0
    row = re.compile(r"0x([0-9a-f]{3,6}) {2,}((?:[0-9a-f]{2} )+)")  # anywhere in the line: input echo may be mixed in
    for tag, addr, n in dumps:
        data = bytearray()
        start = pos
        while pos < len(lines):
            m = row.search(lines[pos])
            pos += 1
            if m and int(m.group(1), 16) == addr:
                data += bytes(int(x, 16) for x in m.group(2).split())
                while len(data) < n and pos < len(lines):
                    m = row.search(lines[pos])
                    if not m:
                        break
                    data += bytes(int(x, 16) for x in m.group(2).split())
                    pos += 1
                break
        if not data:
            pos = start  # not found: the next dump may still be there
        if tag is not None:
            res[tag] = data[:n]
    return res


def boot(sim, block=None):
    sim.cmd("expression pa_pins=0xff")       # brake released (active low), all port A high
    sim.cmd("expression pd_pins=0xff")       # PAS / over current inputs idle high
    sim.cmd("expression pe_pins=0x04")       # hall state 4
    sim.setmem(0x53F2, [100])                # ADC battery voltage (8 bit), set per test below
    sim.setmem(0x53F0, [0x40, 0x00])         # motor current ADC at zero current: calibrated as offset at start up
    # display values with a valid key, so the firmware does not write EEPROM defaults at start up
    # (ucsim's flash controller never reports "done", the write would wait forever)
    sim.setmem(0x4000, [0xCA, 2, 202, 0x14, 25, 1, 10])
    if block is not None:
        sim.setmem(kt.SETTINGS_BLOCK_ADDR, block)
    sim.run_ms(400)                          # start up (timer2 delays, ADC calibration)


def hall_run(sim, erps, revs, widths=(1, 1, 1, 1, 1, 1)):
    """Turn the 'motor' at erps. widths: relative length of the hall states 4,6,2,3,1,5."""
    order = [4, 6, 2, 3, 1, 5]
    tot = sum(widths)
    per_rev_ms = 1000.0 / erps / SIM_ISR_SCALE
    for _ in range(revs):
        for st, w in zip(order, widths):
            sim.cmd("expression pe_pins=0x%02x" % st)
            sim.run_ms(per_rev_ms * w / tot)


def check(cond, text, fails):
    print(("  ok    " if cond else "  FAIL  ") + text)
    if not cond:
        fails.append(text)


def main():
    out = os.path.join(FW, "build", "sim")
    print("building firmware for the simulator (-DSIMULATION) ...")
    ihx = build.build(out, ["SIMULATION"], log=lambda _l: None)
    syms = symbols(os.path.join(out, "main.map"))
    off = struct_offsets()
    fails = []

    # ---------------------------------------------------------------- 1 + 2 + 3
    s = kt.default_settings()
    s.update(cells=13, motor_current_a=18, controller=2, pas_magnets=12, rotor_offset_angle=200, erps_interpolation=15)
    block = kt.settings_block(s)
    sim = Sim(ihx, syms)
    boot(sim, block)
    sim.setmem(0x53F2, [180])   # 13S at ~49 V: 180 * 0.272
    sim.dump("settings", "kt_settings", 40)
    # constant speed, hall sensors evenly spaced
    hall_run(sim, 50, 30)
    sim.dump("erps", "ui16_motor_speed_erps", 2)
    sim.dump("commutation", "ui8_motor_commutation_type", 1)
    sim.dump("step", "ui16_angle_step_x256", 2)
    sim.dump("period", "ui16_period_sum", 2)
    sim.dump("offsets_even", "i8_hall_offset", 8)
    # now hall sensors misplaced: state 6 starts 10/256 late, state 3 starts 8/256 early
    # widths in 1/256 of a revolution for states 4,6,2,3,1,5 (nominal 42.67 each)
    w = [52.67, 32.67, 34.67, 50.67, 42.67, 42.67]
    hall_run(sim, 50, 200, w)
    sim.dump("offsets_learned", "i8_hall_offset", 8)
    print("running the simulation (takes a minute) ...")
    r = sim.go()

    st = r.get("settings", b"")
    print("settings block -> kt_settings")
    check(len(st) >= 40 and st[off["from_eeprom"]] == 1, "settings block accepted", fails)
    if len(st) >= 40:
        check(st[off["cells"]] == 13 and st[off["motor_current_a"]] == 18 and st[off["pas_magnets"]] == 12,
              "cells 13, current 18 A, 12 PAS magnets", fails)
        check(st[off["adc_battery_voltage_max"]] == (13 * 420) // 27, "battery max from cells (%d)" % st[off["adc_battery_voltage_max"]], fails)
        check(st[off["communications_battery_voltage"]] == 48, "display voltage 48 V", fails)
        check(st[off["phase_current_max"]] == 90, "phase current limit 18 A * 2.5 = 90 steps (%d)" % st[off["phase_current_max"]], fails)
    erps = int.from_bytes(r.get("erps", b"\0\0")[:2], "little")
    print("hall sequence at 50 erps")
    # SDCC stores 16 bit values big endian
    erps = int.from_bytes(r.get("erps", b"\0\0")[:2], "big")
    check(48 <= erps <= 52, "speed %d erps" % erps, fails)
    check(r.get("commutation", b"\0")[0] == 2, "sine wave interpolation active", fails)
    period = int.from_bytes(r.get("period", b"\0\0")[:2], "big")
    step = int.from_bytes(r.get("step", b"\0\0")[:2], "big")
    check(abs(period - PWM_HZ / 50) < 8, "6 sector sum %d PWM cycles (expected %d)" % (period, PWM_HZ // 50), fails)
    check(step == 0xFFFF // period if period else False, "angle step %d per PWM cycle" % step, fails)
    even = [int.from_bytes([b], "big", signed=True) for b in r.get("offsets_even", bytes(8))]
    check(all(abs(even[i]) <= 1 for i in range(1, 7)), "evenly spaced halls -> offsets ~0 %s" % even[1:7], fails)
    lr = [int.from_bytes([b], "big", signed=True) for b in r.get("offsets_learned", bytes(8))]
    # expected edge errors (state: offset) relative to the mean
    pos, exp = 0.0, {}
    nominal = {4: 0, 6: 42, 2: 85, 3: 127, 1: 170, 5: 212}
    for stt, wd in zip([4, 6, 2, 3, 1, 5], w):
        exp[stt] = pos - nominal[stt]
        pos += wd
    mean = sum(exp.values()) / 6
    exp = {k: v - mean for k, v in exp.items()}
    print("misplaced halls, learned offsets (state: learned / real)")
    for stt in [4, 6, 2, 3, 1, 5]:
        print("        %d: %+d / %+.1f" % (stt, lr[stt], exp[stt]))
    check(all(abs(lr[k] - exp[k]) <= 2.5 for k in exp), "learned offsets within 2.5/256 (3.5 deg) of the real ones", fails)

    # ---------------------------------------------------------------- 4: no block -> defaults
    sim = Sim(ihx, syms)
    boot(sim, None)
    sim.dump("settings", "kt_settings", 40)
    r = sim.go()
    st = r.get("settings", b"")
    print("no settings block")
    check(len(st) >= 40 and st[off["from_eeprom"]] == 0 and st[off["cells"]] == 7,
          "compile time defaults used (7S)", fails)

    # ---------------------------------------------------------------- 5: phase current limit
    # 10 A battery current at a low duty cycle means a high phase current: the limit must pull the duty
    # cycle down. Same situation with the limit switched off in the settings: duty cycle stays.
    print("phase current limit (10 A battery current, duty cycle 100/508)")
    duty_after = {}
    for limit_on in (True, False):
        s = kt.default_settings()
        s.update(cells=10, motor_current_a=15, phase_current_limit=limit_on)
        sim = Sim(ihx, syms)
        boot(sim, kt.settings_block(s))
        sim.setmem(0x53F2, [140])                    # 10S at 38 V
        sim.setmem(0x53F0, [0x40 + 20, 0])           # total current: 20 steps of 0.5 A above the zero offset
        sim.setmem(syms["ui16_duty_cycle"], [0, 100])
        sim.setmem(syms["ui16_duty_cycle_target"], [0, 100])
        sim.run_ms(2)                                # ~60 PWM interrupts in the simulator
        sim.dump("duty", "ui16_duty_cycle", 2)
        r = sim.go()
        duty_after[limit_on] = int.from_bytes(r.get("duty", b"\0\0")[:2], "big")
    print("        duty cycle after 2 ms: limit on %d, limit off %d" % (duty_after[True], duty_after[False]))
    check(duty_after[True] < 80, "limit on: duty cycle reduced", fails)
    check(duty_after[False] >= 95, "limit off: duty cycle kept", fails)

    print()
    print("ALL OK" if not fails else "%d FAILED" % len(fails))
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
