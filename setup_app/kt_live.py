"""Live data from a running KT firmware over SWIM (no reset, the motor keeps running).

The addresses come from the SDCC map file of the build (kunteng/build/main.map) or, for the prebuilt firmware
of the Android app, from kunteng.sym.json. SDCC stores multi byte values big endian.
"""
import re
import struct

import swim

# name in the firmware, type, label, unit, scale
LIVE_VARS = [
    ("ui8_adc_battery_voltage_filtered", "u8", "Battery", "V", 0.272),
    ("i16_motor_current_filtered_10b", "i16", "Battery current", "A", 0.125),
    ("ui16_motor_speed_erps", "u16", "Motor speed", "erps", 1),
    ("f_wheel_speed", "f32", "Speed", "km/h", 1),
    ("ui16_duty_cycle", "u16", "Duty cycle", "/508", 1),
    ("ui8_adc_throttle_value", "u8", "Throttle (ADC)", "", 1),
    ("ui8_pas_cadence_rpm", "u8", "Cadence", "rpm", 1),
    ("lcd_configuration_variables", "u8", "Assist level", "", 1),
    ("ui8_motor_controller_state", "u8", "Controller state", "", 1),
    ("ui8_motor_controller_error", "u8", "Error code", "", 1),
    ("ui8_motor_state", "u8", "Motor state", "", 1),
    ("ui8_motor_commutation_type", "u8", "Commutation", "", 1),
    ("ui8_hall_sensors_last", "u8", "Hall state", "", 1),
    ("ui8_angle_correction", "u8", "FOC angle correction", "/256", 1),
    ("ui8_phase_b_peak", "u8", "Phase current peak", "A", 0.121),
    ("ui8_coasting", "u8", "Coasting (PWM off)", "", 1),
    ("ui16_emf_duty_per_erps_x256", "u16", "Back-EMF constant", "", 1),
    ("i8_hall_offset", "i8x8", "Learned hall offsets", "/256", 1),
]
SIZES = {"u8": 1, "i8": 1, "u16": 2, "i16": 2, "f32": 4, "i8x8": 8}

STATE_BITS = [(2, "brake"), (4, "over current"), (8, "under voltage"), (32, "motor blocked"), (64, "over voltage"),
              (128, "hall error")]
MOTOR_STATES = {0: "coast", 1: "stop", 2: "start up", 3: "cool down", 4: "running"}
ERRORS = {0: "-", 0x21: "21 over current", 0x22: "22 throttle", 0x24: "24 hall sensors", 0x28: "over voltage",
          0x91: "battery empty"}


def read_map(path):
    """SDCC .map -> {name: address}. Names are cut at 31 characters in the map."""
    syms = {}
    with open(path) as f:
        for line in f:
            m = re.match(r"\s+([0-9A-F]{8})\s+_(\w+)\s", line)
            if m:
                syms[m.group(2)] = int(m.group(1), 16)
    return syms


def lookup(syms, name):
    if name in syms:
        return syms[name]
    return syms.get(name[:31])


def live_symbols(syms):
    """Only the live variables, {name: [address, size]} (also what goes into kunteng.sym.json)."""
    out = {}
    for name, typ, *_ in LIVE_VARS:
        a = lookup(syms, name)
        if a is not None:
            out[name] = [a, SIZES[typ]]
    return out


def decode(typ, raw):
    if typ == "u8":
        return raw[0]
    if typ == "i8":
        return struct.unpack(">b", raw[:1])[0]
    if typ == "u16":
        return struct.unpack(">H", raw[:2])[0]
    if typ == "i16":
        return struct.unpack(">h", raw[:2])[0]
    if typ == "f32":
        return struct.unpack(">f", raw[:4])[0]
    if typ == "i8x8":
        return [struct.unpack(">b", bytes([b]))[0] for b in raw[:8]]
    raise ValueError(typ)


class LiveSession:
    """Keeps a SWIM connection to the running controller and reads the live variables."""

    def __init__(self, symbols, transport=None):
        self.symbols = symbols          # {name: [address, size]}
        self.s = swim.Swim(transport)
        self.open = False

    def connect(self):
        self.s.open_live()
        self.open = True

    def snapshot(self):
        if not self.open:
            self.connect()
        snap = {}
        for name, typ, label, unit, scale in LIVE_VARS:
            if name not in self.symbols:
                continue
            addr, size = self.symbols[name]
            v = decode(typ, self.s.read_mem(addr, size))
            snap[name] = v if typ == "i8x8" else v * scale
        return snap

    def close(self):
        if self.open:
            self.s.close()
        self.open = False


def describe(snap):
    """[(label, text)] for the live page."""
    rows = []
    for name, typ, label, unit, scale in LIVE_VARS:
        if name not in snap:
            continue
        v = snap[name]
        if name == "ui8_motor_controller_state":
            bits = [t for b, t in STATE_BITS if int(v) & b]
            text = "ok" if int(v) == 1 else ", ".join(bits) or str(int(v))
        elif name == "ui8_motor_state":
            text = MOTOR_STATES.get(int(v), str(int(v)))
        elif name == "ui8_motor_controller_error":
            text = ERRORS.get(int(v), "%02x" % int(v))
        elif name == "ui8_motor_commutation_type":
            text = {1: "block (low speed)", 2: "sine wave"}.get(int(v), str(int(v)))
        elif name == "i8_hall_offset":
            text = " ".join("%+d" % v[st] for st in (4, 6, 2, 3, 1, 5))
        elif name == "ui8_coasting":
            text = "yes" if v else "no"
        elif isinstance(v, float) and scale != 1 or typ == "f32":
            text = "%.2f %s" % (v, unit)
        else:
            text = "%d %s" % (v, unit)
        rows.append((label, text.strip()))
    return rows
