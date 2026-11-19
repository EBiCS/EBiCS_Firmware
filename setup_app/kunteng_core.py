"""Kunteng (KT, STM8S105) support for the EBiCS Setup app: settings, build and flash.

The firmware lives in ../kunteng (casainho's KT firmware with fixes). Settings are passed to the compiler
as -D defines, building needs SDCC. Flashing goes over SWIM with the ST-Link (swim.py).
"""
import json
import sys
from pathlib import Path

import swim

APP_DIR = Path(__file__).resolve().parent
REPO = APP_DIR.parent
FW_DIR = REPO / "kunteng"
BUILD_DIR = FW_DIR / "build"
HEX_FILE = BUILD_DIR / "main.ihx"
SETTINGS_FILE = APP_DIR / "my_kunteng.json"

NAME = "Kunteng"
SUBTITLE = "Kunteng (KT) controller setup"

sys.path.insert(0, str(FW_DIR))
import build as fw_build  # noqa: E402  (kunteng/build.py)

# choice fields: key -> [(label, value), ...]
CHOICES = {
    "controller": [("S06S / KT 6 MOSFET (up to 15 A)", 1), ("S12S / KT 12 MOSFET (up to 25 A)", 2)],
    "throttle_type": [("Throttle and/or PAS", 1), ("Torque sensor (on the throttle input)", 2)],
    "wheel": [("16 inch", 0x00), ("20 inch", 0x08), ("24 inch", 0x10), ("26 inch", 0x14), ("700c", 0x18),
              ("28 inch", 0x1C), ("29 inch", 0x1E)],
    "pas_direction": [("Right (normal)", 0), ("Left", 1)],
}

FIELDS = [
    # key, label, kind, default, section, unit, help
    ("controller", "Controller", "choice", 1, "basic", "",
     "Which KT controller you have. The 12 MOSFET boards (S12S) allow more current."),
    ("cells", "Battery cells in series", "int", 10, "basic", "S",
     "7S = 24 V, 10S = 36 V, 13S = 48 V, 14S = 52 V. Sets the low voltage cutoff (3.0 V per cell) and the battery bars."),
    ("motor_current_a", "Max motor current", "int", 15, "basic", "A",
     "Battery side current limit. S06S boards: up to 15 A, S12S: up to 25 A."),
    ("throttle_type", "Throttle / torque sensor", "choice", 1, "basic", "",
     "Throttle and PAS: power from the throttle or the pedal cadence, whichever is higher. "
     "Torque sensor: a bottom bracket torque sensor wired to the throttle input."),
    ("max_speed", "Speed limit before the display sends its own", "int", 25, "basic", "km/h",
     "Used at power up until the display has sent its settings. The display setting wins after that."),
    ("wheel", "Wheel size before the display sends its own", "choice", 0x14, "basic", "",
     "Used at power up until the display has sent its settings."),
    # advanced
    ("regen_current_a", "Regen current while braking", "int", 0, "advanced", "A",
     "Electric braking with the brake lever. Only works with direct drive (no freewheel) motors. 0 = off."),
    ("direct_drive", "Direct drive hub motor", "bool", False, "advanced", "",
     "Tick for big direct drive hub motors: the PWM stays on while braking so regen can work."),
    ("pas_magnets", "PAS magnets", "int", 8, "advanced", "",
     "Number of magnets on the PAS disc (8 or 12 on most sensors)."),
    ("pas_direction", "PAS direction", "choice", 0, "advanced", "",
     "Switch this if the motor only assists when pedalling backwards."),
    ("pas_max_cadence", "Cadence for full PAS power", "int", 90, "advanced", "rpm",
     "At this pedal cadence PAS gives the full power of the assist level."),
    ("assist_1", "Assist level 1", "float", 0.2, "advanced", "x", "Power factor of display level 1 (0 ... 1)."),
    ("assist_2", "Assist level 2", "float", 0.4, "advanced", "x", "Power factor of display level 2 (0 ... 1)."),
    ("assist_3", "Assist level 3", "float", 0.6, "advanced", "x", "Power factor of display level 3 (0 ... 1)."),
    ("assist_4", "Assist level 4", "float", 0.8, "advanced", "x", "Power factor of display level 4 (0 ... 1)."),
    ("assist_5", "Assist level 5", "float", 1.0, "advanced", "x", "Power factor of display level 5 (0 ... 1)."),
    ("over_voltage_protection", "Over voltage protection", "bool", False, "advanced", "",
     "Stops the motor above 4.2 V per cell (shows error 05) and restarts when the voltage drops again."),
    # expert
    ("rotor_offset_angle", "Hall sensor angle offset", "int", 202, "expert", "/256",
     "Angle between the hall sensors and the rotor (256 = 360 deg). 202 fits the Q85/Q100 geared motors. "
     "Wrong values give weak or no torque at start up."),
    ("foc_angle_adjust", "FOC current sample angle", "int", 137, "expert", "/256",
     "Where the phase B current is sampled for the automatic angle correction. 137 fits Q85/Q100 motors."),
    ("erps_interpolation", "Speed for sine wave mode", "int", 15, "expert", "erps",
     "Above this motor speed the controller switches from block commutation to smooth sine wave."),
    ("phase_current_limit", "Limit phase current at low speed", "bool", True, "expert", "",
     "Also limits the motor phase current (estimated from battery current and duty cycle). Protects the "
     "MOSFETs when starting under load."),
    ("hall_learning", "Learn hall sensor positions", "bool", True, "expert", "",
     "Measures the real angle of each hall sensor while riding at steady speed and corrects for it. "
     "Gives smoother and quieter running on motors with badly placed sensors."),
]
FIELD_BY_KEY = {f[0]: f for f in FIELDS}


def default_settings():
    return {f[0]: f[3] for f in FIELDS}


def load_settings():
    s = default_settings()
    if SETTINGS_FILE.is_file():
        try:
            s.update(json.loads(SETTINGS_FILE.read_text(encoding="utf-8")))
        except (OSError, ValueError):
            pass
    return s


def save_settings(s):
    SETTINGS_FILE.write_text(json.dumps(s, indent=2), encoding="utf-8")


def check_settings(s):
    p = []
    if not 7 <= int(s["cells"]) <= 16:
        p.append("Battery cells must be 7 ... 16 (the controller measures up to about 69 V).")
    imax = 18 if int(s["controller"]) == 1 else 30
    if not 1 <= int(s["motor_current_a"]) <= imax:
        p.append(f"Max motor current must be 1 ... {imax} A for this controller.")
    if not 0 <= int(s["regen_current_a"]) <= 15:
        p.append("Regen current must be 0 ... 15 A.")
    if not 1 <= int(s["pas_magnets"]) <= 64:
        p.append("PAS magnets must be 1 ... 64.")
    if not 30 <= int(s["pas_max_cadence"]) <= 150:
        p.append("Cadence for full PAS power must be 30 ... 150 rpm.")
    for i in range(1, 6):
        if not 0.0 <= float(s[f"assist_{i}"]) <= 1.0:
            p.append(f"Assist level {i} must be between 0 and 1.")
    if not 5 <= int(s["max_speed"]) <= 70:
        p.append("Speed limit must be 5 ... 70 km/h.")
    for k in ("rotor_offset_angle", "foc_angle_adjust"):
        if not 0 <= int(s[k]) <= 255:
            p.append(f"{FIELD_BY_KEY[k][1]} must be 0 ... 255.")
    if not 5 <= int(s["erps_interpolation"]) <= 100:
        p.append("Speed for sine wave mode must be 5 ... 100 erps.")
    return p


def defines(s):
    """Compiler defines for the settings."""
    d = [
        "CONTROLLER_TYPE=%d" % int(s["controller"]),
        "MOTOR_TYPE=%d" % (3 if s["direct_drive"] else 1),   # 3 = Q11: direct drive handling, Q85 motor constants
        "BATTERY_LI_ION_CELLS_NUMBER=%d" % int(s["cells"]),
        "MOTOR_CURRENT_MAX_A=%d" % int(s["motor_current_a"]),
        "MOTOR_REGEN_CURRENT_MAX_A=%d" % int(s["regen_current_a"]),
        "EBIKE_THROTTLE_TYPE=%d" % int(s["throttle_type"]),
        "DEFAULT_VALUE_MAX_SPEED=%d" % int(s["max_speed"]),
        "DEFAULT_VALUE_WHEEL_SIZE=%d" % int(s["wheel"]),
        "PAS_NUMBER_MAGNETS=%d" % int(s["pas_magnets"]),
        "PAS_DIRECTION=%d" % int(s["pas_direction"]),
        "PAS_MAX_CADENCE_RPM=%d" % int(s["pas_max_cadence"]),
        "MOTOR_ROTOR_OFFSET_ANGLE=%d" % int(s["rotor_offset_angle"]),
        "FOC_READ_ID_CURRENT_ANGLE_ADJUST=%d" % int(s["foc_angle_adjust"]),
        "MOTOR_ROTOR_ERPS_START_INTERPOLATION_60_DEGREES=%d" % int(s["erps_interpolation"]),
    ]
    for i in range(1, 6):
        d.append("PAS_ASSIST_LEVEL_%d=%s" % (i, repr(float(s[f"assist_{i}"]))))
    if s["over_voltage_protection"]:
        d.append("BATTERY_OVER_VOLTAGE_PROTECTION")
    if not s.get("phase_current_limit", True):
        d.append("DISABLE_PHASE_CURRENT_LIMIT")
    if not s.get("hall_learning", True):
        d.append("DISABLE_HALL_LEARNING")
    return d


def describe_build(s):
    return "\n".join("-D" + x for x in defines(s))


# Runtime settings block in the STM8 data EEPROM, read by the firmware at power up (kunteng/settings.h).
SETTINGS_BLOCK_ADDR = 0x4080
SETTINGS_BLOCK_LEN = 32
SETTINGS_BLOCK_VERSION = 1
FLAG_DIRECT_DRIVE, FLAG_PAS_LEFT, FLAG_OVER_VOLTAGE, FLAG_NO_PHASE_LIMIT, FLAG_NO_HALL_LEARNING = 1, 2, 4, 8, 16


def settings_block(s):
    """32 byte settings block for the firmware. Same layout as kunteng/settings.h."""
    flags = ((FLAG_DIRECT_DRIVE if s["direct_drive"] else 0) |
             (FLAG_PAS_LEFT if int(s["pas_direction"]) == 1 else 0) |
             (FLAG_OVER_VOLTAGE if s["over_voltage_protection"] else 0) |
             (0 if s.get("phase_current_limit", True) else FLAG_NO_PHASE_LIMIT) |
             (0 if s.get("hall_learning", True) else FLAG_NO_HALL_LEARNING))
    b = bytearray(SETTINGS_BLOCK_LEN)
    b[0:4] = bytes([ord("K"), ord("T"), SETTINGS_BLOCK_VERSION, SETTINGS_BLOCK_LEN])
    b[4] = int(s["cells"])
    b[5] = int(s["motor_current_a"])
    b[6] = int(s["regen_current_a"])
    b[7] = int(s["throttle_type"])
    b[8] = flags
    b[9] = int(s["pas_magnets"])
    b[10] = int(s["pas_max_cadence"])
    for i in range(5):
        b[11 + i] = int(round(float(s[f"assist_{i + 1}"]) * 100))
    b[16] = int(s["rotor_offset_angle"]) & 0xFF
    b[17] = int(s["foc_angle_adjust"]) & 0xFF
    b[18] = int(s["erps_interpolation"])
    b[19] = int(s["max_speed"])
    b[20] = int(s["wheel"])
    b[21] = int(s["controller"])
    b[22] = int(s.get("phase_current_factor_x10", 25))
    b[31] = (~sum(b[:31])) & 0xFF
    return bytes(b)


def find_compiler():
    return fw_build.find_sdcc()


def build_firmware(s, log=print):
    try:
        fw_build.build(str(BUILD_DIR), defines(s), log=log)
        return True
    except RuntimeError as e:
        for line in str(e).splitlines():
            log("ERROR: " + line)
        return False


def pyusb_ok():
    try:
        import usb.core  # noqa: F401
        import libusb_package  # noqa: F401
        return True
    except ImportError:
        return False


def install_pyusb(log=print):
    """pip install the USB modules the SWIM driver needs."""
    import subprocess
    r = subprocess.run([sys.executable, "-m", "pip", "install", "--user", "pyusb", "libusb-package"],
                       capture_output=True, text=True)
    for line in (r.stdout + r.stderr).splitlines()[-5:]:
        log(line)
    return r.returncode == 0


def build_and_flash(s, log=print, compile_first=True):
    problems = check_settings(s)
    if problems:
        for p in problems:
            log("ERROR: " + p)
        return False
    if compile_first:
        log("--- building firmware ---")
        if not build_firmware(s, log):
            return False
    if not HEX_FILE.is_file():
        log("ERROR: no compiled firmware found")
        return False
    log("--- installing on the controller (SWIM) ---")
    try:
        return swim.flash(str(HEX_FILE), log=log, settings_block=settings_block(s))
    except swim.SwimError as e:
        log("ERROR: " + str(e))
    except Exception as e:  # USB errors
        log("ERROR: USB: %s" % e)
    return False


def write_settings(s, log=print):
    """Settings only, no reinstall: the controller is restarted and reads them at start up."""
    problems = check_settings(s)
    if problems:
        for p in problems:
            log("ERROR: " + p)
        return False
    try:
        return swim.write_settings(settings_block(s), log=log)
    except swim.SwimError as e:
        log("ERROR: " + str(e))
    except Exception as e:
        log("ERROR: USB: %s" % e)
    return False


def probe():
    return swim.probe()


def unlock(log=print):
    try:
        return swim.unlock(log=log)
    except swim.SwimError as e:
        log("ERROR: " + str(e))
    except Exception as e:
        log("ERROR: USB: %s" % e)
    return False


def describe_probe(i):
    if not isinstance(i, dict):
        return f"error: {i}"
    if not i["probe"]:
        return "ST-Link NOT found or not usable. " + (i["error"] or "")
    if not i["target"]:
        return ("ST-Link found, but the KT controller does not answer. Check the 4 wires (3.3V, SWIM, GND, RST) "
                "and that the controller is powered from the battery." + (f"  ({i['error']})" if i["error"] else ""))
    if i["locked"]:
        return "Controller found - chip is LOCKED (original KT firmware). Unlock it before installing."
    return "Controller found - chip is unlocked, ready to install."
