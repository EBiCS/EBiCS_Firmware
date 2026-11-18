"""
EBiCS Setup - core logic (no GUI).

Tool discovery, bike settings, config.h generation, firmware build,
and ST-Link access through the STM32CubeProgrammer command line.

Part of EBiCS, GPL v3 or later.
"""

import glob
import json
import os
import re
import shutil
import subprocess
import threading
import zipfile
from pathlib import Path

APP_DIR = Path(__file__).resolve().parent
REPO = APP_DIR.parent
DEBUG_DIR = REPO / "Debug"
MAKE_DIR = REPO / "make"
SETTINGS_FILE = APP_DIR / "my_bike.json"
FLASHED_ELF = APP_DIR / "last_flashed.elf"
HEX_FILE = DEBUG_DIR / "LishuiFOC_01.hex"
ELF_FILE = DEBUG_DIR / "LishuiFOC_01.elf"

CREATE_NO_WINDOW = 0x08000000 if os.name == "nt" else 0
SETUP_MAGIC = 0xEB1C5002


# --------------------------------------------------------------------------------------------
# Tools
# --------------------------------------------------------------------------------------------

def _first(paths):
    for p in paths:
        if p and Path(p).is_file():
            return str(Path(p))
    return None


def find_cube_cli():
    return _first([
        r"C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe",
        r"C:\Program Files (x86)\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe",
        shutil.which("STM32_Programmer_CLI"),
    ] + glob.glob(r"C:\ST\STM32CubeIDE*\STM32CubeIDE\plugins\*cubeprogrammer*\tools\bin\STM32_Programmer_CLI.exe"))


def find_stlink_cli():
    return _first([
        r"C:\Program Files (x86)\STMicroelectronics\STM32 ST-LINK Utility\ST-LINK Utility\ST-LINK_CLI.exe",
        shutil.which("ST-LINK_CLI"),
    ])


def _extract_ac6_compiler(tools_dir):
    """System Workbench ships the compiler as a zip that is only unpacked on first IDE use."""
    tools = Path(tools_dir)
    if (tools / "compiler" / "bin" / "arm-none-eabi-gcc.exe").is_file():
        return True
    zips = list(tools.glob("st-gnu-arm-gcc-*.zip"))
    if not zips:
        return False
    try:
        with zipfile.ZipFile(zips[0]) as z:
            z.extractall(tools / "compiler")
        return (tools / "compiler" / "bin" / "arm-none-eabi-gcc.exe").is_file()
    except (OSError, zipfile.BadZipFile):
        return False


def find_toolchain():
    """Returns (gcc_bin_dir, make_exe) or (None, None)."""
    gcc_dirs, makes = [], []
    for tools in sorted(glob.glob(r"C:\Ac6\SystemWorkbench\plugins\fr.ac6.mcu.externaltools.arm-none.win32_*\tools"), reverse=True):
        if _extract_ac6_compiler(tools):
            gcc_dirs.append(str(Path(tools) / "compiler" / "bin"))
        makes.append(str(Path(tools) / "make" / "make.exe"))
    for pattern in (r"C:\ST\STM32CubeIDE*\STM32CubeIDE\plugins\*gnu-tools-for-stm32*\tools\bin",
                    os.path.expandvars(r"%LOCALAPPDATA%\xPacks\@xpack-dev-tools\arm-none-eabi-gcc\*\.content\bin"),
                    r"C:\Program Files (x86)\GNU Arm Embedded Toolchain\*\bin",
                    r"C:\Program Files\Arm GNU Toolchain arm-none-eabi\*\bin"):
        gcc_dirs += sorted(glob.glob(pattern), reverse=True)
    for pattern in (r"C:\ST\STM32CubeIDE*\STM32CubeIDE\plugins\*make*\tools\bin\make.exe",):
        makes += glob.glob(pattern)
    g = shutil.which("arm-none-eabi-gcc")
    if g:
        gcc_dirs.append(str(Path(g).parent))
    m = shutil.which("make")
    if m:
        makes.append(m)
    gcc = next((d for d in gcc_dirs if Path(d, "arm-none-eabi-gcc.exe").is_file() or Path(d, "arm-none-eabi-gcc").is_file()), None)
    make = next((m for m in makes if Path(m).is_file()), None)
    return gcc, make


def run(cmd, cwd=None, timeout=120, env=None, log=None):
    """Run a command, return (returncode, output). Output lines are also passed to log(line)."""
    try:
        p = subprocess.Popen(cmd, cwd=cwd, env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                             stdin=subprocess.DEVNULL, creationflags=CREATE_NO_WINDOW,
                             text=True, encoding="utf-8", errors="replace")
    except OSError as e:
        return -1, str(e)
    out = []
    timer = threading.Timer(timeout, p.kill)
    timer.start()
    try:
        for line in p.stdout:
            out.append(line)
            if log:
                log(line.rstrip("\n"))
        p.wait()
    finally:
        timer.cancel()
    return p.returncode, "".join(out)


# --------------------------------------------------------------------------------------------
# Settings
# --------------------------------------------------------------------------------------------

DISPLAYS = [
    # key, define, label, description
    ("KM5S", "DISPLAY_TYPE_KINGMETER_901U", "KingMeter 5S / KM5S / 901U  (recommended)",
     "Most complete protocol in EBiCS. Use this if your display can be set to '5S' or 'KM5S'."),
    ("JLCD", "DISPLAY_TYPE_KINGMETER_618U", "KingMeter J-LCD / 618U", "Older KingMeter protocol."),
    ("BAFANG_LCD", "DISPLAY_TYPE_BAFANG_LCD", "Bafang LCD (1200 baud)", "Bafang displays incl. 'Blaupunkt'."),
    ("BAFANG_TFT", "DISPLAY_TYPE_BAFANG_850_860", "Bafang 850C / 860C (9600 baud)", "Bafang color displays."),
    ("KUNTENG", "DISPLAY_TYPE_KUNTENG", "Kunteng KT-LCD3 / LCD5 / LCD8", "KT protocol."),
    ("NO2", "DISPLAY_TYPE_NO2", "UART No.2 (S866, EN06 ...)", "Chinese 'protocol 2'. Assist level, light and walk assist only."),
    ("TONGSHENG", "DISPLAY_TYPE_TONGSHENG", "Tongsheng VLCD5 / VLCD6 / XH18  (new)",
     "Tongsheng factory protocol. New in this version, not yet tested on many displays."),
    ("VESC", "DISPLAY_TYPE_VESC", "VESC UART (115200 baud)  (new)",
     "Live data for VESC displays and apps (via a UART-BLE bridge). Not a full VESC: no VESC Tool configuration."),
    ("EBICS", "DISPLAY_TYPE_EBiCS", "EBiCS app / BLE module", "For the EBiCS Flutter app with the EBiCS BLE module."),
    ("DEBUG", "DISPLAY_TYPE_DEBUG", "No display (debug text on UART)", "Prints debug values at 56000 baud."),
]

WHEELS = [("16\"", 1270), ("20\"", 1590), ("24\"", 1900), ("26\"", 2070), ("27.5\"", 2200),
          ("28\" / 700C", 2240), ("29\"", 2290), ("custom", None)]

BATTERIES = [("24 V (7S)", 7), ("36 V (10S)", 10), ("48 V (13S)", 13), ("52 V (14S)", 14),
             ("custom (set voltages below)", 0)]

SPEED_TEMP_INPUTS = [("not used", 0), ("AD1 (PB1)", 6), ("AD2 (PB0)", 5)]

# Field: key, label, kind, default, section, unit, help
# kind: int, float, bool, str, display, battery, wheel
FIELDS = [
    # --- basic: asked in the setup wizard
    ("display", "Display protocol", "display", "KM5S", "basic", "",
     "Which protocol your display speaks. Multi-protocol displays: choose the same in the display menu."),
    ("battery", "Battery", "battery", 10, "basic", "",
     "Nominal battery voltage. Sets the low-voltage cutoff and the battery bars."),
    ("full_voltage_v", "Full charge voltage (0 = automatic)", "float", 0.0, "basic", "V",
     "Battery voltage when fully charged. Regen braking is ramped down to zero at this voltage so the battery "
     "is never overcharged. 0 = cells x 4.2 V. Must be set for a custom battery."),
    ("voltage_min_v", "Cutoff voltage (0 = automatic)", "float", 0.0, "basic", "V",
     "Below this voltage the motor stops (soft ramp just above it). 0 = cells x 3.1 V. Must be set for a custom battery."),
    ("wheel", "Wheel size", "wheel", "28\" / 700C", "basic", "", "Used for speed and the speed limit."),
    ("wheel_mm", "Wheel circumference", "int", 2240, "basic", "mm",
     "Only used when wheel size is 'custom'. Roll the bike one turn and measure."),
    ("speed_limit", "Speed limit", "int", 25, "basic", "km/h", "Assist stops above this speed."),
    ("phase_current_a", "Motor current max", "float", 15.0, "basic", "A",
     "Maximum motor phase current = motor power. Start low (10-15 A) and increase slowly."),
    ("battery_current_a", "Battery current max", "float", 15.0, "basic", "A",
     "Maximum current taken from the battery. Must not exceed what your battery/BMS allows."),
    ("pas_magnets", "Pedal sensor magnets", "int", 12, "basic", "",
     "Number of magnets/pulses per crank turn of the pedal (PAS) sensor. Usually 12 or 32."),
    ("torque_sensor", "Torque sensor", "bool", False, "basic", "",
     "Tick if you have a torque sensor instead of a plain cadence PAS sensor."),
    ("throttle", "Throttle", "bool", True, "basic", "", "Tick if a throttle is connected."),
    ("legal_mode", "Legal mode (EU)", "bool", False, "basic", "",
     "Ramps the motor down at the speed limit and limits to 6 km/h when you don't pedal."),
    ("walk_current_a", "Walk assist current", "float", 6.0, "basic", "A",
     "Motor current for the walk / push assist button."),
    ("sensorless", "Sensorless motor (no hall sensors) - experimental", "bool", False, "basic", "",
     "Run the motor without hall sensors (observer from the EBiCS Sensorless_VESC branch). Needs the motor parameters "
     "in Expert > Sensorless. Hall calibration and gearing measurement are not possible. Test carefully."),
    ("reverse", "Reverse motor direction", "bool", False, "basic", "",
     "Tick if the wheel turns backwards. The setup wizard asks you and sets this."),
    ("gear_ratio", "Pole pairs x gear ratio", "int", 60, "basic", "",
     "Motor electrical turns per wheel turn. The setup wizard measures it when you turn the wheel by hand."),
    # --- advanced
    ("speed_sensor_external", "External speed sensor", "bool", False, "advanced", "",
     "Use a spoke magnet speed sensor instead of the motor hall sensors for speed."),
    ("pulses_per_rev", "Speed sensor magnets", "int", 1, "advanced", "", "Magnets on the wheel for the external speed sensor."),
    ("speed_temp_input", "Speed/temperature wire on", "speedtemp", 0, "advanced", "",
     "Motors with one wire for speed sensor AND temperature (NTC between the signal and GND): connect that wire to an "
     "analog input (AD1 or AD2) with a pull-up resistor to 3.3 V. Not on AD1 together with 'Torque signal on AD1', "
     "not on AD2 together with 'Proportional regen on AD2'."),
    ("speed_on_wire", "  use it for speed", "bool", True, "advanced", "",
     "Speed pulses from this wire (the motor's speed sensor). Sets the speed source to external."),
    ("temp_on_wire", "  use it for motor temperature", "bool", True, "advanced", "",
     "Motor temperature from the NTC on this wire. The motor current is reduced above 'derating start'."),
    ("ntc_pullup_ohm", "  NTC pull-up resistor", "int", 3000, "advanced", "ohm",
     "Pull-up resistor from the input to 3.3 V (measure it). NTC: 10k B3950. Tune it with a 10k resistor until Live data shows 25 C."),
    ("speed_temp_threshold", "  speed pulse threshold", "int", 100, "advanced", "ADC",
     "Readings below = speed pulse, above = temperature. See the raw value in Live data: it must stay above this "
     "when hot and drop below it when the magnet passes."),
    ("motor_temp_derate", "Motor temp: derating start", "int", 100, "advanced", "\u00b0C", "Motor current is reduced from here ..."),
    ("motor_temp_max", "Motor temp: maximum", "int", 130, "advanced", "\u00b0C", "... down to zero at this temperature."),
    ("torque_on_ad1", "Torque signal on AD1", "bool", False, "advanced", "", "Torque sensor wired to AD1 instead of the throttle input."),
    ("ts_coef", "Torque sensor gain", "int", 60000, "advanced", "", "Higher = more assist per torque."),
    ("pas_timeout", "PAS stop timeout", "int", 8000, "advanced", "ticks", "Motor stops after this time without PAS pulse. 8000 = 1 s."),
    ("ramp_end", "PAS full assist at", "int", 1600, "advanced", "ticks", "PAS pulse interval for full assist (8 ticks = 1 ms)."),
    ("dirdet", "Stop on backpedalling", "bool", False, "advanced", "", "PAS direction detection (needs an asymmetric PAS sensor)."),
    ("frac_high", "Direction window max", "int", 30, "advanced", "", ""),
    ("frac_low", "Direction window min", "int", 15, "advanced", "", ""),
    ("throttle_offset", "Throttle min (ADC)", "int", 750, "advanced", "", "Default only: measured automatically at power-up."),
    ("throttle_max", "Throttle max (ADC)", "int", 2600, "advanced", "", "ADC value at full throttle (see Live data)."),
    ("speed_throttle", "Throttle sets target speed", "bool", False, "advanced", "", ""),
    ("regen_current_a", "Brake regen current", "float", 0.0, "advanced", "A", "Phase current for regenerative braking (0 = off)."),
    ("regen_max_a", "Regen max into battery", "float", 0.0, "advanced", "A", ""),
    ("linear_regen", "Proportional regen on AD2", "bool", False, "advanced", "", "Analog brake sensor on AD2."),
    ("voltage_ramp_v", "Soft voltage ramp width", "float", 1.0, "advanced", "V",
     "Current is reduced smoothly within this voltage above the cutoff (motor) and below the full charge voltage (regen)."),
    ("individual_profile", "Individual assist profile", "bool", False, "advanced", "", ""),
    # --- expert (motor control internals)
    ("pll", "Rotor position PLL", "bool", False, "expert", "", "Leave off: hall interpolation works for most motors."),
    ("p_pll", "PLL P (shift)", "int", 10, "expert", "", ""),
    ("i_pll", "PLL I (shift)", "int", 10, "expert", "", ""),
    ("spdshft", "PLL speed shift", "int", 0, "expert", "", ""),
    ("p_iq", "Iq P gain", "str", "500", "expert", "", ""),
    ("i_iq", "Iq I gain", "str", "20", "expert", "", ""),
    ("p_id", "Id P gain", "str", "500", "expert", "", ""),
    ("i_id", "Id I gain", "str", "20", "expert", "", ""),
    ("p_speed", "Speed P gain", "int", 100, "expert", "", ""),
    ("i_speed", "Speed I gain", "int", 10, "expert", "", ""),
    ("speedfilter", "Speed filter (shift)", "int", 1, "expert", "", ""),
    ("sixstep", "6-step threshold", "int", 3500, "expert", "ticks", ""),
    ("trigger_offset", "ADC trigger offset", "int", 50, "expert", "", ""),
    ("trigger_default", "ADC trigger default", "int", 2020, "expert", "", ""),
    ("timer_period", "PWM timer period", "int", 2028, "expert", "", ""),
    ("disable_dyn_adc", "Fixed ADC trigger", "bool", False, "expert", "", ""),
    ("fast_loop_log", "Fast loop logging", "bool", False, "expert", "", ""),
    ("cal_bat_v", "Battery voltage cal.", "int", 256, "expert", "", ""),
    ("cal_v", "Voltage cal. (mV/count)", "int", 25, "expert", "", ""),
    ("cal_i", "Current cal. (mA/count <<8)", "str", "38LL<<8", "expert", "", "Depends on the shunt resistor."),
    ("inductance", "Sensorless: inductance", "str", "2", "expert", "uH/100",
     "Phase inductance for the observer, e.g. 200 uH = 2. Only used in sensorless mode."),
    ("resistance", "Sensorless: resistance", "str", "120", "expert", "mOhm*512/1000",
     "Phase resistance for the observer (Shengyi 117 mOhm = 120). Only used in sensorless mode."),
    ("flux_linkage", "Sensorless: flux linkage", "str", "2400", "expert", "",
     "Tuned by hand in the branch (BionX 2400, Shengyi 1800). Only used in sensorless mode."),
    ("gamma", "Sensorless: observer gain (shift)", "str", "13", "expert", "", "Trial and error, 10..13. Only used in sensorless mode."),
    ("obs_angle_deg", "Sensorless: angle offset", "float", 20.0, "expert", "deg",
     "Offset between observer and rotor. Change in small steps until the motor runs smooth and efficient."),
    ("spec_angle", "Hall angle offset (unused)", "str", "-715827882", "expert", "", ""),
    ("autodetect_at_boot", "Autodetect at every power-up", "bool", False, "expert", "",
     "Only with the debug display. The app starts calibration over the ST-Link instead."),
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


def import_java_ini(path):
    """Import a settings file written by the old Java configurator."""
    v = Path(path).read_text(encoding="utf-8", errors="replace").splitlines()
    v = [x.strip() for x in v]
    b = lambda i: v[i].lower() == "true"
    f = lambda i: float(v[i])
    s = default_settings()
    s.update(trigger_offset=int(v[0]), trigger_default=int(v[1]), timer_period=int(v[2]), cal_bat_v=int(v[3]),
             cal_v=int(v[4]), cal_i=v[5], inductance=v[6], resistance=v[7], flux_linkage=v[8], gamma=v[9],
             p_iq=v[15], i_iq=v[16], p_id=v[17], i_id=v[18], p_pll=int(v[19]), i_pll=int(v[20]),
             p_speed=int(v[21]), i_speed=int(v[22]), spdshft=int(v[23]), speedfilter=int(v[24]), sixstep=int(v[25]),
             ts_coef=int(v[26]), pas_timeout=int(v[27]), ramp_end=int(v[28]), pas_magnets=int(v[29]),
             frac_high=int(v[30]), frac_low=int(v[31]), throttle_offset=int(v[32]), throttle_max=int(v[33]),
             walk_current_a=round(f(34) * cal_i_ma(v[5]) / 1000.0, 2), wheel="custom", wheel_mm=int(v[35]),
             gear_ratio=int(v[36]), speed_limit=int(v[37]), pulses_per_rev=max(1, int(v[38])),
             phase_current_a=f(39) / 1000.0, battery_current_a=f(40) / 1000.0, regen_current_a=f(41) / 1000.0,
             regen_max_a=f(42) / 1000.0, voltage_min_v=f(43) / 1000.0, full_voltage_v=f(44) / 1000.0, spec_angle=v[45],
             dirdet=b(46), torque_sensor=b(47), torque_on_ad1=b(48), disable_dyn_adc=b(57), fast_loop_log=b(58),
             pll=b(59), speed_throttle=b(60), linear_regen=b(61), speed_sensor_external=b(62),
             autodetect_at_boot=b(63), reverse=b(64), legal_mode=b(65), throttle=b(66), individual_profile=b(67))
    # battery cell count from the 'voltage max' setting (about 4.2 V per cell)
    cells = round(f(44) / 4200.0)
    s["battery"] = min((c for _, c in BATTERIES), key=lambda c: abs(c - cells))
    for idx, key in ((49, "JLCD"), (50, "KM5S"), (51, "EBICS"), (52, "KUNTENG"), (53, "BAFANG_LCD"),
                     (54, "BAFANG_TFT"), (55, "NO2"), (56, "DEBUG")):
        if b(idx):
            s["display"] = key
    return s


def cal_i_ma(cal_i):
    m = re.match(r"\s*(\d+)", str(cal_i))
    return int(m.group(1)) if m else 38


def wheel_circumference(s):
    for name, mm in WHEELS:
        if name == s.get("wheel") and mm:
            return mm
    return int(s.get("wheel_mm", 2240))


def battery_mv(s):
    """(cutoff_mv, full_mv, [bar1..bar5 mV]) from the cell count or the custom voltages."""
    cells = int(s.get("battery", 10))
    cutoff = int(cells * 3100)
    full = int(cells * 4200)
    if float(s.get("voltage_min_v") or 0) > 0:
        cutoff = int(float(s["voltage_min_v"]) * 1000)
    if float(s.get("full_voltage_v") or 0) > 0:
        full = int(float(s["full_voltage_v"]) * 1000)
    span = max(1, full - cutoff)
    bars = [int(cutoff + span * f) for f in (0.10, 0.25, 0.45, 0.65, 0.85)]
    return cutoff, full, bars


def check_settings(s):
    """List of problems that would make a bad firmware, empty if fine."""
    p = []
    cutoff, full, _ = battery_mv(s)
    if int(s.get("battery", 10)) == 0 and (float(s.get("voltage_min_v") or 0) <= 0 or float(s.get("full_voltage_v") or 0) <= 0):
        p.append("Custom battery: set both 'Full charge voltage' and 'Cutoff voltage'.")
    if full <= cutoff + 1000:
        p.append("'Full charge voltage' must be clearly above the 'Cutoff voltage'.")
    if float(s.get("voltage_ramp_v") or 0) * 1000 >= (full - cutoff) / 2:
        p.append("'Soft voltage ramp width' is too big for this battery voltage range.")
    idx = int(s.get("speed_temp_input") or 0)
    if idx == 6 and s.get("torque_on_ad1"):
        p.append("Speed/temperature wire and torque sensor can't both use AD1.")
    if idx == 5 and s.get("linear_regen"):
        p.append("Speed/temperature wire and proportional regen can't both use AD2.")
    if idx and not (s.get("speed_on_wire") or s.get("temp_on_wire")):
        p.append("Speed/temperature wire is selected but neither speed nor temperature is ticked.")
    return p


def generate_config_h(s):
    """Same defines and conversions as the Java configurator, plus the new display types."""
    cal_i = cal_i_ma(s["cal_i"])
    cal_v = int(s["cal_v"])
    cutoff, vmax, bars = battery_mv(s)
    display_define = next(d[1] for d in DISPLAYS if d[0] == s["display"])
    ma = lambda a: int(round(float(a) * 1000))
    units = lambda a: int(round(float(a) * 1000 / cal_i))   # amps -> internal current units

    L = ["/*", " * config.h", " *", " *  Automatically created by EBiCS Setup (setup_app/ebics_setup.py)",
         " *  Edit the settings in the app, changes here are overwritten.", " */", "",
         "#ifndef CONFIG_H_", "#define CONFIG_H_", '#include "stdint.h"',
         "#define DISPLAY_TYPE_DEBUG (1<<0)            // ASCII output in debug mode",
         "#define DISPLAY_TYPE_KUNTENG (1<<1)          // Kunteng display",
         "#define DISPLAY_TYPE_BAFANG_LCD (1<<2)       // Bafang LCD, 1200 baud",
         "#define DISPLAY_TYPE_BAFANG_850_860 (1<<3)   // Bafang 850/860, 9600 baud",
         "#define DISPLAY_TYPE_KINGMETER_618U (1<<4)   // King-Meter 618U (J-LCD)",
         "#define DISPLAY_TYPE_KINGMETER_901U (1<<5)   // King-Meter 901U (KM5S)",
         "#define DISPLAY_TYPE_EBiCS (1<<6)            // ANT+ LEV based EBiCS protocol",
         "#define DISPLAY_TYPE_NO2 (1<<7)              // China protocol 'No_2', e.g. S866",
         "#define DISPLAY_TYPE_TONGSHENG (1<<8)        // Tongsheng VLCD5 / VLCD6 / XH18",
         "#define DISPLAY_TYPE_VESC (1<<9)             // VESC UART protocol",
         "#define DISPLAY_TYPE_BAFANG         (DISPLAY_TYPE_BAFANG_LCD|DISPLAY_TYPE_BAFANG_850_860)",
         "#define DISPLAY_TYPE_KINGMETER      (DISPLAY_TYPE_KINGMETER_618U|DISPLAY_TYPE_KINGMETER_901U)",
         "#define EXTERNAL 1", "#define INTERNAL 0", ""]
    d = lambda name, val: L.append(f"#define {name} {val}")
    d("TRIGGER_OFFSET_ADC", s["trigger_offset"]); d("TRIGGER_DEFAULT", s["trigger_default"]); d("_T", s["timer_period"])
    d("CAL_BAT_V", s["cal_bat_v"]); d("CAL_V", cal_v); d("CAL_I", s["cal_i"])
    d("INDUCTANCE", s["inductance"]); d("RESISTANCE", s["resistance"]); d("FLUX_LINKAGE", s["flux_linkage"]); d("GAMMA", s["gamma"])
    for n, mv in enumerate(bars, 1):
        d(f"BATTERY_LEVEL_{n}", mv * 10)
    d("P_FACTOR_I_Q", s["p_iq"]); d("I_FACTOR_I_Q", s["i_iq"]); d("P_FACTOR_I_D", s["p_id"]); d("I_FACTOR_I_D", s["i_id"])
    d("P_FACTOR_PLL", s["p_pll"]); d("I_FACTOR_PLL", s["i_pll"]); d("P_FACTOR_SPEED", s["p_speed"]); d("I_FACTOR_SPEED", s["i_speed"])
    d("SPDSHFT", s["spdshft"]); d("SPEEDFILTER", s["speedfilter"]); d("SIXSTEPTHRESHOLD", s["sixstep"])
    d("TS_COEF", s["ts_coef"]); d("PAS_TIMEOUT", s["pas_timeout"]); d("RAMP_END", s["ramp_end"])
    d("PAS_IMP_PER_TURN", max(1, int(s["pas_magnets"])))
    d("FRAC_HIGH", s["frac_high"]); d("FRAC_LOW", s["frac_low"])
    d("THROTTLE_OFFSET", s["throttle_offset"]); d("THROTTLE_MAX", s["throttle_max"])
    d("PUSHASSIST_CURRENT", units(s["walk_current_a"]))
    d("WHEEL_CIRCUMFERENCE", wheel_circumference(s)); d("GEAR_RATIO", max(1, int(s["gear_ratio"])))
    d("SPEEDLIMIT", s["speed_limit"]); d("PULSES_PER_REVOLUTION", max(1, int(s["pulses_per_rev"])))
    d("PH_CURRENT_MAX", units(s["phase_current_a"])); d("BATTERYCURRENT_MAX", ma(s["battery_current_a"]))
    d("REGEN_CURRENT", units(s["regen_current_a"])); d("REGEN_CURRENT_MAX", ma(s["regen_max_a"]))
    d("VOLTAGE_MIN", int(cutoff / cal_v)); d("VOLTAGE_MAX", int(vmax / cal_v)); d("SPEC_ANGLE", s["spec_angle"])
    d("BATTERYVOLTAGE_MIN", cutoff); d("BATTERYVOLTAGE_MAX", vmax)
    d("VOLTAGE_RAMP_MV", max(100, int(float(s.get("voltage_ramp_v", 1.0)) * 1000)))
    d("MOTOR_TEMPERATURE_THRESHOLD", int(s.get("motor_temp_derate", 100)))
    d("MOTOR_TEMPERATURE_MAX", max(int(s.get("motor_temp_derate", 100)) + 5, int(s.get("motor_temp_max", 130))))
    st = int(s.get("speed_temp_input") or 0)
    speed_from_wire = bool(st and s.get("speed_on_wire"))
    if st:
        d("SPEED_TEMP_ADC", st); d("SPEED_TEMP_THRESHOLD", int(s.get("speed_temp_threshold", 100)))
        if speed_from_wire:
            L.append("#define SPEED_FROM_ADC")
        if s.get("temp_on_wire"):
            d("R_TEMP_PULLUP", max(1, int(s.get("ntc_pullup_ohm", 3000))))
    if s.get("sensorless"):
        L.append("#define SENSORLESS")
        d("OBS_ANGLE_OFFSET", f"{int(round(float(s.get('obs_angle_deg', 20.0)) / 180.0 * 2 ** 31)) & 0xFFFFFFFF}UL")
    flag = lambda on, text: L.append(text) if on else None
    flag(s["dirdet"], "#define DIRDET")
    flag(s["torque_sensor"], "#define TS_MODE")
    flag(s["torque_on_ad1"], "#define TQONAD1")
    d("DISPLAY_TYPE", display_define)
    flag(s["disable_dyn_adc"], "#define DISABLE_DYNAMIC_ADC")
    flag(s["fast_loop_log"], "#define FAST_LOOP_LOG")
    d("SPEED_PLL", 1 if s["pll"] else 0)
    flag(s["speed_throttle"], "#define SPEEDTHROTTLE")
    flag(s["linear_regen"], "#define ADC_BRAKE")
    d("SPEEDSOURCE", "EXTERNAL" if (s["speed_sensor_external"] or speed_from_wire) else "INTERNAL")
    d("AUTODETECT", 1 if (s["autodetect_at_boot"] and s["display"] == "DEBUG" and not s.get("sensorless")) else 0)
    d("REVERSE", -1 if s["reverse"] else 1)
    flag(s["legal_mode"], "#define LEGALFLAG")
    flag(s["throttle"], "#define THROTTLE_OVERRIDE")
    flag(s["individual_profile"], "#define INDIVIDUAL_MODES")
    L += ["", "#endif /* CONFIG_H_ */", ""]
    return "\r\n".join(L)


# --------------------------------------------------------------------------------------------
# Build
# --------------------------------------------------------------------------------------------

def build_firmware(settings, log=print):
    """Write config.h and compile. Returns True on success."""
    gcc, make = find_toolchain()
    if not gcc or not make:
        log("ERROR: ARM compiler or make not found. Install STM32CubeIDE or System Workbench (Ac6).")
        return False
    (REPO / "Inc" / "config.h").write_text(generate_config_h(settings), encoding="utf-8", newline="")
    log("config.h written")
    env = dict(os.environ)
    env["PATH"] = os.pathsep.join([gcc, str(Path(make).parent), env.get("PATH", "")])
    DEBUG_DIR.mkdir(exist_ok=True)
    run([make, "-f", "../make/makefile", "clean"], cwd=DEBUG_DIR, env=env, timeout=60)
    for f in MAKE_DIR.iterdir():
        if f.is_file():
            shutil.copy2(f, DEBUG_DIR / f.name)
    for sub in ("Drivers/STM32F1xx_HAL_Driver/Src", "startup", "Src"):
        (DEBUG_DIR / sub).mkdir(parents=True, exist_ok=True)
    shutil.copy2(DEBUG_DIR / "subdir_drivers.mk", DEBUG_DIR / "Drivers/STM32F1xx_HAL_Driver/Src/subdir.mk")
    shutil.copy2(DEBUG_DIR / "subdir_src.mk", DEBUG_DIR / "Src/subdir.mk")
    shutil.copy2(DEBUG_DIR / "subdir_startup.mk", DEBUG_DIR / "startup/subdir.mk")
    shutil.copy2(DEBUG_DIR / "STLINK_FLASH.ld", REPO / "STM32F103C6Tx_FLASH.ld")
    shutil.copy2(DEBUG_DIR / "STLINK_system_stm32f1xx.c", REPO / "Src/system_stm32f1xx.c")
    log("compiling ...")
    rc, out = run([make, "all"], cwd=DEBUG_DIR, env=env, timeout=600,
                  log=lambda l: log(l) if ("error" in l.lower() or "warning: " in l and "Src/" in l) else None)
    m = re.search(r"^\s*(\d+)\s+(\d+)\s+(\d+)\s+\d+\s+[0-9a-f]+\s+LishuiFOC_01\.elf", out, re.M)
    if rc != 0 or not HEX_FILE.is_file():
        log("ERROR: compiling failed")
        return False
    if m:
        used = int(m.group(1)) + int(m.group(2))
        log(f"firmware size: {used} of 30720 bytes")
    log("compiled OK")
    return True


def read_symbols(elf=None):
    """name -> (address, size) from the ELF of the flashed firmware."""
    elf = Path(elf or (FLASHED_ELF if FLASHED_ELF.is_file() else ELF_FILE))
    gcc, _ = find_toolchain()
    if not elf.is_file() or not gcc:
        return {}
    nm = str(Path(gcc) / "arm-none-eabi-nm")
    rc, out = run([nm, "-S", str(elf)], timeout=30)
    syms = {}
    for line in out.splitlines():
        p = line.split()
        if len(p) == 4:
            try:
                syms[p[3]] = (int(p[0], 16), int(p[1], 16))
            except ValueError:
                pass
    return syms


# --------------------------------------------------------------------------------------------
# ST-Link access through STM32_Programmer_CLI
# --------------------------------------------------------------------------------------------

class StLink:
    def __init__(self):
        self.cli = find_cube_cli()
        self.old_cli = find_stlink_cli()
        self.lock = threading.Lock()

    def _cube(self, args, timeout=60, log=None):
        if not self.cli:
            return -1, "STM32CubeProgrammer not found"
        with self.lock:
            return run([self.cli] + args, timeout=timeout, log=log)

    def probe(self):
        """dict with keys: probe, target, device, flash_kb, voltage, error"""
        rc, out = self._cube(["-c", "port=SWD", "mode=HOTPLUG"], timeout=30)
        info = {"probe": "ST-LINK SN" in out, "target": "Device name" in out and "Error" not in out}
        m = re.search(r"Device name\s*:\s*(.+)", out)
        info["device"] = m.group(1).strip() if m else ""
        m = re.search(r"Flash size\s*:\s*(\d+)", out)
        info["flash_kb"] = int(m.group(1)) if m else 0
        m = re.search(r"Voltage\s*:\s*([\d.]+)", out)
        info["voltage"] = float(m.group(1)) if m else 0.0
        m = re.search(r"Error:\s*(.+)", out)
        info["error"] = m.group(1).strip() if m else ""
        if not self.cli:
            info["error"] = "STM32CubeProgrammer is not installed"
        return info

    def read_protection(self):
        """'locked', 'unlocked' or None if unknown."""
        rc, out = self._cube(["-c", "port=SWD", "mode=HOTPLUG", "-ob", "displ"], timeout=30)
        m = re.search(r"RDP\s*:\s*0x([0-9A-Fa-f]+)", out)
        if not m:
            return None
        return "unlocked" if int(m.group(1), 16) == 0xA5 else "locked"

    def unlock(self, log=None):
        rc, out = self._cube(["-c", "port=SWD", "freq=400", "-ob", "RDP=0xA5"], timeout=120, log=log)
        return rc == 0

    def flash(self, hexfile, log=None):
        if self.cli:
            rc, out = self._cube(["-c", "port=SWD", "-w", str(hexfile), "-v", "-rst"], timeout=180, log=log)
            return rc == 0
        if self.old_cli:
            with self.lock:
                rc, out = run([self.old_cli, "-c", "SWD", "-P", str(hexfile), "-V", "-Rst"], timeout=180, log=log)
            return rc == 0 and "Error" not in out
        if log:
            log("ERROR: neither STM32CubeProgrammer nor ST-LINK Utility found")
        return False

    def read(self, regions):
        """regions: list of (address, size). Returns {address: byte} or None if no connection."""
        args = ["-c", "port=SWD", "mode=HOTPLUG"]
        for addr, size in regions:
            args += ["-r8", hex(addr), str(size)]
        rc, out = self._cube(args, timeout=30)
        if "Error" in out and "0x2000" not in out:
            return None
        mem = {}
        for line in out.splitlines():
            m = re.match(r"\s*0x([0-9A-Fa-f]{8})\s*:\s*((?:[0-9A-Fa-f]{2}\s*)+)$", line)
            if m:
                a = int(m.group(1), 16)
                for i, bt in enumerate(m.group(2).split()):
                    mem[a + i] = int(bt, 16)
        return mem

    def write8(self, addr, value):
        rc, out = self._cube(["-c", "port=SWD", "mode=HOTPLUG", "-w8", hex(addr), hex(value)], timeout=30)
        return rc == 0 and "Error" not in out


# --------------------------------------------------------------------------------------------
# Live values
# --------------------------------------------------------------------------------------------

LIVE_VARS = {
    # name: (type, offset inside the symbol, label)
    "ui32_setup_magic": ("u32", 0),
    "ui8_setup_state": ("u8", 0),
    "ui8_setup_progress": ("u8", 0),
    "ui8_setup_halls_found": ("u8", 0),
    "ui8_hall_calibrated": ("u8", 0),
    "ui32_hall_events": ("u32", 0),
    "ui32_pas_events": ("u32", 0),
    "brake_flag": ("u8", 0),
    "ui16_throttle": ("u16", 0),
    "ui16_throttle_offset": ("u16", 0),
    "uint16_full_rotation_counter": ("u16", 0),
    "i16_hall_order": ("i16", 0),
    "ui32_KV": ("i16", 0),
    "ui8_watchdog_reset": ("u8", 0),
    "ui16_motor_temp_adc": ("u16", 0),
    "ui8_motor_temp_sensor_ok": ("u8", 0),
    "ui8_sensorless_running": ("u8", 0),
}
MS_FIELDS = {"Voltage": ("i32", 0), "Speed": ("u32", 4), "i_q": ("i32", 12), "i_q_setpoint": ("i32", 16),
             "u_abs": ("i32", 44), "Battery_Current": ("i32", 48), "assist_level": ("u8", 54),
             "Temperature": ("i16", 56), "int_Temperature": ("i16", 58)}
HALLS = ["Hall_45", "Hall_51", "Hall_13", "Hall_32", "Hall_26", "Hall_64"]

_SIZES = {"u8": 1, "u16": 2, "i16": 2, "u32": 4, "i32": 4}


def _decode(mem, addr, typ):
    n = _SIZES[typ]
    try:
        raw = bytes(mem[addr + i] for i in range(n))
    except KeyError:
        return None
    return int.from_bytes(raw, "little", signed=typ.startswith("i"))


class Live:
    """Reads the controller's variables over the ST-Link."""

    def __init__(self, stlink):
        self.st = stlink
        self.syms = {}

    def load_symbols(self):
        self.syms = read_symbols()
        return bool(self.syms)

    def snapshot(self, with_halls=False):
        if not self.syms and not self.load_symbols():
            return None
        regions, plan = [], []
        for name, (typ, off) in LIVE_VARS.items():
            if name in self.syms:
                a = self.syms[name][0] + off
                regions.append((a, _SIZES[typ])); plan.append((name, a, typ))
        if "MS" in self.syms:
            base = self.syms["MS"][0]
            regions.append((base, 64))
            for f, (typ, off) in MS_FIELDS.items():
                plan.append(("MS." + f, base + off, typ))
        if with_halls:
            for h in HALLS:
                if h in self.syms:
                    regions.append((self.syms[h][0], 4)); plan.append((h, self.syms[h][0], "u32"))
        if "adcData" in self.syms:
            base = self.syms["adcData"][0]
            regions.append((base, 18))
            for n in range(9):
                plan.append((f"adc{n}", base + 2 * n, "u16"))
        mem = self.st.read(regions)
        if mem is None:
            return None
        return {name: _decode(mem, a, typ) for name, a, typ in plan}

    def start_calibration(self):
        if "ui8_setup_cmd" not in self.syms:
            return False
        return self.st.write8(self.syms["ui8_setup_cmd"][0], 1)


def speed_kmh(snap, settings):
    """Wheel speed from the motor hall timing (internal speed source)."""
    if not snap or snap.get("uint16_full_rotation_counter", 8000) >= 8000:
        return 0.0
    tics = snap.get("MS.Speed") or 0
    if tics <= 0:
        return 0.0
    t_wheel = tics * 6 * max(1, int(settings["gear_ratio"])) / 500000.0
    return wheel_circumference(settings) / 1000.0 / t_wheel * 3.6 if t_wheel > 0 else 0.0


def hall_angles_deg(snap):
    """The six hall angles in degrees (0..360) in rotation order, or None."""
    vals = [snap.get(h) for h in HALLS]
    if None in vals:
        return None
    return [v * 360.0 / 2 ** 32 for v in vals]


def check_hall_angles(angles):
    """True if the six angles are roughly 60 degrees apart."""
    if not angles:
        return False
    a = sorted(angles)
    gaps = [(a[(i + 1) % 6] - a[i]) % 360 for i in range(6)]
    return all(35 <= g <= 85 for g in gaps)
