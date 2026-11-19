"""
Build the prebuilt firmware variants for the EBiCS Setup Android app.

One firmware per display protocol, each for hall sensor motors and for sensorless motors. The numeric
bike settings are NOT compiled in: the app writes them into the runtime settings block (Inc/settings.h).
For every variant this writes:

    <out>/<name>.bin        firmware image from 0x08000000 (without the settings block)
    <out>/<name>.sym.json   addresses of the variables the app reads/writes over SWD
    <out>/manifest.json     list of variants, flash layout, firmware version
    <out>/test_vectors.json settings -> expected settings block (checked by the app's unit tests)

Plus the Kunteng (KT, STM8) firmware: one build, all settings at runtime (EEPROM settings block):

    <out>/kunteng.ihx                Intel hex for the STM8S105 (needs SDCC)
    <out>/kunteng_test_vectors.json  KT settings -> expected settings block

Usage:  python setup_app/build_variants.py [--out android_app/assets/firmware] [--only km5s_hall ...]
        python setup_app/build_variants.py --kunteng-only
Needs arm-none-eabi-gcc and make (see ebics_core.find_toolchain), works on Windows and Linux (CI).

Part of EBiCS, GPL v3 or later.
"""

import argparse
import hashlib
import json
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import ebics_core as core  # noqa: E402
import kunteng_core as kt  # noqa: E402

SYMBOLS = (list(core.LIVE_VARS) + core.HALLS +
           ["MS", "adcData", "ui8_setup_cmd", "RS", "ui8_settings_source", "ui8_motor_temp_sensor_ok",
            "ebics_settings_flash"])
TREE = ["Src", "Inc", "Drivers", "Startup", "make", "STM32F103C6Tx_FLASH.ld"]
FLASH_BASE = 0x08000000
EEPROM_START = 0x08007800


def variant_list():
    out = []
    for key, _define, label, _desc in core.DISPLAYS:
        for sensorless in (False, True):
            out.append({"name": f"{key.lower()}_{'sensorless' if sensorless else 'hall'}", "display": key,
                        "display_label": label, "sensorless": sensorless})
    return out


def git_version():
    try:
        return subprocess.run(["git", "describe", "--always", "--dirty", "--tags"], cwd=core.REPO,
                              capture_output=True, text=True, timeout=20).stdout.strip() or "unknown"
    except OSError:
        return "unknown"


def test_vectors():
    cases = []
    base = core.default_settings()
    cases.append(("defaults", dict(base)))
    s = dict(base)
    s.update(battery=0, full_voltage_v=58.8, voltage_min_v=44.0, wheel="custom", wheel_mm=2155, speed_limit=32,
             phase_current_a=22.5, battery_current_a=18, pas_magnets=32, throttle=False, legal_mode=True,
             reverse=True, gear_ratio=88, regen_current_a=5, regen_max_a=3.5, voltage_ramp_v=1.5)
    cases.append(("custom_battery_legal_reverse", s))
    s = dict(base)
    s.update(sensorless=True, speed_temp_input=5, speed_on_wire=True, temp_on_wire=True, ntc_pullup_ohm=4700,
             speed_temp_threshold=150, motor_temp_derate=90, motor_temp_max=120, torque_sensor=True,
             torque_on_ad1=True, dirdet=True, obs_angle_deg=-12.5, inductance="3", gamma="11", battery=13)
    cases.append(("sensorless_wire_torque", s))
    return [{"name": n, "settings": st, "block_hex": core.settings_block(st).hex()} for n, st in cases]


def kunteng_test_vectors():
    cases = [("defaults", kt.default_settings())]
    s = kt.default_settings()
    s.update(controller=2, cells=13, motor_current_a=22, regen_current_a=6, throttle_type=2, direct_drive=True,
             pas_magnets=12, pas_direction=1, pas_max_cadence=75, assist_1=0.125, assist_2=0.333, assist_3=0.5,
             assist_4=0.875, assist_5=0.95, over_voltage_protection=True, rotor_offset_angle=190,
             foc_angle_adjust=140, erps_interpolation=20, max_speed=32, wheel=0x18, phase_current_limit=False,
             hall_learning=False, display=1, ramp_s=1.25, motor_characteristic=88, foc_delay_us=-42)
    cases.append(("everything_changed", s))
    return [{"name": n, "settings": st, "block_hex": kt.settings_block(st).hex()} for n, st in cases]


def build_kunteng(out, manifest):
    """KT firmware for the app. Returns True on success (False also when SDCC is missing)."""
    print("=== kunteng (STM8)", flush=True)
    if not kt.find_compiler():
        print("   ERROR: SDCC not found (apt install sdcc / winget install SDCC.SDCC)")
        return False
    with tempfile.TemporaryDirectory(prefix="kt_build_") as tmp:
        try:
            ihx = kt.fw_build.build(tmp, [], log=lambda l: print("   " + l))
        except RuntimeError as e:
            print("   ERROR: %s" % e)
            return False
        data = Path(ihx).read_text()
        import kt_live
        syms = kt_live.live_symbols(kt_live.read_map(str(Path(tmp) / "main.map")))
        if len(syms) != len(kt_live.LIVE_VARS):
            print("   ERROR: live data symbols missing in the map file")
            return False
    (out / "kunteng.ihx").write_text(data)
    (out / "kunteng.sym.json").write_text(json.dumps(syms, indent=1, sort_keys=True))
    manifest["kunteng"] = {"file": "kunteng.ihx", "symbols": "kunteng.sym.json",
                           "sha256": hashlib.sha256(data.encode()).hexdigest(),
                           "settings_block": {"address": kt.SETTINGS_BLOCK_ADDR, "size": kt.SETTINGS_BLOCK_LEN,
                                              "version": kt.SETTINGS_BLOCK_VERSION}}
    (out / "kunteng_test_vectors.json").write_text(json.dumps(kunteng_test_vectors(), indent=1))
    return True


def read_symbols(elf, gcc_dir):
    nm = str(Path(gcc_dir) / "arm-none-eabi-nm")
    out = subprocess.run([nm, "-S", str(elf)], capture_output=True, text=True, timeout=60).stdout
    syms = {}
    for line in out.splitlines():
        p = line.split()
        if len(p) == 4 and p[3] in SYMBOLS:
            syms[p[3]] = [int(p[0], 16), int(p[1], 16)]
    return syms


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=str(core.REPO / "android_app" / "assets" / "firmware"))
    ap.add_argument("--only", nargs="*", help="variant names, e.g. km5s_hall")
    ap.add_argument("--kunteng-only", action="store_true", help="only the Kunteng (STM8) firmware")
    args = ap.parse_args()
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    if args.kunteng_only:
        mpath = out / "manifest.json"
        manifest = json.loads(mpath.read_text()) if mpath.is_file() else {"format": 1, "variants": []}
        ok = build_kunteng(out, manifest)
        mpath.write_text(json.dumps(manifest, indent=1))
        return 0 if ok else 1
    gcc, make = core.find_toolchain()
    if not gcc or not make:
        print("ERROR: arm-none-eabi-gcc or make not found")
        return 1

    variants = [v for v in variant_list() if not args.only or v["name"] in args.only]
    manifest = {"format": 1, "firmware_version": git_version(), "flash_base": FLASH_BASE,
                "settings_block": {"address": core.SETTINGS_BLOCK_ADDR, "size": core.SETTINGS_BLOCK_SIZE,
                                   "magic": core.SETTINGS_MAGIC, "version": core.SETTINGS_VERSION},
                "eeprom_start": EEPROM_START, "setup_magic": core.SETUP_MAGIC, "variants": []}
    failed = []
    with tempfile.TemporaryDirectory(prefix="ebics_variants_") as tmp:
        root = Path(tmp) / "src"
        root.mkdir()
        for item in TREE:
            src = core.REPO / item
            if src.is_dir():
                shutil.copytree(src, root / item)
            else:
                shutil.copy2(src, root / item)
        for v in variants:
            print(f"=== {v['name']}", flush=True)
            s = core.default_settings()
            s.update(display=v["display"], sensorless=v["sensorless"])
            if not core.build_firmware(s, log=lambda l: print("   " + l), root=root):
                failed.append(v["name"])
                continue
            dbg = root / "Debug"
            data = (dbg / "LishuiFOC_01.bin").read_bytes()
            syms = read_symbols(dbg / "LishuiFOC_01.elf", gcc)
            problems = []
            if FLASH_BASE + len(data) > core.SETTINGS_BLOCK_ADDR:
                problems.append(f"image too big: {len(data)} bytes")
            if syms.get("ebics_settings_flash", [0])[0] != core.SETTINGS_BLOCK_ADDR:
                problems.append("settings block not at its fixed address")
            missing = [n for n in ("ui32_setup_magic", "ui8_setup_cmd", "ui8_setup_state", "MS", "RS") if n not in syms]
            if missing:
                problems.append("missing symbols: " + ", ".join(missing))
            if problems:
                print("   ERROR: " + "; ".join(problems))
                failed.append(v["name"])
                continue
            (out / f"{v['name']}.bin").write_bytes(data)
            (out / f"{v['name']}.sym.json").write_text(json.dumps(syms, indent=1, sort_keys=True))
            manifest["variants"].append(dict(v, file=f"{v['name']}.bin", symbols=f"{v['name']}.sym.json",
                                             size=len(data), sha256=hashlib.sha256(data).hexdigest()))
            print(f"   {len(data)} bytes, free {core.SETTINGS_BLOCK_ADDR - FLASH_BASE - len(data)}")
    if not build_kunteng(out, manifest):
        failed.append("kunteng")
    (out / "manifest.json").write_text(json.dumps(manifest, indent=1))
    (out / "test_vectors.json").write_text(json.dumps(test_vectors(), indent=1))
    if failed:
        print("FAILED: " + ", ".join(failed))
        return 1
    print(f"OK: {len(manifest['variants'])} variants in {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
