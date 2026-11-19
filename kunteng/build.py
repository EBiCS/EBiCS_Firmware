#!/usr/bin/env python3
"""Build the Kunteng (KT, STM8S105) firmware with SDCC.

Usage: python build.py [--out DIR] [-DNAME=VALUE ...]
Produces main.ihx (Intel hex) in the output directory. Needs SDCC 4.x (sdcc on PATH or in the default
install folder). Extra -D defines are passed to every compile, the setup app uses that to inject settings.
"""
import os
import shutil
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))

SPL = ["itc", "clk", "iwdg", "gpio", "exti", "uart2", "tim1", "tim2", "adc1", "flash"]
APP = ["watchdog", "gpio", "utils", "uart", "adc", "brake", "pas", "timers", "pwm", "eeprom",
       "motor", "ebike_app", "wheel_speed_sensor", "settings", "display"]

CFLAGS = ["-mstm8", "--std-c99", "--opt-code-size", "-Ddouble=float"]


def find_sdcc():
    for p in [shutil.which("sdcc"),
              r"C:\Program Files\SDCC\bin\sdcc.exe",
              r"C:\Program Files (x86)\SDCC\bin\sdcc.exe",
              "/usr/bin/sdcc", "/usr/local/bin/sdcc"]:
        if p and os.path.isfile(p):
            return p
    return None


def build(out_dir=None, defines=(), log=print):
    sdcc = find_sdcc()
    if not sdcc:
        raise RuntimeError("SDCC not found. Install it from https://sdcc.sourceforge.net/ (or: winget install SDCC.SDCC)")
    env = dict(os.environ)
    env["PATH"] = os.path.dirname(sdcc) + os.pathsep + env.get("PATH", "")  # sdcpp needs cc1 next to it
    out_dir = os.path.abspath(out_dir or os.path.join(HERE, "build"))
    os.makedirs(out_dir, exist_ok=True)
    inc = ["-I" + os.path.join(HERE, "StdPeriphLib", "inc"), "-I" + HERE]
    defs = [d if d.startswith("-D") else "-D" + d for d in defines]
    srcs = [os.path.join(HERE, "StdPeriphLib", "src", "stm8s_%s.c" % s) for s in SPL] + \
           [os.path.join(HERE, a + ".c") for a in APP]
    rels = []
    for src in srcs:
        rel = os.path.join(out_dir, os.path.splitext(os.path.basename(src))[0] + ".rel")
        cmd = [sdcc, "-c"] + CFLAGS + inc + defs + ["-o", rel, src]
        r = subprocess.run(cmd, capture_output=True, text=True, env=env)
        msg = (r.stdout + r.stderr).strip()
        if r.returncode:
            raise RuntimeError("compile failed: %s\n%s" % (os.path.basename(src), msg))
        if msg:
            log(msg)
        rels.append(rel)
    ihx = os.path.join(out_dir, "main.ihx")
    cmd = [sdcc] + CFLAGS + inc + defs + ["--out-fmt-ihx", "-o", ihx, os.path.join(HERE, "main.c")] + rels
    r = subprocess.run(cmd, capture_output=True, text=True, env=env)
    msg = (r.stdout + r.stderr).strip()
    if r.returncode:
        raise RuntimeError("link failed\n" + msg)
    if msg:
        log(msg)
    size = flash_size(ihx)
    log("Kunteng firmware built: %s (%d bytes of 32768 flash)" % (ihx, size))
    if size > 32768:
        raise RuntimeError("firmware does not fit into the 32 KB flash of the STM8S105")
    return ihx


def flash_size(ihx):
    """Highest flash byte used (STM8 flash starts at 0x8000)."""
    top = 0
    with open(ihx) as f:
        for line in f:
            if not line.startswith(":") or line[7:9] != "00":
                continue
            n = int(line[1:3], 16)
            addr = int(line[3:7], 16)
            if addr >= 0x8000:
                top = max(top, addr + n - 0x8000)
    return top


if __name__ == "__main__":
    args = sys.argv[1:]
    out = None
    if "--out" in args:
        i = args.index("--out")
        out = args[i + 1]
        del args[i:i + 2]
    try:
        build(out, [a[2:] for a in args if a.startswith("-D")])
    except RuntimeError as e:
        print(e)
        sys.exit(1)
