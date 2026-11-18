"""
EBiCS Setup - simple setup app for Lishui controllers running EBiCS.

Start with "EBiCS Setup.bat" in the firmware folder, or: pythonw setup_app/ebics_setup.py

Part of EBiCS, GPL v3 or later.
"""

import queue
import shutil
import sys
import threading
import time
import tkinter as tk
from pathlib import Path
from tkinter import ttk, messagebox, filedialog

sys.path.insert(0, str(Path(__file__).resolve().parent))
import ebics_core as core  # noqa: E402

BG = "#f5f6f8"
SIDEBAR = "#1f2a37"
SIDEBAR_ACTIVE = "#33475b"
ACCENT = "#1f6feb"
OK = "#1a7f37"
BAD = "#c62828"
WARN = "#b26a00"
MUTED = "#5b6573"
SCALE = 1.0


def px(n):
    """Pixels scaled for the screen DPI (fonts scale by themselves)."""
    return int(n * SCALE)


# ============================================================================================
# helpers
# ============================================================================================

class LogBox(ttk.Frame):
    def __init__(self, master, height=10):
        super().__init__(master)
        self.text = tk.Text(self, height=height, wrap="word", font=("Consolas", 9), bg="#0d1117", fg="#d6deeb",
                            relief="flat", padx=8, pady=6, state="disabled")
        sb = ttk.Scrollbar(self, command=self.text.yview)
        self.text.configure(yscrollcommand=sb.set)
        self.text.pack(side="left", fill="both", expand=True)
        sb.pack(side="right", fill="y")

    def clear(self):
        self.text.configure(state="normal")
        self.text.delete("1.0", "end")
        self.text.configure(state="disabled")

    def add(self, line):
        self.text.configure(state="normal")
        self.text.insert("end", line + "\n")
        self.text.see("end")
        self.text.configure(state="disabled")


class Tooltip:
    """Hover help. Only one tooltip exists at a time; it closes as soon as the mouse is no longer over its
    widget (checked every 150 ms, because <Leave> is not delivered reliably, e.g. when a page changes)."""
    _tip = None      # the one tooltip window
    _owner = None    # Tooltip object that currently shows it

    def __init__(self, widget, text):
        self.widget, self.text, self._pending = widget, text, None
        if text:
            widget.bind("<Enter>", self._enter, add="+")
            widget.bind("<Leave>", self._leave, add="+")
            widget.bind("<ButtonPress>", self._leave, add="+")

    def _enter(self, _e=None):
        self._cancel()
        self._pending = self.widget.after(350, self._show)

    def _leave(self, _e=None):
        self._cancel()
        if Tooltip._owner is self:
            Tooltip.close()

    def _cancel(self):
        if self._pending:
            try:
                self.widget.after_cancel(self._pending)
            except tk.TclError:
                pass
            self._pending = None

    def _pointer_inside(self):
        try:
            x, y = self.widget.winfo_pointerxy()
            w = self.widget.winfo_containing(x, y)
        except (tk.TclError, KeyError):
            return False
        while w is not None:
            if w == self.widget:
                return True
            w = w.master
        return False

    def _show(self):
        self._pending = None
        if not self._pointer_inside():
            return
        Tooltip.close()
        x = self.widget.winfo_rootx() + px(20)
        y = self.widget.winfo_rooty() + self.widget.winfo_height() + px(4)
        tip = tk.Toplevel(self.widget)
        tip.wm_overrideredirect(True)
        tip.attributes("-topmost", True)
        tip.geometry(f"+{x}+{y}")
        tk.Label(tip, text=self.text, bg="#fffbe6", relief="solid", borderwidth=1, wraplength=px(360),
                 justify="left", padx=6, pady=4, font=("Segoe UI", 9)).pack()
        Tooltip._tip, Tooltip._owner = tip, self
        self._watch()

    def _watch(self):
        if Tooltip._owner is not self:
            return
        if not self._pointer_inside():
            Tooltip.close()
            return
        self.widget.after(150, self._watch)

    @classmethod
    def close(cls):
        if cls._tip is not None:
            try:
                cls._tip.destroy()
            except tk.TclError:
                pass
        cls._tip, cls._owner = None, None


def field_widget(parent, app, key, var_store):
    """Create the input widget for a settings field, returns (widget, variable)."""
    _, label, kind, default, section, unit, helptext = core.FIELD_BY_KEY[key]
    val = app.settings.get(key, default)
    if kind == "bool":
        var = tk.BooleanVar(value=bool(val))
        w = ttk.Checkbutton(parent, variable=var)
    elif kind == "display":
        labels = [d[2] for d in core.DISPLAYS]
        cur = next((d[2] for d in core.DISPLAYS if d[0] == val), labels[0])
        var = tk.StringVar(value=cur)
        w = ttk.Combobox(parent, textvariable=var, values=labels, state="readonly", width=44)
    elif kind == "battery":
        labels = [b[0] for b in core.BATTERIES]
        cur = next((b[0] for b in core.BATTERIES if b[1] == int(val)), labels[1])
        var = tk.StringVar(value=cur)
        w = ttk.Combobox(parent, textvariable=var, values=labels, state="readonly", width=16)
    elif kind == "speedtemp":
        labels = [x[0] for x in core.SPEED_TEMP_INPUTS]
        cur = next((x[0] for x in core.SPEED_TEMP_INPUTS if x[1] == int(val or 0)), labels[0])
        var = tk.StringVar(value=cur)
        w = ttk.Combobox(parent, textvariable=var, values=labels, state="readonly", width=16)
    elif kind == "wheel":
        labels = [x[0] for x in core.WHEELS]
        var = tk.StringVar(value=val if val in labels else labels[-1])
        w = ttk.Combobox(parent, textvariable=var, values=labels, state="readonly", width=16)
    else:
        var = tk.StringVar(value=str(val))
        w = ttk.Entry(parent, textvariable=var, width=18)
    var_store[key] = var
    return w, var


def read_fields(var_store, settings):
    """Copy widget values into settings. Returns error text or None."""
    new = dict(settings)
    for key, var in var_store.items():
        _, label, kind, default, *_ = core.FIELD_BY_KEY[key]
        v = var.get()
        try:
            if kind == "bool":
                new[key] = bool(v)
            elif kind == "display":
                new[key] = next(d[0] for d in core.DISPLAYS if d[2] == v)
            elif kind == "battery":
                new[key] = next(b[1] for b in core.BATTERIES if b[0] == v)
            elif kind == "speedtemp":
                new[key] = next(x[1] for x in core.SPEED_TEMP_INPUTS if x[0] == v)
            elif kind == "wheel":
                new[key] = v
            elif kind == "int":
                new[key] = int(float(str(v).strip()))
            elif kind == "float":
                new[key] = float(str(v).strip().replace(",", "."))
            else:
                new[key] = str(v).strip()
        except (ValueError, StopIteration):
            return f"'{label}' has an invalid value: {v}"
    problems = core.check_settings(new)
    if problems:
        return "\n".join(problems)
    settings.update(new)
    return None


# ============================================================================================
# main window
# ============================================================================================

class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("EBiCS Setup")
        global SCALE
        SCALE = max(1.0, self.winfo_fpixels("1i") / 96.0)
        self.geometry(f"{px(1080)}x{px(720)}")
        self.minsize(px(900), px(620))
        self.configure(bg=BG)
        self._style()
        self.settings = core.load_settings()
        self.stlink = core.StLink()
        self.live = core.Live(self.stlink)
        self.q = queue.Queue()
        self.after(40, self._pump)

        self.sidebar = tk.Frame(self, bg=SIDEBAR, width=px(210))
        self.sidebar.pack(side="left", fill="y")
        self.sidebar.pack_propagate(False)
        tk.Label(self.sidebar, text="EBiCS", font=("Segoe UI Semibold", 20), fg="white", bg=SIDEBAR).pack(pady=(22, 0))
        tk.Label(self.sidebar, text="Lishui controller setup", font=("Segoe UI", 9), fg="#9fb0c2", bg=SIDEBAR).pack(pady=(0, 20))

        self.content = ttk.Frame(self, style="Page.TFrame")
        self.content.pack(side="left", fill="both", expand=True)
        self.pages, self.nav = {}, {}
        for key, text, cls in (("home", "Home", HomePage), ("wizard", "First setup", WizardPage),
                               ("settings", "Settings", SettingsPage), ("flash", "Install firmware", FlashPage),
                               ("live", "Live data", LivePage), ("tools", "Tools", ToolsPage)):
            b = tk.Button(self.sidebar, text=("  " + text), anchor="w", relief="flat", bd=0, padx=18, pady=10,
                          font=("Segoe UI", 11), fg="white", bg=SIDEBAR, activebackground=SIDEBAR_ACTIVE,
                          activeforeground="white", cursor="hand2", command=lambda k=key: self.show(k))
            b.pack(fill="x")
            self.nav[key] = b
            self.pages[key] = cls(self.content, self)
        tk.Label(self.sidebar, text="Keep the controller powered\nfrom the battery while connected.",
                 font=("Segoe UI", 8), fg="#9fb0c2", bg=SIDEBAR, justify="left").pack(side="bottom", pady=14)
        self.current = None
        self.show("home")
        self.protocol("WM_DELETE_WINDOW", self._close)

    def _style(self):
        s = ttk.Style(self)
        s.theme_use("clam")
        s.configure(".", font=("Segoe UI", 10), background=BG)
        s.configure("Page.TFrame", background=BG)
        s.configure("Card.TFrame", background="white", relief="solid", borderwidth=1)
        s.configure("TLabel", background=BG)
        s.configure("Card.TLabel", background="white")
        s.configure("H1.TLabel", font=("Segoe UI Semibold", 20), background=BG)
        s.configure("H2.TLabel", font=("Segoe UI Semibold", 13), background=BG)
        s.configure("Muted.TLabel", foreground=MUTED, background=BG)
        s.configure("Big.TLabel", font=("Segoe UI", 12), background=BG)
        s.configure("Value.TLabel", font=("Segoe UI Semibold", 18), background="white")
        s.configure("TCheckbutton", background=BG, indicatorsize=px(15), indicatormargin=(2, 2, px(8), 2))
        s.configure("TRadiobutton", background=BG, font=("Segoe UI", 11), indicatorsize=px(15), indicatormargin=(2, 2, px(8), 2))
        s.configure("Card.TCheckbutton", background="white")
        s.configure("Accent.TButton", font=("Segoe UI Semibold", 11), foreground="white", background=ACCENT,
                    padding=(16, 9), borderwidth=0)
        s.map("Accent.TButton", background=[("active", "#1758c7"), ("disabled", "#9db7e4")])
        s.configure("TButton", padding=(12, 7))
        s.configure("Horizontal.TProgressbar", thickness=18)

    def show(self, key):
        Tooltip.close()
        if self.current:
            self.pages[self.current].pack_forget()
            self.pages[self.current].on_hide()
            self.nav[self.current].configure(bg=SIDEBAR)
        self.current = key
        self.nav[key].configure(bg=SIDEBAR_ACTIVE)
        self.pages[key].pack(fill="both", expand=True)
        self.pages[key].on_show()

    # --- threading: run fn in background, call done(result) in the GUI thread
    def bg(self, fn, done=None):
        def worker():
            try:
                r = fn()
            except Exception as e:  # report instead of dying silently
                r = e
            if done:
                self.q.put(lambda: done(r))
        threading.Thread(target=worker, daemon=True).start()

    def ui(self, fn):
        self.q.put(fn)

    def _pump(self):
        while True:
            try:
                fn = self.q.get_nowait()
            except queue.Empty:
                break
            try:
                fn()
            except tk.TclError:
                pass  # the page was left before a background job finished: its widgets are gone
        self.after(40, self._pump)

    def save(self):
        core.save_settings(self.settings)

    def _close(self):
        self.save()
        self.destroy()


class Page(ttk.Frame):
    def __init__(self, master, app):
        super().__init__(master, style="Page.TFrame", padding=(28, 22))
        self.app = app

    def on_show(self):
        pass

    def on_hide(self):
        pass


# ============================================================================================
# home
# ============================================================================================

class HomePage(Page):
    def __init__(self, master, app):
        super().__init__(master, app)
        ttk.Label(self, text="Welcome", style="H1.TLabel").pack(anchor="w")
        ttk.Label(self, text="This app sets up your Lishui controller with the open source EBiCS firmware:\n"
                             "unlock the chip, install the firmware, calibrate the motor and test everything.",
                  style="Big.TLabel").pack(anchor="w", pady=(4, 18))
        ttk.Button(self, text="Start first setup  →", style="Accent.TButton",
                   command=lambda: app.show("wizard")).pack(anchor="w")
        ttk.Label(self, text="Already set up? Use Settings to change something and Install firmware to update.",
                  style="Muted.TLabel").pack(anchor="w", pady=(8, 24))
        ttk.Label(self, text="What this PC has", style="H2.TLabel").pack(anchor="w")
        self.status = ttk.Frame(self, style="Page.TFrame")
        self.status.pack(anchor="w", pady=6)

    def on_show(self):
        for w in self.status.winfo_children():
            w.destroy()
        gcc, make = core.find_toolchain()
        rows = [("STM32CubeProgrammer (unlock, flash, live data)", bool(self.app.stlink.cli),
                 "Install STM32CubeProgrammer from st.com"),
                ("ARM compiler", bool(gcc), "Install STM32CubeIDE or System Workbench (Ac6)"),
                ("make", bool(make), "Comes with STM32CubeIDE / System Workbench")]
        for text, ok, fix in rows:
            r = ttk.Frame(self.status, style="Page.TFrame")
            r.pack(anchor="w", pady=2)
            tk.Label(r, text="✔" if ok else "✖", fg=OK if ok else BAD, bg=BG, font=("Segoe UI", 12)).pack(side="left")
            ttk.Label(r, text="  " + text + ("" if ok else f"  -  {fix}")).pack(side="left")


# ============================================================================================
# settings
# ============================================================================================

class SettingsPage(Page):
    def __init__(self, master, app):
        super().__init__(master, app)
        top = ttk.Frame(self, style="Page.TFrame")
        top.pack(fill="x")
        ttk.Label(top, text="Settings", style="H1.TLabel").pack(side="left")
        ttk.Button(top, text="Save + install firmware", style="Accent.TButton", command=self.save_install).pack(side="right")
        ttk.Button(top, text="Save", command=self.save).pack(side="right", padx=8)
        ttk.Label(self, text="Hover over a setting for an explanation. Changes take effect after installing the firmware.",
                  style="Muted.TLabel").pack(anchor="w", pady=(2, 10))
        self.nb = ttk.Notebook(self)
        self.nb.pack(fill="both", expand=True)
        bar = ttk.Frame(self, style="Page.TFrame")
        bar.pack(fill="x", pady=(10, 0))
        ttk.Button(bar, text="Import old Java settings (.ini)", command=self.import_ini).pack(side="left")
        ttk.Button(bar, text="Export profile", command=self.export).pack(side="left", padx=8)
        ttk.Button(bar, text="Load profile", command=self.load).pack(side="left")
        ttk.Button(bar, text="Reset to defaults", command=self.reset).pack(side="right")
        self.vars = {}

    def on_show(self):
        self.build_tabs()

    def build_tabs(self):
        for t in self.nb.tabs():
            self.nb.forget(t)
        self.vars = {}
        for section, title in (("basic", "Basic"), ("advanced", "Advanced"), ("expert", "Expert (motor control)")):
            outer = ttk.Frame(self.nb, style="Page.TFrame")
            canvas = tk.Canvas(outer, bg=BG, highlightthickness=0)
            sb = ttk.Scrollbar(outer, orient="vertical", command=canvas.yview)
            frame = ttk.Frame(canvas, style="Page.TFrame", padding=12)
            frame.bind("<Configure>", lambda e, c=canvas: c.configure(scrollregion=c.bbox("all")))
            canvas.create_window((0, 0), window=frame, anchor="nw")
            canvas.configure(yscrollcommand=sb.set)
            canvas.pack(side="left", fill="both", expand=True)
            sb.pack(side="right", fill="y")
            canvas.bind("<Enter>", lambda e, c=canvas: c.bind_all("<MouseWheel>", lambda ev: c.yview_scroll(-int(ev.delta / 120), "units")))
            canvas.bind("<Leave>", lambda e, c=canvas: c.unbind_all("<MouseWheel>"))
            if section == "expert":
                ttk.Label(frame, text="Only change these if you know what they do. Wrong values can damage the motor or controller.",
                          foreground=WARN).grid(row=0, column=0, columnspan=3, sticky="w", pady=(0, 8))
            row = 1
            for f in core.FIELDS:
                if f[4] != section:
                    continue
                lab = ttk.Label(frame, text=f[1])
                lab.grid(row=row, column=0, sticky="w", pady=4, padx=(0, 14))
                cell = ttk.Frame(frame, style="Page.TFrame")
                cell.grid(row=row, column=1, sticky="w")
                w, _ = field_widget(cell, self.app, f[0], self.vars)
                w.pack(side="left")
                ttk.Label(cell, text=f[5], style="Muted.TLabel").pack(side="left", padx=6)
                Tooltip(lab, f[6]); Tooltip(w, f[6])
                row += 1
            self.nb.add(outer, text=title)

    def save(self):
        err = read_fields(self.vars, self.app.settings)
        if err:
            messagebox.showerror("Settings", err)
            return False
        self.app.save()
        return True

    def save_install(self):
        if self.save():
            self.app.show("flash")
            self.app.pages["flash"].install()

    def import_ini(self):
        p = filedialog.askopenfilename(initialdir=str(core.REPO / "experimental settings"),
                                       filetypes=[("Java configurator settings", "*.ini")])
        if p:
            try:
                self.app.settings = core.import_java_ini(p)
            except (OSError, ValueError, IndexError) as e:
                messagebox.showerror("Import", f"Could not read this file:\n{e}")
                return
            self.app.save()
            self.build_tabs()
            messagebox.showinfo("Import", "Settings imported. Check the Basic tab, then save.")

    def export(self):
        if not self.save():
            return
        p = filedialog.asksaveasfilename(defaultextension=".json", filetypes=[("EBiCS profile", "*.json")])
        if p:
            Path(p).write_text(core.json.dumps(self.app.settings, indent=2), encoding="utf-8")

    def load(self):
        p = filedialog.askopenfilename(filetypes=[("EBiCS profile", "*.json")])
        if p:
            s = core.default_settings()
            s.update(core.json.loads(Path(p).read_text(encoding="utf-8")))
            self.app.settings = s
            self.app.save()
            self.build_tabs()

    def reset(self):
        if messagebox.askyesno("Reset", "Reset all settings to the defaults?"):
            self.app.settings = core.default_settings()
            self.app.save()
            self.build_tabs()


# ============================================================================================
# install firmware
# ============================================================================================

def build_and_flash(app, log, compile_first=True):
    """Background job. Returns True on success."""
    problems = core.check_settings(app.settings)
    if problems:
        for pr in problems:
            log("ERROR: " + pr)
        return False
    if compile_first:
        log("--- building firmware ---")
        if not core.build_firmware(app.settings, log=log):
            return False
    if not core.HEX_FILE.is_file():
        log("ERROR: no compiled firmware found")
        return False
    log("--- installing on the controller ---")
    ok = app.stlink.flash(core.HEX_FILE, log=lambda l: log(l) if l.strip() and not l.startswith("  ") else None)
    if ok:
        shutil.copy2(core.ELF_FILE, core.FLASHED_ELF)
        app.live.syms = {}
        log("firmware installed and verified")
    else:
        log("ERROR: installing failed. Check the ST-Link wiring and that the controller is powered.")
    return ok


class FlashPage(Page):
    def __init__(self, master, app):
        super().__init__(master, app)
        ttk.Label(self, text="Install firmware", style="H1.TLabel").pack(anchor="w")
        self.summary = ttk.Label(self, style="Big.TLabel", justify="left")
        self.summary.pack(anchor="w", pady=(6, 14))
        bar = ttk.Frame(self, style="Page.TFrame")
        bar.pack(anchor="w")
        self.btn = ttk.Button(bar, text="Build and install", style="Accent.TButton", command=self.install)
        self.btn.pack(side="left")
        self.btn2 = ttk.Button(bar, text="Install last build again", command=lambda: self.install(False))
        self.btn2.pack(side="left", padx=8)
        self.result = ttk.Label(self, style="H2.TLabel")
        self.result.pack(anchor="w", pady=(14, 4))
        self.log = LogBox(self, height=18)
        self.log.pack(fill="both", expand=True)

    def on_show(self):
        s = self.app.settings
        disp = next(d[2] for d in core.DISPLAYS if d[0] == s["display"])
        self.summary.configure(text=f"Display: {disp}\nMotor current {s['phase_current_a']} A, battery current "
                                    f"{s['battery_current_a']} A, speed limit {s['speed_limit']} km/h")

    def install(self, compile_first=True):
        self.btn.state(["disabled"]); self.btn2.state(["disabled"])
        self.log.clear()
        self.result.configure(text="Working ...", foreground=MUTED)
        log = lambda l: self.app.ui(lambda: self.log.add(l))

        def done(ok):
            self.btn.state(["!disabled"]); self.btn2.state(["!disabled"])
            ok = ok is True
            self.result.configure(text="Done - firmware installed" if ok else "Failed - see the log",
                                  foreground=OK if ok else BAD)
        self.app.bg(lambda: build_and_flash(self.app, log, compile_first), done)


# ============================================================================================
# live data
# ============================================================================================

class LivePage(Page):
    TILES = [("battery", "Battery", "V"), ("speed", "Speed", "km/h"), ("assist", "Assist level", ""),
             ("throttle", "Throttle", "%"), ("brake", "Brake", ""), ("motor_a", "Motor current", "A"),
             ("battery_a", "Battery current", "A"), ("pas", "Pedal pulses", ""), ("halls", "Hall events", ""),
             ("temp", "Controller temp", "\u00b0C"), ("calibrated", "Motor calibrated", ""), ("kv", "KV", ""),
             ("motor_temp", "Motor temp", "\u00b0C"), ("wire_raw", "Speed/temp wire (raw ADC)", ""),
             ("sensorless", "Sensorless observer", ""), ("watchdog", "Last reset", "")]

    def __init__(self, master, app):
        super().__init__(master, app)
        top = ttk.Frame(self, style="Page.TFrame")
        top.pack(fill="x")
        ttk.Label(top, text="Live data", style="H1.TLabel").pack(side="left")
        self.status = ttk.Label(top, style="Muted.TLabel")
        self.status.pack(side="right")
        ttk.Label(self, text="Read from the running controller over the ST-Link (about once per second).",
                  style="Muted.TLabel").pack(anchor="w", pady=(2, 12))
        grid = ttk.Frame(self, style="Page.TFrame")
        grid.pack(fill="x")
        self.vals = {}
        for i, (key, label, unit) in enumerate(self.TILES):
            card = ttk.Frame(grid, style="Card.TFrame", padding=(14, 10))
            card.grid(row=i // 4, column=i % 4, padx=6, pady=6, sticky="nsew")
            grid.columnconfigure(i % 4, weight=1)
            ttk.Label(card, text=label, style="Card.TLabel", foreground=MUTED).pack(anchor="w")
            v = ttk.Label(card, text="-", style="Value.TLabel")
            v.pack(anchor="w")
            ttk.Label(card, text=unit, style="Card.TLabel", foreground=MUTED).pack(anchor="w")
            self.vals[key] = v
        self.halls = ttk.Label(self, style="Muted.TLabel")
        self.halls.pack(anchor="w", pady=(10, 0))
        self.running = False

    def on_show(self):
        self.running = True
        self.poll(first=True)

    def on_hide(self):
        self.running = False

    def poll(self, first=False):
        if not self.running:
            return
        self.app.bg(lambda: self.app.live.snapshot(with_halls=first), self.show_snap)

    def show_snap(self, snap):
        if not self.running:
            return
        if not isinstance(snap, dict):
            self.status.configure(text="no connection - check ST-Link and power" if snap is None else str(snap), foreground=BAD)
        else:
            s = self.app.settings
            self.status.configure(text="connected  " + time.strftime("%H:%M:%S"), foreground=OK)
            put = lambda k, t: self.vals[k].configure(text=t)
            cal_v = int(s["cal_v"]); cal_i = core.cal_i_ma(s["cal_i"])
            put("battery", f"{(snap.get('MS.Voltage') or 0) * cal_v / 1000:.1f}")
            put("speed", f"{core.speed_kmh(snap, s):.1f}")
            put("assist", str(snap.get("MS.assist_level", "-")))
            thr, off = snap.get("ui16_throttle") or 0, snap.get("ui16_throttle_offset") or 0
            span = max(1, int(s["throttle_max"]) - off)
            put("throttle", f"{max(0, min(100, (thr - off) * 100 // span))}   ({thr})")
            put("brake", "PRESSED" if snap.get("brake_flag") else "off")
            put("motor_a", f"{(snap.get('MS.i_q') or 0) * cal_i / 1000:.1f}")
            put("battery_a", f"{(snap.get('MS.Battery_Current') or 0) / 1000:.1f}")
            put("pas", str(snap.get("ui32_pas_events", "-")))
            put("halls", str(snap.get("ui32_hall_events", "-")))
            put("temp", str(snap.get("MS.int_Temperature", "-")))
            put("calibrated", "not needed" if s.get("sensorless") else ("yes" if snap.get("ui8_hall_calibrated") else "NO"))
            put("kv", str(snap.get("ui32_KV", "-")))
            st_idx = int(s.get("speed_temp_input") or 0)
            if st_idx and s.get("temp_on_wire"):
                put("motor_temp", str(snap.get("MS.Temperature", "-")) if snap.get("ui8_motor_temp_sensor_ok") else "not detected")
            else:
                put("motor_temp", "no sensor")
            put("wire_raw", str(snap.get(f"adc{st_idx}", "-")) if st_idx else "not used")
            put("sensorless", ("running" if snap.get("ui8_sensorless_running") else "stopped / start") if s.get("sensorless") else "off (hall)")
            put("watchdog", "WATCHDOG" if snap.get("ui8_watchdog_reset") else "normal")
            if snap.get("ui32_setup_magic") != core.SETUP_MAGIC:
                self.status.configure(text="old firmware on the controller - install the new one for all values", foreground=WARN)
            ang = core.hall_angles_deg(snap)
            if ang:
                okay = core.check_hall_angles(ang)
                self.halls.configure(text="Hall angles: " + ", ".join(f"{a:.0f}°" for a in ang) +
                                     ("   (good, about 60° apart)" if okay else "   (unusual spacing - run the motor calibration again)"),
                                     foreground=OK if okay else WARN)
        self.after(300, self.poll)


# ============================================================================================
# tools
# ============================================================================================

class ToolsPage(Page):
    def __init__(self, master, app):
        super().__init__(master, app)
        ttk.Label(self, text="Tools", style="H1.TLabel").pack(anchor="w", pady=(0, 10))
        bar = ttk.Frame(self, style="Page.TFrame")
        bar.pack(anchor="w")
        for text, cmd in (("Check connection", self.check), ("Check chip lock", self.lock_state),
                          ("Unlock chip (erases flash)", self.unlock), ("Show config.h", self.show_config),
                          ("Open firmware folder", self.open_folder)):
            ttk.Button(bar, text=text, command=cmd).pack(side="left", padx=(0, 8))
        self.log = LogBox(self, height=22)
        self.log.pack(fill="both", expand=True, pady=(12, 0))

    def _log(self, line):
        self.app.ui(lambda: self.log.add(line))

    def check(self):
        self.log.add("checking ST-Link and controller ...")
        self.app.bg(self.app.stlink.probe, lambda i: self.log.add(describe_probe(i)))

    def lock_state(self):
        self.log.add("reading read-out protection ...")
        self.app.bg(self.app.stlink.read_protection,
                    lambda r: self.log.add({"locked": "chip is LOCKED (original firmware) - unlock it before installing EBiCS",
                                            "unlocked": "chip is unlocked"}.get(r, "could not read the protection state")))

    def unlock(self):
        if not messagebox.askyesno("Unlock chip", "Unlocking erases the whole flash, including the original firmware.\n"
                                                  "This cannot be undone. Continue?"):
            return
        self.app.bg(lambda: self.app.stlink.unlock(log=self._log),
                    lambda ok: self.log.add("unlocked - switch the controller off and on again" if ok is True else "unlock failed"))

    def show_config(self):
        self.log.clear()
        for line in core.generate_config_h(self.app.settings).splitlines():
            self.log.add(line)

    def open_folder(self):
        import os
        os.startfile(str(core.REPO))


def describe_probe(i):
    if not isinstance(i, dict):
        return f"error: {i}"
    if not i["probe"]:
        return "ST-Link NOT found. Plug it into USB (a different port helps sometimes)." + (f"  ({i['error']})" if i["error"] else "")
    if not i["target"]:
        return ("ST-Link found, but the controller does not answer.\n"
                "  - is the controller switched on (battery)?\n"
                "  - is GND connected?  (on Lishui plugs the BLACK wire is SWDIO, YELLOW is GND)\n"
                "  - try swapping SWDIO and SWCLK")
    return f"connected: {i['device']}, {i['flash_kb']} KB flash, {i['voltage']:.2f} V"


# ============================================================================================
# first setup wizard
# ============================================================================================

class WizardPage(Page):
    def __init__(self, master, app):
        super().__init__(master, app)
        self.head = ttk.Label(self, style="H1.TLabel")
        self.head.pack(anchor="w")
        self.stepinfo = ttk.Label(self, style="Muted.TLabel")
        self.stepinfo.pack(anchor="w", pady=(0, 10))
        self.body = ttk.Frame(self, style="Page.TFrame")
        self.body.pack(fill="both", expand=True)
        nav = ttk.Frame(self, style="Page.TFrame")
        nav.pack(fill="x", pady=(12, 0))
        self.back_btn = ttk.Button(nav, text="← Back", command=self.back)
        self.back_btn.pack(side="left")
        self.retry_btn = ttk.Button(nav, text="↻ Retry step", command=self.retry)
        self.retry_btn.pack(side="left", padx=(8, 0))
        self.skip_btn = ttk.Button(nav, text="Skip step", command=self.skip)
        self.skip_btn.pack(side="left", padx=(8, 0))
        self.next_btn = ttk.Button(nav, text="Next →", style="Accent.TButton", command=self.next)
        self.next_btn.pack(side="right")
        Tooltip(self.retry_btn, "Start this step again from the beginning.")
        Tooltip(self.skip_btn, "Go on without finishing this step. You can come back with Back, "
                               "or do it later from Settings / Install firmware / Live data.")
        self.steps = [self.s_welcome, self.s_connect, self.s_unlock, self.s_bike, self.s_sensors, self.s_install,
                      self.s_calibrate, self.s_direction, self.s_gear, self.s_reinstall, self.s_test, self.s_done]
        self.i = 0
        self.need_reinstall = False
        self.polling = False

    def on_show(self):
        self.render()

    def on_hide(self):
        self.polling = False

    def render(self):
        Tooltip.close()
        self.polling = False
        for w in self.body.winfo_children():
            w.destroy()
        self.next_btn.state(["!disabled"])
        self.next_btn.configure(text="Next →")
        self.back_btn.state(["!disabled"] if self.i > 0 else ["disabled"])
        middle = 0 < self.i < len(self.steps) - 1
        self.retry_btn.state(["!disabled"] if middle else ["disabled"])
        self.skip_btn.state(["!disabled"] if middle else ["disabled"])
        self.stepinfo.configure(text=f"Step {self.i + 1} of {len(self.steps)}")
        self.steps[self.i]()

    SKIP_WARNINGS = {
        "s_connect": "Without a connection the next steps (unlock, install, calibration) can't work.",
        "s_unlock": "A locked chip can't be programmed. Only skip if you know it is already unlocked.",
        "s_bike": "Your answers on this page are not saved; the current settings are used.",
        "s_sensors": "Your answers on this page are not saved; the current settings are used.",
        "s_install": "The controller keeps its current firmware. Calibration and tests need the new one.",
        "s_calibrate": "Without calibration the motor may run rough or not at all (hall motors). "
                       "You can calibrate later by running this setup again.",
        "s_reinstall": "The changed direction/gearing will only be used after you install the firmware "
                       "(Install firmware page).",
    }

    def back(self):
        if self.i > 0:
            self.i -= 1
            # going back over the automatic re-install step only makes sense if something changed
            if self.steps[self.i] == self.s_reinstall and not self.need_reinstall:
                self.i -= 1
            self.render()

    def retry(self):
        self.render()

    def skip(self):
        warn = self.SKIP_WARNINGS.get(self.steps[self.i].__name__)
        if warn and not messagebox.askyesno("Skip this step?", warn + "\n\nSkip anyway?"):
            return
        self.polling = False
        self._advance()

    def _advance(self):
        if self.i < len(self.steps) - 1:
            self.i += 1
            # skip the re-install step when nothing changed
            if self.steps[self.i] == self.s_reinstall and not self.need_reinstall:
                self.i += 1
            self.render()
        else:
            self.app.show("home")

    def block(self, on=True):
        self.next_btn.state(["disabled"] if on else ["!disabled"])

    # --- small UI builders
    def text(self, t, style="Big.TLabel", **kw):
        lab = ttk.Label(self.body, text=t, style=style, justify="left", wraplength=px(760), **kw)
        lab.pack(anchor="w", pady=(0, 8))
        return lab

    def bullets(self, items):
        for t in items:
            ttk.Label(self.body, text="•  " + t, style="Big.TLabel", wraplength=px(740), justify="left").pack(anchor="w", padx=(10, 0), pady=1)

    def status(self):
        lab = ttk.Label(self.body, style="H2.TLabel", wraplength=px(760), justify="left")
        lab.pack(anchor="w", pady=(12, 6))
        return lab

    # ---------------------------------------------------------------- steps
    def s_welcome(self):
        self.head.configure(text="First setup")
        self.text("This guides you through everything, one step at a time. You need:")
        self.bullets(["An ST-Link V2 programmer on USB, wired to the controller's programming plug: "
                      "SWDIO, SWCLK and GND (don't connect 3.3 V when the battery powers the controller)",
                      "The controller connected to the motor and switched on from the battery",
                      "The wheel lifted off the ground, so it can spin freely",
                      "About 10 minutes"])
        self.text("Lishui programming plug: BLACK = SWDIO, YELLOW = GND, GREEN = SWCLK, RED = 3.3 V. "
                  "Check with a multimeter, colours can differ.", style="Muted.TLabel")

    def s_connect(self):
        self.head.configure(text="Connect")
        self.text("Plug in the ST-Link and switch the controller on. This page checks every 2 seconds.")
        st = self.status()
        st.configure(text="Checking ...", foreground=MUTED)
        self.block()
        self.polling = True

        def check():
            if self.polling:
                self.app.bg(self.app.stlink.probe, done)

        def done(info):
            if not self.polling:
                return
            ok = isinstance(info, dict) and info["target"]
            st.configure(text=("✔  " if ok else "") + describe_probe(info), foreground=OK if ok else BAD)
            self.block(not ok)
            if ok:
                self.polling = False
            else:
                self.after(2000, check)
        check()

    def s_unlock(self):
        self.head.configure(text="Unlock the chip")
        st = self.status()
        st.configure(text="Reading the chip protection ...", foreground=MUTED)
        area = ttk.Frame(self.body, style="Page.TFrame")
        area.pack(anchor="w", fill="x")
        self.block()

        def done(state):
            for w in area.winfo_children():
                w.destroy()
            if state == "unlocked":
                st.configure(text="✔  The chip is unlocked - nothing to do.", foreground=OK)
                self.block(False)
                return
            st.configure(text="The chip is locked (original Lishui firmware)." if state == "locked"
                         else "Could not read the protection state.", foreground=WARN)
            ttk.Label(area, text="To install EBiCS the chip must be unlocked once. This ERASES the original firmware "
                                 "for good - it cannot be backed up or restored.", style="Big.TLabel",
                      wraplength=px(760), justify="left").pack(anchor="w", pady=(0, 8))
            agree = tk.BooleanVar()
            ttk.Checkbutton(area, text="I understand the original firmware will be erased", variable=agree).pack(anchor="w")
            log = LogBox(area, height=6)

            def unlock():
                if not agree.get():
                    messagebox.showinfo("Unlock", "Tick the box first.")
                    return
                b.state(["disabled"])
                log.pack(fill="x", pady=8)

                def after(ok):
                    if ok is True:
                        st.configure(text="✔  Unlocked. Now switch the controller OFF and ON again, then click Next.",
                                     foreground=OK)
                        self.block(False)
                    else:
                        st.configure(text="Unlock failed - see the log. Check wiring and power, then try again.", foreground=BAD)
                        b.state(["!disabled"])
                self.app.bg(lambda: self.app.stlink.unlock(log=lambda l: self.app.ui(lambda: log.add(l))), after)
            b = ttk.Button(area, text="Unlock chip", style="Accent.TButton", command=unlock)
            b.pack(anchor="w", pady=8)
        self.app.bg(self.app.stlink.read_protection, done)

    def s_bike(self):
        self.head.configure(text="Your bike")
        self.text("A few questions. Hover over a question for help. You can change all of this later in Settings.")
        form = ttk.Frame(self.body, style="Page.TFrame")
        form.pack(anchor="w")
        self.vars = {}
        keys = ["display", "battery", "full_voltage_v", "voltage_min_v", "wheel", "wheel_mm", "speed_limit",
                "phase_current_a", "battery_current_a", "pas_magnets", "torque_sensor", "throttle", "legal_mode",
                "sensorless"]
        for r, key in enumerate(keys):
            f = core.FIELD_BY_KEY[key]
            lab = ttk.Label(form, text=f[1])
            lab.grid(row=r, column=0, sticky="w", pady=5, padx=(0, 16))
            cell = ttk.Frame(form, style="Page.TFrame")
            cell.grid(row=r, column=1, sticky="w")
            w, _ = field_widget(cell, self.app, key, self.vars)
            w.pack(side="left")
            ttk.Label(cell, text=f[5], style="Muted.TLabel").pack(side="left", padx=6)
            Tooltip(lab, f[6]); Tooltip(w, f[6])
        self.desc = ttk.Label(self.body, style="Muted.TLabel", wraplength=px(760), justify="left")
        self.desc.pack(anchor="w", pady=(10, 0))
        upd = lambda *_: self.desc.configure(text=next((d[3] for d in core.DISPLAYS if d[2] == self.vars["display"].get()), ""))
        self.vars["display"].trace_add("write", upd)
        upd()
        self.next_btn.configure(text="Save and continue →")

    def _save_bike(self):
        err = read_fields(self.vars, self.app.settings)
        if err:
            messagebox.showerror("Your bike", err)
            return False
        self.app.save()
        return True

    def s_install(self):
        self.head.configure(text="Install the firmware")
        self.text("The firmware is now built for your bike and installed on the controller. This takes about a minute.")
        st = self.status()
        bar = ttk.Progressbar(self.body, mode="indeterminate", length=px(500))
        bar.pack(anchor="w")
        log = LogBox(self.body, height=12)
        log.pack(fill="both", expand=True, pady=10)
        self._install_into(st, bar, log, compile_first=True)

    def _install_into(self, st, bar, log, compile_first):
        self.block()
        bar.start(12)
        st.configure(text="Working ...", foreground=MUTED)

        def done(ok):
            bar.stop()
            if ok is True:
                st.configure(text="✔  Firmware installed.", foreground=OK)
                self.block(False)
            else:
                st.configure(text="Installing failed - see the log below.", foreground=BAD)
                ttk.Button(self.body, text="Try again", command=self.render).pack(anchor="w")
        self.app.bg(lambda: build_and_flash(self.app, lambda l: self.app.ui(lambda: log.add(l)), compile_first), done)

    def next(self):
        if self.steps[self.i] == self.s_bike and not self._save_bike():
            return
        if self.steps[self.i] == self.s_sensors and not self._save_sensors():
            return
        self._advance()

    # ---------------------------------------------------------------- speed sensor / motor temperature
    SENSOR_MODES = [
        ("halls", "Speed from the motor's hall sensors, no temperature sensor  (most bikes)"),
        ("external", "Separate speed sensor (spoke magnet) on the controller's SPEED input"),
        ("wire", "One wire for speed and/or motor temperature on AD1 or AD2"),
    ]

    def s_sensors(self):
        self.head.configure(text="Speed sensor and motor temperature")
        s = self.app.settings
        self.text("How does the controller get the wheel speed, and does the motor have a temperature sensor? "
                  "If you're not sure, keep the first option.")
        if int(s.get("speed_temp_input") or 0):
            mode = "wire"
        elif s.get("speed_sensor_external"):
            mode = "external"
        else:
            mode = "halls"
        self.sensor_mode = tk.StringVar(value=mode)
        for key, label in self.SENSOR_MODES:
            ttk.Radiobutton(self.body, text=label, value=key, variable=self.sensor_mode,
                            command=lambda: self._sensor_form()).pack(anchor="w", padx=10, pady=2)
        self.sensor_box = ttk.Frame(self.body, style="Page.TFrame")
        self.sensor_box.pack(anchor="w", fill="x", pady=(10, 0), padx=(30, 0))
        self.vars = {}
        self._sensor_form()
        self.next_btn.configure(text="Save and continue →")

    def _sensor_form(self):
        for w in self.sensor_box.winfo_children():
            w.destroy()
        self.vars = {}
        mode = self.sensor_mode.get()
        if mode == "halls":
            keys = []
            note = "Speed comes from the motor itself. Run the gearing step later so the speed is right."
        elif mode == "external":
            keys = ["pulses_per_rev"]
            note = "Number of magnets on the wheel (usually 1)."
        else:
            keys = ["speed_temp_input", "speed_on_wire", "temp_on_wire", "ntc_pullup_ohm", "speed_temp_threshold",
                    "pulses_per_rev", "motor_temp_derate", "motor_temp_max"]
            note = ("Wire: the motor's speed/temperature wire to AD1 (PB1) or AD2 (PB0), plus a pull-up resistor to 3.3 V. "
                    "Only tick 'temperature' if the motor really has an NTC: a missing sensor is ignored (shown as "
                    "'not detected'), it no longer causes a motor overheat error. The test step shows the reading.")
        form = ttk.Frame(self.sensor_box, style="Page.TFrame")
        form.pack(anchor="w")
        for r, key in enumerate(keys):
            f = core.FIELD_BY_KEY[key]
            lab = ttk.Label(form, text=f[1].strip())
            lab.grid(row=r, column=0, sticky="w", pady=4, padx=(0, 16))
            cell = ttk.Frame(form, style="Page.TFrame")
            cell.grid(row=r, column=1, sticky="w")
            w, var = field_widget(cell, self.app, key, self.vars)
            if key == "speed_temp_input" and var.get() == core.SPEED_TEMP_INPUTS[0][0]:
                var.set(core.SPEED_TEMP_INPUTS[1][0])  # 'wire' chosen: default to AD1 instead of 'not used'
            w.pack(side="left")
            ttk.Label(cell, text=f[5], style="Muted.TLabel").pack(side="left", padx=6)
            Tooltip(lab, f[6]); Tooltip(w, f[6])
        ttk.Label(self.sensor_box, text=note, style="Muted.TLabel", wraplength=px(720), justify="left").pack(anchor="w", pady=(8, 0))

    def _save_sensors(self):
        s = dict(self.app.settings)
        mode = self.sensor_mode.get()
        if mode == "halls":
            s.update(speed_temp_input=0, speed_sensor_external=False)
        elif mode == "external":
            s.update(speed_temp_input=0, speed_sensor_external=True)
        else:
            s.update(speed_sensor_external=False)
        err = read_fields(self.vars, s)
        if not err and mode == "wire" and not (s.get("speed_on_wire") or s.get("temp_on_wire")):
            err = "Tick 'use it for speed' and/or 'use it for motor temperature', or choose another option."
        if err:
            messagebox.showerror("Speed sensor and motor temperature", err)
            return False
        self.app.settings.update(s)
        self.app.save()
        return True

    def s_calibrate(self):
        self.head.configure(text="Motor calibration")
        if self.app.settings.get("sensorless"):
            self.text("Sensorless motor: there are no hall sensors to calibrate.")
            self.bullets(["Check the motor parameters in Settings > Expert > 'Sensorless' (inductance, resistance, "
                          "flux linkage, gain, angle offset). The defaults are from the EBiCS Sensorless_VESC branch.",
                          "The motor starts with a short open loop spin before the observer takes over.",
                          "Sensorless is experimental: test with the wheel lifted, low current first."])
            return
        self.text("The controller now finds the position of the motor's hall sensors and measures the motor. "
                  "Before you start:")
        checks = []
        for t in ("The wheel is lifted and can spin freely", "Nothing touches the wheel, chain or pedals",
                  "Hands and cables are away from the wheel"):
            v = tk.BooleanVar()
            ttk.Checkbutton(self.body, text=t, variable=v).pack(anchor="w", padx=10, pady=2)
            checks.append(v)
        st = self.status()
        bar = ttk.Progressbar(self.body, length=px(560), maximum=100)
        bar.pack(anchor="w", pady=4)
        detail = ttk.Label(self.body, style="Muted.TLabel", wraplength=px(760), justify="left")
        detail.pack(anchor="w")
        self.block()

        def start():
            if not all(v.get() for v in checks):
                messagebox.showinfo("Calibration", "Please confirm all three points first.")
                return
            btn.state(["disabled"])
            st.configure(text="Starting ...", foreground=MUTED)

            def started(ok):
                if ok is not True:
                    st.configure(text="Could not start: " + (ok if isinstance(ok, str) else "no connection to the controller"),
                                 foreground=BAD)
                    btn.state(["!disabled"])
                    return
                self.polling = True
                self.cal = {"t0": time.time(), "stopped_since": None, "last_halls": None, "phase": 0}
                self.poll_calibration(st, bar, detail, btn)

            def kick():
                if not self.app.live.load_symbols():
                    return "firmware information missing - install the firmware first (previous step)"
                snap = self.app.live.snapshot()
                if not snap:
                    return None
                if snap.get("ui32_setup_magic") != core.SETUP_MAGIC:
                    return "the controller runs an old firmware - go back and install the firmware"
                return self.app.live.start_calibration() or None
            self.app.bg(kick, started)
        btn = ttk.Button(self.body, text="Start calibration", style="Accent.TButton", command=start)
        btn.pack(anchor="w", pady=10)

    def poll_calibration(self, st, bar, detail, btn):
        if not self.polling:
            return

        def show(snap):
            if not self.polling:
                return
            if not isinstance(snap, dict):
                detail.configure(text="(no answer from the controller this time, retrying ...)")
                self.after(500, lambda: self.poll_calibration(st, bar, detail, btn))
                return
            state, prog = snap.get("ui8_setup_state"), snap.get("ui8_setup_progress") or 0
            halls = snap.get("ui32_hall_events")
            if state == 0:
                if time.time() - self.cal["t0"] > 8:
                    st.configure(text="The controller did not start the calibration.", foreground=BAD)
                    detail.configure(text="Is the motor running (throttle/pedals)? Release everything and try again.")
                    btn.state(["!disabled"])
                    self.polling = False
                    return
                st.configure(text="Starting ...", foreground=MUTED)
            elif state == 1:
                st.configure(text="1/2  Finding the hall sensor positions ...", foreground=ACCENT)
                detail.configure(text="The motor turns slowly and may click or buzz - that is normal. About 6 seconds.")
                bar["value"] = prog
            elif state == 2:
                st.configure(text="2/2  Measuring the motor speed ...", foreground=ACCENT)
                detail.configure(text="The wheel speeds up to full speed. This can take a few minutes. Keep clear of the wheel.")
                bar["value"] = prog
            elif state == 4:
                found = snap.get("ui8_setup_halls_found") or 0
                st.configure(text="✖  Calibration failed: not all hall sensor positions were found.", foreground=BAD)
                detail.configure(text=f"Found {bin(found).count('1')} of 6. Check the hall sensor plug (5 thin wires) "
                                      "and that the wheel can turn freely, then switch the controller off and on and try again.")
                btn.state(["!disabled"])
                self.polling = False
                return
            elif state == 3:
                bar["value"] = 100
                if halls == self.cal["last_halls"]:
                    self.cal["stopped_since"] = self.cal["stopped_since"] or time.time()
                else:
                    self.cal["stopped_since"] = None
                if self.cal["stopped_since"] and time.time() - self.cal["stopped_since"] > 1.5:
                    self.polling = False
                    self.app.bg(lambda: self.app.live.snapshot(with_halls=True), lambda s: self.cal_done(s, st, detail))
                    return
                st.configure(text="Measured. Please wait until the wheel has stopped ...", foreground=WARN)
                detail.configure(text="Don't touch the wheel - it is still coasting.")
                self.cal.setdefault("measured_at", time.time())
                if time.time() - self.cal["measured_at"] > 45 and not self.cal.get("manual_btn"):
                    # hall noise can keep the counter moving: let the user confirm instead of waiting forever
                    def manual():
                        self.polling = False
                        self.app.bg(lambda: self.app.live.snapshot(with_halls=True), lambda s: self.cal_done(s, st, detail))
                    self.cal["manual_btn"] = ttk.Button(self.body, text="The wheel has stopped", command=manual)
                    self.cal["manual_btn"].pack(anchor="w", pady=6)
            self.cal["last_halls"] = halls
            self.after(300, lambda: self.poll_calibration(st, bar, detail, btn))
        self.app.bg(self.app.live.snapshot, show)

    def cal_done(self, snap, st, detail):
        ang = core.hall_angles_deg(snap) if isinstance(snap, dict) else None
        good = core.check_hall_angles(ang)
        st.configure(text="✔  Calibration done, the wheel has stopped.", foreground=OK)
        msg = ""
        if ang:
            msg = "Hall angles: " + ", ".join(f"{a:.0f}°" for a in ang)
            msg += "  - look good." if good else "  - unusual spacing. It may still work; if the motor runs rough, calibrate again."
        if isinstance(snap, dict) and snap.get("ui32_KV"):
            msg += f"\nMotor KV value: {snap['ui32_KV']}"
        detail.configure(text=msg)
        self.block(False)

    def s_direction(self):
        self.head.configure(text="Motor direction")
        if self.app.settings.get("sensorless") or not self.app.settings.get("throttle"):
            self.text("Give the motor a short spin (throttle, or turn the pedals with the wheel lifted). "
                      "Which way did the wheel turn?")
        else:
            self.text("During the calibration the wheel spun up. Which way did it turn?")
        row = ttk.Frame(self.body, style="Page.TFrame")
        row.pack(anchor="w", pady=10)
        st = self.status()
        self.block()

        def answer(forward):
            if forward:
                st.configure(text="✔  Direction is correct.", foreground=OK)
            else:
                self.app.settings["reverse"] = not self.app.settings["reverse"]
                self.app.save()
                self.need_reinstall = True
                st.configure(text="OK - the direction will be reversed. The firmware is installed again in a moment.",
                             foreground=WARN)
            self.block(False)
        ttk.Button(row, text="Forward (correct)", style="Accent.TButton", command=lambda: answer(True)).pack(side="left")
        ttk.Button(row, text="Backward", command=lambda: answer(False)).pack(side="left", padx=10)

    def s_gear(self):
        self.head.configure(text="Measure the motor gearing")
        if self.app.settings.get("sensorless"):
            self.text("Without hall sensors the gearing can't be measured by turning the wheel. Enter "
                      "'Pole pairs x gear ratio' in Settings, or use an external speed sensor / the speed wire "
                      "(Settings > Advanced) for a correct speed.")
            return
        self.text("This makes the speed on the display correct. Put a mark on the tyre (e.g. at the valve).")
        self.bullets(["Click 'Start measuring'",
                      "Turn the wheel slowly BACKWARDS by hand, exactly 3 full turns",
                      "Click 'Done'"])
        self.text("(Backwards, because geared hub motors freewheel when the wheel turns forward.)", style="Muted.TLabel")
        row = ttk.Frame(self.body, style="Page.TFrame")
        row.pack(anchor="w", pady=10)
        st = self.status()
        st.configure(text=f"Current value: {self.app.settings['gear_ratio']}   (you can also skip this step)", foreground=MUTED)
        state = {"start": None}

        def begin():
            def got(snap):
                if not isinstance(snap, dict) or snap.get("ui32_hall_events") is None:
                    st.configure(text="No connection to the controller.", foreground=BAD)
                    return
                state["start"] = snap["ui32_hall_events"]
                st.configure(text="Measuring - turn the wheel backwards 3 full turns, then click Done.", foreground=ACCENT)
                b2.state(["!disabled"])
            self.app.bg(self.app.live.snapshot, got)

        def finish():
            def got(snap):
                if not isinstance(snap, dict) or state["start"] is None:
                    st.configure(text="No connection to the controller.", foreground=BAD)
                    return
                events = (snap["ui32_hall_events"] - state["start"]) & 0xFFFFFFFF
                ratio = round(events / 18.0)  # 6 hall events per electrical turn, 3 wheel turns
                if ratio < 4:
                    st.configure(text=f"Only {events} hall events counted - did the motor turn? Try again "
                                      "(turn backwards, not forwards).", foreground=BAD)
                    return
                if ratio != int(self.app.settings["gear_ratio"]):
                    self.app.settings["gear_ratio"] = ratio
                    self.app.save()
                    self.need_reinstall = True
                st.configure(text=f"✔  Measured: {ratio}  ({events} hall events in 3 turns). Saved.", foreground=OK)
            self.app.bg(self.app.live.snapshot, got)
        ttk.Button(row, text="Start measuring", style="Accent.TButton", command=begin).pack(side="left")
        b2 = ttk.Button(row, text="Done", command=finish)
        b2.pack(side="left", padx=10)
        b2.state(["disabled"])

    def s_reinstall(self):
        self.head.configure(text="Update the firmware")
        self.text("Your answers changed the settings, so the firmware is installed again. "
                  "The motor calibration is kept.")
        st = self.status()
        bar = ttk.Progressbar(self.body, mode="indeterminate", length=px(500))
        bar.pack(anchor="w")
        log = LogBox(self.body, height=10)
        log.pack(fill="both", expand=True, pady=10)
        self.need_reinstall = False
        self._install_into(st, bar, log, compile_first=True)

    def s_test(self):
        self.head.configure(text="Test")
        self.text("Try each item. It gets a tick when the controller sees it. (Wheel still lifted!)")
        items = {"display": ("Display: change the assist level on the display", None),
                 "throttle": ("Throttle: twist it - the wheel should turn", None),
                 "brake": ("Brake: pull a brake lever", None),
                 "pas": ("Pedal sensor: turn the pedals forward by hand", None)}
        if not self.app.settings["throttle"]:
            del items["throttle"]
        s = self.app.settings
        wire = int(s.get("speed_temp_input") or 0)
        if s.get("speed_sensor_external") or (wire and s.get("speed_on_wire")):
            items["speed"] = ("Speed sensor: turn the wheel by hand a few turns", None)
        if wire and s.get("temp_on_wire"):
            items["temp"] = ("Motor temperature sensor: reading must look plausible", None)
        labels = {}
        for k, (t, _) in items.items():
            labels[k] = ttk.Label(self.body, text="○  " + t, style="Big.TLabel")
            labels[k].pack(anchor="w", padx=10, pady=3)
        st = self.status()
        st.configure(text="Waiting for input ...", foreground=MUTED)
        seen, base = set(), {}
        self.polling = True

        def poll():
            if not self.polling:
                return

            def show(snap):
                if not self.polling:
                    return
                if isinstance(snap, dict):
                    if not base:
                        base.update(snap)
                    if snap.get("MS.assist_level") != base.get("MS.assist_level"):
                        seen.add("display")
                    if (snap.get("ui16_throttle") or 0) > (snap.get("ui16_throttle_offset") or 0) + 150:
                        seen.add("throttle")
                    if snap.get("brake_flag"):
                        seen.add("brake")
                    if (snap.get("ui32_pas_events") or 0) - (base.get("ui32_pas_events") or 0) > 4:
                        seen.add("pas")
                    if "speed" in labels and snap.get("MS.Speed") not in (None, base.get("MS.Speed"), 32000):
                        seen.add("speed")
                    if "temp" in labels:
                        ok = snap.get("ui8_motor_temp_sensor_ok")
                        temp = snap.get("MS.Temperature")
                        raw = snap.get(f"adc{wire}")
                        if ok and temp is not None and -20 <= temp <= 90:
                            seen.add("temp")
                            items["temp"] = (f"Motor temperature sensor: {temp} °C (raw {raw})", None)
                        elif ok:
                            items["temp"] = (f"Motor temperature sensor: {temp} °C looks wrong (raw {raw}) - "
                                             "check 'NTC pull-up resistor' in Settings", None)
                            labels["temp"].configure(text="!  " + items["temp"][0], foreground=WARN)
                        else:
                            items["temp"] = (f"Motor temperature sensor: NOT DETECTED (raw {raw}) - wire or pull-up "
                                             "missing; temperature is ignored. Untick it in Settings if there is none.", None)
                            labels["temp"].configure(text="!  " + items["temp"][0], foreground=WARN)
                    for k in labels:
                        if k in seen:
                            labels[k].configure(text="✔  " + items[k][0], foreground=OK)
                    st.configure(text="All good!" if seen >= set(labels) else "Waiting for input ...",
                                 foreground=OK if seen >= set(labels) else MUTED)
                else:
                    st.configure(text="No connection to the controller ...", foreground=BAD)
                self.after(300, poll)
            self.app.bg(self.app.live.snapshot, show)
        poll()
        self.text("If something doesn't get a tick, you can still finish and check it later on the Live data page.",
                  style="Muted.TLabel")

    def s_done(self):
        self.head.configure(text="Finished")
        self.text("Your controller is set up. Lower the wheel and go for a careful test ride.")
        self.bullets(["Change power, speed limit or display later in Settings, then 'Save + install firmware'",
                      "Live data shows what the controller sees (battery, throttle, brake, speed ...)",
                      "If you change the motor or swap wires, run this setup again"])
        self.next_btn.configure(text="Close")


def main():
    try:  # sharp text on scaled Windows displays
        import ctypes
        ctypes.windll.shcore.SetProcessDpiAwareness(1)
    except (AttributeError, OSError):
        pass
    app = App()
    app.mainloop()


if __name__ == "__main__":
    main()
