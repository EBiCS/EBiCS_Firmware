"""SWIM access to STM8 chips (Kunteng KT controllers) through an ST-Link V2 / V2-1 / V3, in pure Python.

The ST-Link speaks SWIM itself; this module only sends its USB commands (same protocol as the open
source stm8flash tool). Needs pyusb and a libusb backend (pip install pyusb libusb-package).

Memory map of the STM8S105x6 used in KT controllers:
  flash 0x8000-0xFFFF (32 KB, 128 byte blocks), data EEPROM 0x4000-0x43FF, option bytes 0x4800-0x487F.
"""
import time

ST_VID = 0x0483
# (product id, OUT endpoint, IN endpoint)
STLINK_PIDS = {0x3748: (0x02, 0x81),   # V2
               0x374B: (0x01, 0x81),   # V2-1
               0x374A: (0x01, 0x81),   # V2-1 without mass storage
               0x3752: (0x01, 0x81),   # V2-1 variant
               0x374E: (0x01, 0x81),   # V3
               0x374F: (0x01, 0x81),   # V3
               0x3753: (0x01, 0x81)}   # V3

GET_VERSION, DEBUG, DFU, SWIM, GET_CURRENT_MODE = 0xF1, 0xF2, 0xF3, 0xF4, 0xF5
MODE_DFU, MODE_MASS, MODE_DEBUG, MODE_SWIM, MODE_BOOTLOADER = 0, 1, 2, 3, 4
DEBUG_EXIT, DFU_EXIT = 0x21, 0x07
(SWIM_ENTER, SWIM_EXIT, SWIM_READ_CAP, SWIM_SPEED, SWIM_ENTER_SEQ, SWIM_GEN_RST, SWIM_RESET, SWIM_ASSERT_RESET,
 SWIM_DEASSERT_RESET, SWIM_READSTATUS, SWIM_WRITEMEM, SWIM_READMEM, SWIM_READBUF, SWIM_READBUFSIZE) = range(14)
SWIM_OK, SWIM_BUSY = 0x00, 0x01

# STM8S registers
SWIM_CSR = 0x7F80
CLK_CKDIVR = 0x50C6
FLASH_CR2, FLASH_NCR2, FLASH_IAPSR = 0x505B, 0x505C, 0x505F
FLASH_PUKR, FLASH_DUKR = 0x5062, 0x5064
DM_CSR2 = 0x7F99

FLASH_START, FLASH_SIZE, BLOCK = 0x8000, 0x8000, 128
EEPROM_START, EEPROM_SIZE = 0x4000, 0x400
OPT_START = 0x4800

# Option bytes for the KT controller (same values as optionbytes.hex of the KT firmware):
# ROP off, no UBC, AFR5 = 1 (TIM1_CH1N..CH3N on PB0..PB2, the low side gate drivers), the rest default.
# Each option byte is followed by its complement, except ROP.
KT_OPTION_BYTES = bytes([0x00,          # 0x4800 ROP = 0: read-out protection off (removing it erases the chip)
                         0x00, 0xFF,    # UBC / NUBC
                         0x20, 0xDF,    # AFR / NAFR: AFR5
                         0x00, 0xFF,    # OPT3
                         0x00, 0xFF,    # OPT4
                         0x00, 0xFF,    # OPT5
                         0x00, 0xFF,    # OPT6
                         0x00, 0xFF])   # OPT7


class SwimError(Exception):
    pass


class UsbTransport:
    """Bulk transfers to a real ST-Link through pyusb."""

    def __init__(self):
        try:
            import usb.core
            import usb.util
        except ImportError:
            raise SwimError("Python module 'pyusb' is missing. Install it with:  python -m pip install pyusb libusb-package")
        backend = None
        try:
            import libusb_package
            backend = libusb_package.get_libusb1_backend()
        except ImportError:
            pass
        try:
            devs = list(usb.core.find(find_all=True, idVendor=ST_VID, backend=backend))
        except usb.core.NoBackendError:
            raise SwimError("No USB backend. Install it with:  python -m pip install libusb-package")
        devs = [d for d in devs if d.idProduct in STLINK_PIDS]
        if not devs:
            raise SwimError("ST-Link not found. Plug it into USB (a different port helps sometimes).")
        self.dev = devs[0]
        self.ep_out, self.ep_in = STLINK_PIDS[self.dev.idProduct]
        try:
            try:
                self.dev.set_configuration()
            except usb.core.USBError:
                pass  # already configured by the driver
            usb.util.claim_interface(self.dev, 0)
        except Exception as e:
            raise SwimError("Cannot open the ST-Link (%s). Close STM32CubeProgrammer / ST-Link Utility. If it still fails, "
                            "the ST-Link needs the WinUSB driver (Zadig, or ST's ST-Link driver)." % e)
        self._usb = usb

    def write(self, data, timeout=3000):
        self.dev.write(self.ep_out, bytes(data), timeout)

    def read(self, n, timeout=3000):
        return bytes(self.dev.read(self.ep_in, n, timeout))

    def close(self):
        try:
            self._usb.util.dispose_resources(self.dev)
        except Exception:
            pass


class Swim:
    """One SWIM session. Use as a context manager: with Swim() as s: ..."""

    def __init__(self, transport=None, log=None, sleep=time.sleep):
        self.t = transport
        self.log = log or (lambda _l: None)
        self.sleep = sleep
        self.read_buf_size = 6144
        self.version = None

    # ---------------------------------------------------------------- low level
    def _cmd(self, *args, data=b"", rx=0):
        buf = bytearray(16)
        payload = bytes(args) + bytes(data)
        head = payload[:16]
        buf[:len(head)] = head
        self.t.write(buf)
        if len(payload) > 16:
            self.t.write(payload[16:])
        return self.t.read(rx) if rx else b""

    def _swim(self, *args, data=b""):
        """SWIM command, then poll READSTATUS until the ST-Link reports done."""
        self._cmd(SWIM, *args, data=data)
        last, stalls = None, 0
        for _ in range(400):
            st = self._cmd(SWIM, SWIM_READSTATUS, rx=4)
            if st[0] == SWIM_OK:
                return
            if st == last:
                stalls += 1
                if stalls >= 4:
                    break
            else:
                stalls = 0
            last = st
            self.sleep(0.01)
        raise SwimError("SWIM command %02x failed (status %02x). Check the wiring (SWIM, GND, RST) and that the "
                        "controller is powered." % (args[0], last[0] if last else 0xFF))

    @staticmethod
    def _addr(a):
        return [0x00, (a >> 16) & 0xFF, (a >> 8) & 0xFF, a & 0xFF]

    def read_mem(self, addr, length):
        out = bytearray()
        while length > 0:
            n = min(length, self.read_buf_size)
            self._swim(SWIM_READMEM, n >> 8, n & 0xFF, *self._addr(addr))
            self._cmd(SWIM, SWIM_READBUF)
            out += self.t.read(n)
            addr += n
            length -= n
        return bytes(out)

    def read_byte(self, addr):
        return self.read_mem(addr, 1)[0]

    def write_mem(self, addr, data):
        data = bytes(data)
        self._swim(SWIM_WRITEMEM, len(data) >> 8, len(data) & 0xFF, *self._addr(addr), data=data)

    def write_byte(self, addr, value):
        self.write_mem(addr, [value & 0xFF])

    # ---------------------------------------------------------------- session
    def open(self):
        if self.t is None:
            self.t = UsbTransport()
        v = self._cmd(GET_VERSION, rx=6)
        ver = (v[0] << 8) | v[1]
        self.version = {"stlink": (ver >> 12) & 0x3F, "jtag": (ver >> 6) & 0x3F, "swim": ver & 0x3F}
        if self.version["swim"] == 0:
            raise SwimError("This ST-Link has no SWIM support (needs an ST-Link V2 with SWIM pin).")
        mode = self._cmd(GET_CURRENT_MODE, rx=2)[0]
        if mode == MODE_DEBUG:
            self._cmd(DEBUG, DEBUG_EXIT)
        elif mode in (MODE_DFU, MODE_MASS, MODE_BOOTLOADER):
            self._cmd(DFU, DFU_EXIT)
        if mode != MODE_SWIM:
            self._cmd(SWIM, SWIM_ENTER)
        bs = self._cmd(SWIM, SWIM_READBUFSIZE, rx=2)
        self.read_buf_size = (bs[0] | (bs[1] << 8)) or 6144
        self._cmd(SWIM, SWIM_READ_CAP, 0x01, rx=8)
        self._swim(SWIM_ASSERT_RESET)
        self._swim(SWIM_ENTER_SEQ)
        # mask interrupts, access to the whole memory, SWIM has priority, stall the CPU
        self.write_byte(SWIM_CSR, 0xA1)
        self._swim(SWIM_DEASSERT_RESET)
        self.sleep(0.001)
        self._high_speed()
        return self

    def open_live(self):
        """Connect to the running firmware without reset and without stalling the CPU (live data). The SWIM
        entry sequence does not reset the chip; SWIM_CSR is left as it is (no SAFE_MASK: interrupts keep running),
        memory is read on the fly between CPU accesses."""
        if self.t is None:
            self.t = UsbTransport()
        v = self._cmd(GET_VERSION, rx=6)
        ver = (v[0] << 8) | v[1]
        self.version = {"stlink": (ver >> 12) & 0x3F, "jtag": (ver >> 6) & 0x3F, "swim": ver & 0x3F}
        if self.version["swim"] == 0:
            raise SwimError("This ST-Link has no SWIM support (needs an ST-Link V2 with SWIM pin).")
        mode = self._cmd(GET_CURRENT_MODE, rx=2)[0]
        if mode == MODE_DEBUG:
            self._cmd(DEBUG, DEBUG_EXIT)
        elif mode in (MODE_DFU, MODE_MASS, MODE_BOOTLOADER):
            self._cmd(DFU, DFU_EXIT)
        if mode != MODE_SWIM:
            self._cmd(SWIM, SWIM_ENTER)
        bs = self._cmd(SWIM, SWIM_READBUFSIZE, rx=2)
        self.read_buf_size = (bs[0] | (bs[1] << 8)) or 6144
        self._cmd(SWIM, SWIM_READ_CAP, 0x01, rx=8)
        self._swim(SWIM_ENTER_SEQ)
        self._swim(SWIM_RESET)          # resynchronise the SWIM clock, does not reset the chip
        return self

    def _high_speed(self):
        csr = 0
        for _ in range(10):
            csr = self.read_byte(SWIM_CSR)
            if csr & 0x02:  # HSIT: high speed possible
                break
            self.sleep(0.0005)
        self._swim(SWIM_RESET)
        if csr & 0x02:
            self.write_byte(SWIM_CSR, csr | 0x10)
            self._swim(SWIM_SPEED, 1)

    def reset_target(self):
        """Start the firmware (system reset through SWIM)."""
        self.write_byte(SWIM_CSR, self.read_byte(SWIM_CSR) | 0x04)
        self._swim(SWIM_GEN_RST)
        self.sleep(0.001)

    def close(self):
        if self.t is not None:
            try:
                self._cmd(SWIM, SWIM_EXIT)
            except Exception:
                pass
            if hasattr(self.t, "close"):
                self.t.close()

    def __enter__(self):
        return self.open()

    def __exit__(self, *exc):
        self.close()

    # ---------------------------------------------------------------- flash programming
    def _wait_eop(self, what):
        for _ in range(10):
            sr = self.read_byte(FLASH_IAPSR)
            if sr & 0x04:   # EOP
                return
            if sr & 0x01:   # WR_PG_DIS
                raise SwimError("%s: write protected. The chip is still locked - run 'Unlock chip' first." % what)
            self.sleep(0.003)
        raise SwimError("%s: programming timeout" % what)

    def _lock(self):
        sr = self.read_byte(FLASH_IAPSR)
        self.write_byte(FLASH_IAPSR, sr & ~0x0A)   # clear PUL and DUL

    def write_blocks(self, start, data, progress=None):
        """Program flash or EEPROM in 128 byte blocks. Only blocks that differ are written."""
        is_flash = start >= FLASH_START
        if start % BLOCK:
            raise SwimError("start address must be block aligned")
        if len(data) % BLOCK:
            data = bytes(data) + bytes(BLOCK - len(data) % BLOCK)   # SDCC leaves unused flash as 0x00
        self.write_byte(CLK_CKDIVR, 0x00)   # CPU at full 16 MHz while programming
        if is_flash:
            self.write_byte(FLASH_PUKR, 0x56)
            self.write_byte(FLASH_PUKR, 0xAE)
        else:
            self.write_byte(FLASH_DUKR, 0xAE)
            self.write_byte(FLASH_DUKR, 0x56)
        current = self.read_mem(start, len(data))
        n = len(data) // BLOCK
        for i in range(n):
            blk = data[i * BLOCK:(i + 1) * BLOCK]
            old = current[i * BLOCK:(i + 1) * BLOCK]
            if blk != old:
                fast = not any(old)   # fast programming only into an erased (all 0x00) block
                mode = 0x10 if fast else 0x01
                self.write_byte(DM_CSR2, self.read_byte(DM_CSR2) | 0x08)   # keep the CPU stalled
                self.write_byte(FLASH_CR2, mode)
                self.write_byte(FLASH_NCR2, (~mode) & 0xFF)
                self.write_mem(start + i * BLOCK, blk)
                self.sleep(0.003 if fast else 0.006)
                self._wait_eop("block 0x%04X" % (start + i * BLOCK))
            if progress:
                progress(i + 1, n)
        self._lock()

    def write_option_bytes(self, values=KT_OPTION_BYTES):
        """Write option bytes one by one. Writing ROP = 0 on a protected chip erases flash and EEPROM."""
        self.write_byte(CLK_CKDIVR, 0x00)
        self.write_byte(FLASH_DUKR, 0xAE)
        self.write_byte(FLASH_DUKR, 0x56)
        self.write_byte(FLASH_CR2, 0x80)    # OPT: option byte programming
        self.write_byte(FLASH_NCR2, 0x7F)
        for i, v in enumerate(values):
            self.write_byte(OPT_START + i, v)
            self.sleep(0.007)
            self._wait_eop("option byte %d" % i)
        self._lock()

    def read_option_bytes(self, n=len(KT_OPTION_BYTES)):
        return self.read_mem(OPT_START, n)


# -------------------------------------------------------------------- Intel hex
def read_ihx(path):
    """Returns {address: byte} from an Intel hex file (SDCC .ihx)."""
    mem, base = {}, 0
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line.startswith(":"):
                continue
            raw = bytes.fromhex(line[1:])
            if (sum(raw) & 0xFF) != 0:
                raise SwimError("hex file checksum error: " + line)
            n, addr, typ = raw[0], (raw[1] << 8) | raw[2], raw[3]
            if typ == 0:
                for i in range(n):
                    mem[base + addr + i] = raw[4 + i]
            elif typ == 1:
                break
            elif typ == 4:
                base = ((raw[4] << 8) | raw[5]) << 16
            elif typ == 2:
                base = ((raw[4] << 8) | raw[5]) << 4
    return mem


def flash_image(mem):
    """Flash bytes of an image as one contiguous block run starting at 0x8000 (padded with 0x00)."""
    addrs = [a for a in mem if FLASH_START <= a < FLASH_START + FLASH_SIZE]
    if not addrs:
        raise SwimError("hex file has no data for the STM8 flash (0x8000-0xFFFF)")
    if any(a < FLASH_START or a >= FLASH_START + FLASH_SIZE for a in mem if not (EEPROM_START <= a < EEPROM_START + EEPROM_SIZE)):
        raise SwimError("hex file has data outside the STM8S105 flash")
    end = max(addrs) + 1
    end = (end + BLOCK - 1) // BLOCK * BLOCK
    img = bytearray(end - FLASH_START)
    for a in addrs:
        img[a - FLASH_START] = mem[a]
    return bytes(img)


# -------------------------------------------------------------------- high level jobs
def probe(transport=None):
    """Returns a dict with what was found. Never raises."""
    info = {"probe": False, "target": False, "locked": None, "error": None, "version": None}
    s = Swim(transport)
    try:
        s.open()
        info["probe"] = True
        info["version"] = s.version
        opt = s.read_option_bytes(4)
        info["target"] = True
        # a protected chip reads back ROP = 0xAA and zeros everywhere else
        info["locked"] = opt[0] == 0xAA
        info["option_bytes"] = opt.hex()
    except SwimError as e:
        info["error"] = str(e)
        info["probe"] = info["probe"] or "SWIM command" in str(e)
    except Exception as e:  # USB errors from pyusb
        info["error"] = str(e)
    finally:
        s.close()
    return info


def unlock(log=print, transport=None):
    """Remove read-out protection and set the option bytes the KT firmware needs. Erases the chip."""
    with Swim(transport, log) as s:
        log("writing option bytes (removes read-out protection, erases the original firmware) ...")
        s.write_option_bytes(KT_OPTION_BYTES)
        got = s.read_option_bytes()
        if got != KT_OPTION_BYTES:
            log("option bytes read back as %s - switch the controller off and on, then check again" % got.hex())
        else:
            log("option bytes ok")
        s.reset_target()
    return True


SETTINGS_BLOCK_ADDR = 0x4080   # runtime settings of the KT firmware (kunteng/settings.h)


def _write_settings(s, settings_block, log):
    """Settings block into EEPROM block 1, keeping the rest of that block."""
    blk = bytearray(s.read_mem(SETTINGS_BLOCK_ADDR, BLOCK))
    blk[:len(settings_block)] = settings_block
    s.write_blocks(SETTINGS_BLOCK_ADDR, bytes(blk))
    if s.read_mem(SETTINGS_BLOCK_ADDR, len(settings_block)) != bytes(settings_block):
        raise SwimError("settings block verify failed")
    log("settings written")


def write_settings(settings_block, log=print, transport=None):
    """Only the settings block (the firmware must be installed already), then restart the controller."""
    with Swim(transport, log) as s:
        _write_settings(s, settings_block, log)
        s.reset_target()
    return True


def flash(ihx_path, log=print, transport=None, clear_eeprom_key=True, settings_block=None):
    """Program the firmware and verify it. With settings_block: also write the runtime settings."""
    img = flash_image(read_ihx(ihx_path))
    with Swim(transport, log) as s:
        opt = s.read_option_bytes(4)
        if opt[0] == 0xAA:
            raise SwimError("the chip is locked (original firmware). Unlock it first (erases the original firmware).")
        if opt[3] != KT_OPTION_BYTES[3]:
            log("option bytes are not set up for the KT board (AFR5): writing them first")
            s.write_option_bytes(KT_OPTION_BYTES)
        log("writing %d bytes ..." % len(img))
        last = [-1]

        def prog(i, n):
            pct = i * 100 // n
            if pct // 10 != last[0]:
                last[0] = pct // 10
                log("  %d %%" % pct)
        s.write_blocks(FLASH_START, img, prog)
        log("verifying ...")
        if s.read_mem(FLASH_START, len(img)) != img:
            raise SwimError("verify failed: flash content differs from the firmware file")
        if clear_eeprom_key:
            # forget the display settings stored by the old firmware: the new defaults are used until the display sends its own
            blk = bytearray(s.read_mem(EEPROM_START, BLOCK))
            if blk[0] != 0:
                blk[0] = 0
                s.write_blocks(EEPROM_START, bytes(blk))
        if settings_block is not None:
            _write_settings(s, settings_block, log)
        s.reset_target()
    log("firmware installed and verified")
    return True
