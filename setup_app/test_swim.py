"""Tests for swim.py against a simulated ST-Link + STM8S105 (no hardware needed).

Run: python setup_app/test_swim.py
"""
import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import swim  # noqa: E402


class FakeStm8StLink:
    """Speaks the ST-Link USB command protocol and keeps an STM8S105 memory image."""

    def __init__(self, locked=False, mode=swim.MODE_DFU):
        self.mem = bytearray(0x10000)
        self.mode = mode
        self.locked = locked
        if locked:
            self.mem[swim.OPT_START] = 0xAA
            self.mem[swim.FLASH_START:swim.FLASH_START + 64] = b"\xA5" * 64   # "original firmware"
            self.mem[swim.EEPROM_START] = 0x55
        self.pending = b""         # data the next read returns
        self.expect_data = 0       # bytes of a WRITEMEM still to come in a separate USB transfer
        self.write_addr = 0
        self.readbuf = b""
        self.log = []
        self.block_writes = 0
        self.opt_writes = []

    # --- transport interface
    def write(self, data):
        data = bytes(data)
        if self.expect_data:
            self._store(data[:self.expect_data])
            self.expect_data -= len(data)
            return
        self._command(data)

    def read(self, n):
        out, self.pending = self.pending[:n], self.pending[n:]
        assert len(out) == n, "host read %d bytes but only %d were pending" % (n, len(out))
        return out

    # --- simulation
    def _command(self, c):
        op = c[0]
        if op == swim.GET_VERSION:
            self.pending = bytes([0x26, 0x07, 0x83, 0x04, 0x48, 0x37])   # V2, JTAG 24, SWIM 7
        elif op == swim.GET_CURRENT_MODE:
            self.pending = bytes([self.mode, 0])
        elif op == swim.DFU:
            self.mode = swim.MODE_MASS
        elif op == swim.SWIM:
            self._swim(c)
        else:
            raise AssertionError("unexpected command %02x" % op)

    def _swim(self, c):
        sub = c[1]
        self.log.append(sub)
        if sub == swim.SWIM_ENTER:
            self.mode = swim.MODE_SWIM
        elif sub == swim.SWIM_READBUFSIZE:
            self.pending = bytes([0x00, 0x18])   # 6144, little endian
        elif sub == swim.SWIM_READ_CAP:
            self.pending = bytes(8)
        elif sub == swim.SWIM_READSTATUS:
            self.pending = bytes([swim.SWIM_OK, 0, 0, 0])
        elif sub == swim.SWIM_READMEM:
            n = (c[2] << 8) | c[3]
            addr = (c[5] << 16) | (c[6] << 8) | c[7]
            self.readbuf = bytes(self._read(addr + i) for i in range(n))
        elif sub == swim.SWIM_READBUF:
            self.pending = self.readbuf
        elif sub == swim.SWIM_WRITEMEM:
            n = (c[2] << 8) | c[3]
            self.write_addr = (c[5] << 16) | (c[6] << 8) | c[7]
            first = c[8:8 + min(n, 8)]
            self._store(first)
            self.expect_data = n - len(first)
            if n == swim.BLOCK:
                self.block_writes += 1

    def _read(self, a):
        if self.locked and swim.FLASH_START <= a:
            return 0x71   # protected memory does not read back
        if a == swim.FLASH_IAPSR:
            return self.mem[a] | 0x04   # EOP always done in the simulation
        if a == swim.SWIM_CSR:
            return self.mem[a] | 0x02   # high speed possible
        return self.mem[a]

    def _store(self, data):
        for b in data:
            a = self.write_addr
            self.write_addr += 1
            if swim.OPT_START <= a < swim.OPT_START + 0x80:
                assert self.mem[swim.FLASH_CR2] == 0x80, "option byte written without OPT mode"
                self.opt_writes.append((a, b))
                if a == swim.OPT_START and b == 0x00 and self.locked:
                    # removing read-out protection erases flash and EEPROM
                    self.mem[swim.FLASH_START:] = bytes(0x10000 - swim.FLASH_START)
                    self.mem[swim.EEPROM_START:swim.EEPROM_START + swim.EEPROM_SIZE] = bytes(swim.EEPROM_SIZE)
                    self.locked = False
                self.mem[a] = b
            elif swim.FLASH_START <= a:
                assert not self.locked, "flash write on a locked chip"
                assert self.mem[swim.FLASH_CR2] in (0x01, 0x10), "flash written without block mode"
                self.mem[a] = b
            else:
                self.mem[a] = b


def write_ihx(path, start, data):
    with open(path, "w") as f:
        for off in range(0, len(data), 16):
            chunk = data[off:off + 16]
            a = start + off
            rec = bytes([len(chunk), a >> 8, a & 0xFF, 0]) + chunk
            f.write(":%s%02X\n" % (rec.hex().upper(), (-sum(rec)) & 0xFF))
        f.write(":00000001FF\n")


class SwimTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp()

    def _fw(self, size=1000):
        data = bytes((i * 7 + 3) & 0xFF for i in range(size))
        p = os.path.join(self.tmp, "fw.ihx")
        write_ihx(p, swim.FLASH_START, data)
        return p, data

    def test_probe_unlocked(self):
        info = swim.probe(FakeStm8StLink())
        self.assertTrue(info["probe"])
        self.assertTrue(info["target"])
        self.assertFalse(info["locked"])

    def test_probe_locked(self):
        info = swim.probe(FakeStm8StLink(locked=True))
        self.assertTrue(info["locked"])

    def test_flash_refuses_locked_chip(self):
        p, _ = self._fw()
        with self.assertRaises(swim.SwimError):
            swim.flash(p, log=lambda _l: None, transport=FakeStm8StLink(locked=True))

    def test_unlock_then_flash(self):
        t = FakeStm8StLink(locked=True)
        swim.unlock(log=lambda _l: None, transport=_Reuse(t))
        self.assertFalse(t.locked)
        self.assertEqual(bytes(t.mem[swim.OPT_START:swim.OPT_START + 15]), swim.KT_OPTION_BYTES)
        self.assertEqual(t.mem[swim.FLASH_START], 0)   # erased
        p, data = self._fw(1000)
        swim.flash(p, log=lambda _l: None, transport=_Reuse(t))
        self.assertEqual(bytes(t.mem[swim.FLASH_START:swim.FLASH_START + len(data)]), data)
        self.assertEqual(t.block_writes, 8)   # 1000 bytes -> 8 blocks of 128

    def test_reflash_writes_only_changed_blocks(self):
        t = FakeStm8StLink()
        t.mem[swim.OPT_START:swim.OPT_START + 15] = swim.KT_OPTION_BYTES
        p, data = self._fw(1024)
        swim.flash(p, log=lambda _l: None, transport=_Reuse(t))
        t.block_writes = 0
        changed = bytearray(data)
        changed[300] ^= 0xFF
        write_ihx(p, swim.FLASH_START, bytes(changed))
        swim.flash(p, log=lambda _l: None, transport=_Reuse(t))
        self.assertEqual(t.block_writes, 1)
        self.assertEqual(t.mem[swim.FLASH_START + 300], changed[300])

    def test_flash_sets_option_bytes_when_afr_missing(self):
        t = FakeStm8StLink()   # unlocked but AFR5 not set (e.g. a chip unlocked with another tool)
        p, _ = self._fw(200)
        swim.flash(p, log=lambda _l: None, transport=_Reuse(t))
        self.assertEqual(t.mem[swim.OPT_START + 3], 0x20)

    def test_eeprom_key_cleared(self):
        t = FakeStm8StLink()
        t.mem[swim.OPT_START:swim.OPT_START + 15] = swim.KT_OPTION_BYTES
        t.mem[swim.EEPROM_START] = 0x5A
        p, _ = self._fw(200)
        swim.flash(p, log=lambda _l: None, transport=_Reuse(t))
        self.assertEqual(t.mem[swim.EEPROM_START], 0)

    def test_settings_block_written_with_firmware_and_alone(self):
        import kunteng_core as kt
        t = FakeStm8StLink()
        t.mem[swim.OPT_START:swim.OPT_START + 15] = swim.KT_OPTION_BYTES
        t.mem[swim.SETTINGS_BLOCK_ADDR + 100] = 0x77   # something else in the same EEPROM block survives
        st = kt.default_settings()
        p, _ = self._fw(300)
        swim.flash(p, log=lambda _l: None, transport=_Reuse(t), settings_block=kt.settings_block(st))
        self.assertEqual(bytes(t.mem[swim.SETTINGS_BLOCK_ADDR:swim.SETTINGS_BLOCK_ADDR + 32]), kt.settings_block(st))
        self.assertEqual(t.mem[swim.SETTINGS_BLOCK_ADDR + 100], 0x77)
        st["motor_current_a"] = 11
        t.block_writes = 0
        swim.write_settings(kt.settings_block(st), log=lambda _l: None, transport=_Reuse(t))
        self.assertEqual(t.mem[swim.SETTINGS_BLOCK_ADDR + 5], 11)
        self.assertEqual(t.block_writes, 1)

    def test_settings_block_layout(self):
        import kunteng_core as kt
        b = kt.settings_block(kt.default_settings())
        self.assertEqual(len(b), 32)
        self.assertEqual(b[:4], bytes([0x4B, 0x54, 1, 32]))
        self.assertEqual(b[31], (~sum(b[:31])) & 0xFF)
        self.assertEqual(list(b[11:16]), [20, 40, 60, 80, 100])

    def test_firmware_build_output_parses(self):
        ihx = os.path.join(os.path.dirname(__file__), "..", "kunteng", "build", "main.ihx")
        if not os.path.isfile(ihx):
            self.skipTest("kunteng firmware not built")
        img = swim.flash_image(swim.read_ihx(ihx))
        self.assertEqual(len(img) % swim.BLOCK, 0)
        self.assertEqual(img[0], 0x82)   # STM8 reset vector: INT opcode


class _Reuse:
    """Wraps a fake so several Swim sessions can use the same simulated chip."""

    def __init__(self, fake):
        self.f = fake

    def write(self, d):
        self.f.write(d)

    def read(self, n):
        return self.f.read(n)


if __name__ == "__main__":
    unittest.main(verbosity=2)
