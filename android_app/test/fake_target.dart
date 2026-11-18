import 'dart:typed_data';

import 'package:ebics_setup/usb/usb_bridge.dart';

/// Simulated ST-Link V2 (J37) wired to an STM32F103C6: answers the USB protocol, emulates flash, SRAM,
/// the flash controller (FPEC), the debug registers and the RAM flash loader. Used to test the flashing
/// code without hardware.
class FakeTarget implements UsbTransport {
  static const flashBase = 0x08000000, flashSize = 0x8000;
  static const sramBase = 0x20000000, sramSize = 0x2800;

  final flash = Uint8List(flashSize)..fillRange(0, flashSize, 0xFF);
  final sram = Uint8List(sramSize);
  final optionBytes = Uint8List(16)..fillRange(0, 16, 0xFF);
  final regs = List<int>.filled(21, 0);
  final erasedPages = <int>[];
  final programmedRanges = <(int, int)>[];
  bool rdp = false;
  bool halted = false;
  bool flashLocked = true;
  bool optUnlocked = false;
  int _keyStep = 0, _optKeyStep = 0;
  int cr = 0x80, sr = 0, ar = 0, demcr = 0, dbgmcuCr = 0;
  int loaderRuns = 0;

  Uint8List? _pendingIn;
  List<int>? _pendingWrite; // [kind, addr, len]

  FakeTarget() {
    // emulated EEPROM pages with a pattern, must survive everything
    for (var i = 0x7800; i < flashSize; i++) {
      flash[i] = 0xA0 ^ (i & 0xFF);
    }
  }

  static int _le32(Uint8List b, int o) => b[o] | (b[o + 1] << 8) | (b[o + 2] << 16) | (b[o + 3] << 24);
  static Uint8List _u32(int v) => Uint8List.fromList([v & 0xFF, (v >> 8) & 0xFF, (v >> 16) & 0xFF, (v >> 24) & 0xFF]);

  Uint8List _ok([int len = 2]) => Uint8List(len)..[0] = 0x80;

  @override
  Future<void> bulkOut(Uint8List data, {int timeoutMs = 1000}) async {
    final pw = _pendingWrite;
    if (pw != null) {
      _pendingWrite = null;
      final addr = pw[1];
      for (var i = 0; i < data.length; i++) {
        _writeByte(addr + i, data[i]);
      }
      if (pw[0] == 32) {
        for (var i = 0; i + 3 < data.length; i += 4) {
          _onWrite32(addr + i, _le32(data, i));
        }
      }
      return;
    }
    final c = data;
    switch (c[0]) {
      case 0xF1:
        const v = (2 << 12) | (37 << 6) | 7;
        _pendingIn = Uint8List.fromList([v >> 8, v & 0xFF, 0x83, 0x04, 0x48, 0x37]);
      case 0xF5:
        _pendingIn = Uint8List.fromList([0x01, 0x00]);
      case 0xF3:
        break;
      case 0xF7:
        _pendingIn = Uint8List.fromList([..._u32(1000), ..._u32(1375)]);
      case 0xF2:
        _debug(c);
      default:
        throw StateError('unknown command 0x${c[0].toRadixString(16)}');
    }
  }

  void _debug(Uint8List c) {
    final addr = _le32(c, 2);
    final len = c[6] | (c[7] << 8);
    switch (c[1]) {
      case 0x21: // exit
        break;
      case 0x43: // freq
      case 0x30: // enter swd
      case 0x32: // reset sys
        _pendingIn = _ok();
      case 0x31:
        _pendingIn = Uint8List(12)
          ..[0] = 0x80
          ..setRange(4, 8, _u32(0x1BA01477));
      case 0x07: // read mem 32
        if (addr & 3 != 0 || len & 3 != 0) throw StateError('unaligned 32-bit read');
        if ((addr & 0x3FF) + len > 0x400) throw StateError('read crosses 1 KB');
        _pendingIn = Uint8List.fromList(List.generate(len, (i) => _readByte(addr + i)));
      case 0x08: // write mem 32
        if (addr & 3 != 0 || len & 3 != 0) throw StateError('unaligned 32-bit write');
        if ((addr & 0x3FF) + len > 0x400) throw StateError('write crosses 1 KB');
        _pendingWrite = [32, addr, len];
      case 0x0D: // write mem 8
        _pendingWrite = [8, addr, len];
      case 0x3E:
        _pendingIn = _ok(12);
      case 0x3B:
        _pendingIn = _ok();
      case 0x35: // write debug reg
        _onWrite32(addr, _le32(c, 6));
        _pendingIn = _ok();
      case 0x36: // read debug reg
        _pendingIn = Uint8List(8)
          ..[0] = 0x80
          ..setRange(4, 8, _u32(_read32(addr)));
      case 0x33: // read reg
        _pendingIn = Uint8List(8)
          ..[0] = 0x80
          ..setRange(4, 8, _u32(regs[c[2]]));
      case 0x34: // write reg
        regs[c[2]] = _le32(c, 3);
        _pendingIn = _ok();
      default:
        throw StateError('unknown debug command 0x${c[1].toRadixString(16)}');
    }
  }

  @override
  Future<Uint8List> bulkIn(int length, {int timeoutMs = 1000}) async {
    final r = _pendingIn;
    _pendingIn = null;
    if (r == null) throw UsbException('nothing to read');
    return r.length > length ? Uint8List.sublistView(r, 0, length) : r;
  }

  // ------------------------------------------------------------------ memory

  int _readByte(int a) {
    if (a >= flashBase && a < flashBase + flashSize) return rdp ? 0 : flash[a - flashBase];
    if (a >= sramBase && a < sramBase + sramSize) return sram[a - sramBase];
    if (a >= 0x1FFFF800 && a < 0x1FFFF810) return optionBytes[a - 0x1FFFF800];
    final w = _read32(a & ~3);
    return (w >> (8 * (a & 3))) & 0xFF;
  }

  void _writeByte(int a, int v) {
    if (a >= sramBase && a < sramBase + sramSize) sram[a - sramBase] = v;
    if (a >= flashBase && a < flashBase + flashSize) throw StateError('direct write into flash at 0x${a.toRadixString(16)}');
  }

  int _read32(int a) => switch (a) {
        0xE000EDF0 => (halted ? (1 << 17) : 0) | 1,
        0xE000EDFC => demcr,
        0xE0042000 => 0x20036412, // low density, rev
        0xE0042004 => dbgmcuCr,
        0x1FFFF7E0 => 32,
        0x40022010 => cr | (flashLocked ? 0x80 : 0) | (optUnlocked ? 0x200 : 0),
        0x4002200C => sr,
        0x4002201C => rdp ? 0x2 : 0,
        0x40022020 => 0xFFFFFFFF,
        _ => (a >= sramBase && a < sramBase + sramSize) || (a >= flashBase && a < flashBase + flashSize)
            ? _readByte(a) | (_readByte(a + 1) << 8) | (_readByte(a + 2) << 16) | (_readByte(a + 3) << 24)
            : 0,
      };

  void _onWrite32(int a, int v) {
    switch (a) {
      case 0xE000EDF0: // DHCSR
        if ((v >> 16) != 0xA05F) return;
        if (v & 2 != 0) {
          halted = true;
        } else if (v & 1 != 0 || v == 0xA05F0000) {
          if (regs[15] == 0x20000000 && halted) {
            _runLoader();
            halted = true;
          } else {
            halted = false;
          }
        }
      case 0xE000EDFC:
        demcr = v;
      case 0xE000ED0C:
        if (v == 0x05FA0004) {
          halted = (demcr & 1) != 0;
          flashLocked = true;
          optUnlocked = false;
          cr = 0;
          regs[15] = 0x08000000;
        }
      case 0xE0042004:
        dbgmcuCr = v;
      case 0x40022004: // KEYR
        if (_keyStep == 0 && v == 0x45670123) {
          _keyStep = 1;
        } else if (_keyStep == 1 && v == 0xCDEF89AB) {
          flashLocked = false;
          _keyStep = 0;
        } else {
          _keyStep = 0;
        }
      case 0x40022008: // OPTKEYR
        if (_optKeyStep == 0 && v == 0x45670123) {
          _optKeyStep = 1;
        } else if (_optKeyStep == 1 && v == 0xCDEF89AB) {
          optUnlocked = true;
          _optKeyStep = 0;
        }
      case 0x4002200C:
        sr &= ~(v & 0x34);
      case 0x40022014:
        ar = v;
      case 0x40022010:
        if (v & 0x80 != 0) {
          flashLocked = true;
          optUnlocked = false;
          cr = 0;
          return;
        }
        if (flashLocked) return;
        cr = v & 0x7F;
        if (v & 0x40 != 0) {
          if (v & 0x2 != 0) {
            final page = (ar - flashBase) ~/ 1024;
            erasedPages.add(page);
            flash.fillRange(page * 1024, page * 1024 + 1024, 0xFF);
            sr |= 0x20;
          } else if (v & 0x20 != 0 && optUnlocked) {
            if (rdp) flash.fillRange(0, flashSize, 0xFF); // RDP removal: mass erase
            optionBytes.fillRange(0, 16, 0xFF);
            sr |= 0x20;
          }
        }
    }
  }

  void _runLoader() {
    loaderRuns++;
    var src = regs[0], dst = regs[1], n = regs[2];
    final start = dst;
    while (n > 0) {
      final lo = sram[src - sramBase], hi = sram[src - sramBase + 1];
      src += 2;
      if (cr & 0x1 != 0 && dst >= flashBase && dst < flashBase + flashSize) {
        final o = dst - flashBase;
        if (flash[o] != 0xFF || flash[o + 1] != 0xFF) {
          sr |= 0x4; // PGERR
          break;
        }
        flash[o] = lo;
        flash[o + 1] = hi;
      } else if (cr & 0x10 != 0 && dst >= 0x1FFFF800 && dst < 0x1FFFF810) {
        final o = dst - 0x1FFFF800;
        optionBytes[o] = lo;
        optionBytes[o + 1] = (~lo) & 0xFF; // complement written by the flash controller
      } else {
        sr |= 0x10;
        break;
      }
      dst += 2;
      n--;
    }
    programmedRanges.add((start, dst));
    regs[0] = src;
    regs[1] = dst;
    regs[2] = n;
  }

  /// RDP state after a power cycle
  void powerCycle() {
    rdp = optionBytes[0] != 0xA5;
    halted = false;
  }
}
