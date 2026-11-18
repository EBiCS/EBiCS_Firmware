import 'dart:typed_data';

import '../usb/usb_bridge.dart';

/// ST-Link V2 USB protocol (debug commands, SWD), as used by OpenOCD (src/jtag/drivers/stlink_usb.c),
/// the stlink tools and WebStlink. Every command is a 16 byte block on the bulk OUT endpoint; data and
/// status come back on the bulk IN endpoint.
class StLinkException implements Exception {
  final String message;
  StLinkException(this.message);
  @override
  String toString() => message;
}

class StLinkVersion {
  final int stlink, jtag, swim, vid, pid;
  StLinkVersion(this.stlink, this.jtag, this.swim, this.vid, this.pid);
  @override
  String toString() => 'V${stlink}J$jtag${swim > 0 ? 'S$swim' : ''}';
}

class StLink {
  // top level commands
  static const _getVersion = 0xF1;
  static const _debugCommand = 0xF2;
  static const _dfuCommand = 0xF3;
  static const _swimCommand = 0xF4;
  static const _getCurrentMode = 0xF5;
  static const _getTargetVoltage = 0xF7;

  // modes (GET_CURRENT_MODE)
  static const modeDfu = 0x00;
  static const modeMass = 0x01;
  static const modeDebug = 0x02;
  static const modeSwim = 0x03;

  // debug sub commands
  static const _readMem32 = 0x07;
  static const _writeMem32 = 0x08;
  static const _writeMem8 = 0x0D;
  static const _exit = 0x21;
  static const _readIdCodes = 0x31;
  static const _enterApiV2 = 0x30;
  static const _resetSys = 0x32;
  static const _readReg = 0x33;
  static const _writeReg = 0x34;
  static const _writeDebugReg = 0x35;
  static const _readDebugReg = 0x36;
  static const _getLastRwStatus = 0x3B;
  static const _getLastRwStatus2 = 0x3E;
  static const _swdSetFreq = 0x43;
  static const _readMem16 = 0x47;
  static const _writeMem16 = 0x48;
  static const _enterSwd = 0xA3;

  static const _ok = 0x80;
  static const _swdApWait = 0x10;
  static const _swdDpWait = 0x14;

  final UsbTransport usb;
  StLinkVersion? version;

  StLink(this.usb);

  bool get hasMem16 => (version?.jtag ?? 0) >= 26;
  bool get _hasRwStatus2 => (version?.jtag ?? 0) >= 15;
  bool get _hasSetFreq => (version?.jtag ?? 0) >= 22;
  bool get _hasVoltage => (version?.jtag ?? 0) >= 13;

  Future<Uint8List> _xfer(List<int> cmd, {int rx = 0, Uint8List? tx, int timeoutMs = 1000}) async {
    final c = Uint8List(16)..setRange(0, cmd.length, cmd);
    await usb.bulkOut(c, timeoutMs: timeoutMs);
    if (tx != null && tx.isNotEmpty) await usb.bulkOut(tx, timeoutMs: timeoutMs);
    if (rx > 0) {
      final r = await usb.bulkIn(rx, timeoutMs: timeoutMs);
      if (r.length < rx) throw StLinkException('short answer from the ST-Link (${r.length} of $rx bytes)');
      return r;
    }
    return Uint8List(0);
  }

  static List<int> _u32(int v) => [v & 0xFF, (v >> 8) & 0xFF, (v >> 16) & 0xFF, (v >> 24) & 0xFF];
  static List<int> _u16(int v) => [v & 0xFF, (v >> 8) & 0xFF];
  static int _le32(Uint8List b, int o) => b[o] | (b[o + 1] << 8) | (b[o + 2] << 16) | (b[o + 3] << 24);

  static String statusText(int s) => switch (s) {
        0x80 => 'ok',
        0x81 => 'target fault',
        0x09 => 'no answer from the controller (SWD)',
        0x10 || 0x14 => 'SWD wait',
        0x11 || 0x15 => 'SWD fault',
        0x12 || 0x16 => 'SWD parity error',
        0x17 => 'SWD acknowledge error',
        0x18 || 0x19 => 'SWD sequence error',
        0x1A || 0x1B => 'SWD access error',
        _ => 'error 0x${s.toRadixString(16)}',
      };

  void _check(int status, String what) {
    if (status != _ok) throw StLinkException('$what: ${statusText(status)}');
  }

  // ------------------------------------------------------------------ connection

  Future<StLinkVersion> readVersion() async {
    final r = await _xfer([_getVersion], rx: 6);
    final v = (r[0] << 8) | r[1];
    return version = StLinkVersion((v >> 12) & 0xF, (v >> 6) & 0x3F, v & 0x3F, r[2] | (r[3] << 8), r[4] | (r[5] << 8));
  }

  Future<int> currentMode() async => (await _xfer([_getCurrentMode], rx: 2))[0];

  /// Target voltage in volts (0 if the ST-Link can't measure it).
  Future<double> targetVoltage() async {
    if (!_hasVoltage) return 0;
    final r = await _xfer([_getTargetVoltage], rx: 8);
    final a0 = _le32(r, 0), a1 = _le32(r, 4);
    return a0 == 0 ? 0 : 2 * a1 * 1.2 / a0;
  }

  /// Leaves DFU/mass storage/SWIM/debug mode and enters SWD debug mode, without resetting the target.
  /// [freqDivisor]: 0 = 4 MHz, 1 = 1.8 MHz, 3 = 950 kHz, 7 = 480 kHz, 15 = 240 kHz, 31 = 125 kHz.
  Future<void> enterSwd({int freqDivisor = 3}) async {
    await readVersion();
    if (version!.stlink != 2 && version!.stlink != 3) {
      throw StLinkException('unsupported ST-Link version $version (only ST-Link V2/V2-1/V3)');
    }
    var mode = await currentMode();
    if (mode == modeDfu) await _xfer([_dfuCommand, 0x07]);
    if (mode == modeSwim) await _xfer([_swimCommand, 0x01]);
    if (mode == modeDebug) await _xfer([_debugCommand, _exit]);
    if (_hasSetFreq && version!.stlink == 2) {
      final r = await _xfer([_debugCommand, _swdSetFreq, freqDivisor], rx: 2);
      _check(r[0], 'setting the SWD speed');
    }
    final r = await _xfer([_debugCommand, _enterApiV2, _enterSwd], rx: 2, timeoutMs: 3000);
    _check(r[0], 'connecting to the controller');
  }

  /// SWD-DP IDCODE (0x1BA01477 for Cortex-M3).
  Future<int> readIdCode() async {
    final r = await _xfer([_debugCommand, _readIdCodes], rx: 12);
    _check(r[0], 'reading the chip id');
    return _le32(r, 4);
  }

  Future<void> exitDebug() async {
    try {
      await _xfer([_debugCommand, _exit]);
    } catch (_) {}
  }

  // ------------------------------------------------------------------ memory

  Future<void> _rwStatus(String what) async {
    if (_hasRwStatus2) {
      final r = await _xfer([_debugCommand, _getLastRwStatus2], rx: 12);
      _check(r[0], what);
    } else {
      final r = await _xfer([_debugCommand, _getLastRwStatus], rx: 2);
      _check(r[0], what);
    }
  }

  /// Reads [length] bytes (multiple of 4) from a 4-aligned address, max 1024 bytes and no 1 KB crossing per
  /// transfer (the AHB-AP auto-increment wraps at 1 KB): larger reads are split here.
  Future<Uint8List> readMem(int addr, int length) async {
    if (addr & 3 != 0 || length & 3 != 0) throw ArgumentError('readMem needs 4-aligned address and length');
    final out = Uint8List(length);
    var done = 0;
    while (done < length) {
      final a = addr + done;
      final n = _chunk(a, length - done);
      final r = await _retry(() async {
        final d = await _xfer([_debugCommand, _readMem32, ..._u32(a), ..._u16(n)], rx: n);
        await _rwStatus('reading memory at 0x${a.toRadixString(16)}');
        return d;
      });
      out.setRange(done, done + n, r);
      done += n;
    }
    return out;
  }

  /// Writes bytes (multiple of 4) to a 4-aligned address with 32-bit accesses.
  Future<void> writeMem(int addr, Uint8List data) async {
    if (addr & 3 != 0 || data.length & 3 != 0) throw ArgumentError('writeMem needs 4-aligned address and length');
    var done = 0;
    while (done < data.length) {
      final a = addr + done;
      final n = _chunk(a, data.length - done);
      await _retry(() async {
        await _xfer([_debugCommand, _writeMem32, ..._u32(a), ..._u16(n)],
            tx: Uint8List.sublistView(data, done, done + n));
        await _rwStatus('writing memory at 0x${a.toRadixString(16)}');
      });
      done += n;
    }
  }

  /// Byte writes (max 64 bytes), e.g. for single uint8_t variables.
  Future<void> writeMem8(int addr, Uint8List data) async {
    if (data.length > 64) throw ArgumentError('writeMem8: max 64 bytes');
    await _retry(() async {
      await _xfer([_debugCommand, _writeMem8, ..._u32(addr), ..._u16(data.length)], tx: data);
      await _rwStatus('writing memory at 0x${addr.toRadixString(16)}');
    });
  }

  /// 16-bit writes (ST-Link firmware J26 or newer).
  Future<void> writeMem16(int addr, Uint8List data) async {
    if (!hasMem16) throw StLinkException('this ST-Link firmware has no 16-bit memory access');
    await _retry(() async {
      await _xfer([_debugCommand, _writeMem16, ..._u32(addr), ..._u16(data.length)], tx: data);
      await _rwStatus('writing memory at 0x${addr.toRadixString(16)}');
    });
  }

  Future<Uint8List> readMem16(int addr, int length) async {
    if (!hasMem16) throw StLinkException('this ST-Link firmware has no 16-bit memory access');
    return _retry(() async {
      final d = await _xfer([_debugCommand, _readMem16, ..._u32(addr), ..._u16(length)], rx: length);
      await _rwStatus('reading memory at 0x${addr.toRadixString(16)}');
      return d;
    });
  }

  static int _chunk(int addr, int remaining) {
    final toBoundary = 0x400 - (addr & 0x3FF);
    return remaining < toBoundary ? remaining : toBoundary;
  }

  Future<int> read32(int addr) async {
    final r = await _retry(() async {
      final r = await _xfer([_debugCommand, _readDebugReg, ..._u32(addr)], rx: 8);
      _check(r[0], 'reading 0x${addr.toRadixString(16)}');
      return r;
    });
    return _le32(r, 4) & 0xFFFFFFFF;
  }

  Future<void> write32(int addr, int value) async {
    await _retry(() async {
      final r = await _xfer([_debugCommand, _writeDebugReg, ..._u32(addr), ..._u32(value)], rx: 2);
      _check(r[0], 'writing 0x${addr.toRadixString(16)}');
    });
  }

  // ------------------------------------------------------------------ core registers

  /// Core register: 0-12 = R0-R12, 13 = SP, 14 = LR, 15 = PC, 16 = xPSR, 17 = MSP, 18 = PSP.
  Future<int> readReg(int index) async {
    final r = await _xfer([_debugCommand, _readReg, index], rx: 8);
    _check(r[0], 'reading core register $index');
    return _le32(r, 4) & 0xFFFFFFFF;
  }

  Future<void> writeReg(int index, int value) async {
    final r = await _xfer([_debugCommand, _writeReg, index, ..._u32(value)], rx: 2);
    _check(r[0], 'writing core register $index');
  }

  Future<void> resetSys() async {
    final r = await _xfer([_debugCommand, _resetSys], rx: 2);
    _check(r[0], 'reset');
  }

  Future<T> _retry<T>(Future<T> Function() f) async {
    for (var i = 0;; i++) {
      try {
        return await f();
      } on StLinkException catch (e) {
        final wait = e.message.contains(statusText(_swdApWait)) || e.message.contains(statusText(_swdDpWait));
        if (!wait || i >= 5) rethrow;
        await Future<void>.delayed(const Duration(milliseconds: 10));
      }
    }
  }
}
