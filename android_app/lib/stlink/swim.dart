import 'dart:typed_data';

import '../usb/usb_bridge.dart';

/// SWIM access to the STM8S105 of Kunteng (KT) controllers through the ST-Link V2.
/// Same protocol and steps as setup_app/swim.py (and the open source stm8flash tool).
class SwimException implements Exception {
  final String message;
  SwimException(this.message);
  @override
  String toString() => message;
}

class Swim {
  static const _getVersion = 0xF1, _debug = 0xF2, _dfu = 0xF3, _swim = 0xF4, _getCurrentMode = 0xF5;
  static const modeDfu = 0, modeMass = 1, modeDebug = 2, modeSwim = 3, modeBootloader = 4;
  static const swimEnter = 0x00,
      swimExit = 0x01,
      swimReadCap = 0x02,
      swimSpeed = 0x03,
      swimEnterSeq = 0x04,
      swimGenRst = 0x05,
      swimReset = 0x06,
      swimAssertReset = 0x07,
      swimDeassertReset = 0x08,
      swimReadStatus = 0x09,
      swimWriteMem = 0x0A,
      swimReadMem = 0x0B,
      swimReadBuf = 0x0C,
      swimReadBufSize = 0x0D;

  // STM8S registers and memory
  static const swimCsr = 0x7F80, clkCkdivr = 0x50C6;
  static const flashCr2 = 0x505B, flashNcr2 = 0x505C, flashIapsr = 0x505F, flashPukr = 0x5062, flashDukr = 0x5064;
  static const dmCsr2 = 0x7F99;
  static const flashStart = 0x8000, flashSize = 0x8000, block = 128;
  static const eepromStart = 0x4000, eepromSize = 0x400, optStart = 0x4800;

  /// Option bytes for the KT board: ROP off, AFR5 (TIM1_CHxN on PB0..PB2), defaults with complements.
  static final ktOptionBytes = Uint8List.fromList([
    0x00,
    0x00,
    0xFF,
    0x20,
    0xDF,
    0x00,
    0xFF,
    0x00,
    0xFF,
    0x00,
    0xFF,
    0x00,
    0xFF,
    0x00,
    0xFF,
  ]);

  final UsbTransport usb;
  final Future<void> Function(Duration) _sleep;
  int readBufSize = 6144;
  int swimVersion = 0;

  Swim(this.usb, {Future<void> Function(Duration)? sleep}) : _sleep = sleep ?? Future<void>.delayed;

  // ------------------------------------------------------------------ low level
  Future<Uint8List> _cmd(List<int> args, {Uint8List? data, int rx = 0}) async {
    final payload = <int>[...args, ...?data];
    final head = Uint8List(16)..setRange(0, payload.length < 16 ? payload.length : 16, payload);
    await usb.bulkOut(head, timeoutMs: 3000);
    if (payload.length > 16) await usb.bulkOut(Uint8List.fromList(payload.sublist(16)), timeoutMs: 3000);
    if (rx > 0) {
      final r = await usb.bulkIn(rx, timeoutMs: 3000);
      if (r.length < rx) throw SwimException('short answer from the ST-Link');
      return r;
    }
    return Uint8List(0);
  }

  /// SWIM command, then poll the status until the ST-Link reports done.
  Future<void> _swimCmd(List<int> args, {Uint8List? data}) async {
    await _cmd([_swim, ...args], data: data);
    Uint8List? last;
    var stalls = 0;
    for (var i = 0; i < 400; i++) {
      final st = await _cmd([_swim, swimReadStatus], rx: 4);
      if (st[0] == 0) return;
      if (last != null && _same(st, last)) {
        if (++stalls >= 4) break;
      } else {
        stalls = 0;
      }
      last = st;
      await _sleep(const Duration(milliseconds: 10));
    }
    throw SwimException(
      'SWIM command 0x${args[0].toRadixString(16)} failed. Check the wiring (SWIM, GND, RST) '
      'and that the controller is powered.',
    );
  }

  static bool _same(Uint8List a, Uint8List b) {
    for (var i = 0; i < a.length; i++) {
      if (a[i] != b[i]) return false;
    }
    return true;
  }

  static List<int> _addr(int a) => [0, (a >> 16) & 0xFF, (a >> 8) & 0xFF, a & 0xFF];

  Future<Uint8List> readMem(int addr, int length) async {
    final out = BytesBuilder();
    while (length > 0) {
      final n = length < readBufSize ? length : readBufSize;
      await _swimCmd([swimReadMem, n >> 8, n & 0xFF, ..._addr(addr)]);
      await _cmd([_swim, swimReadBuf]);
      out.add(await usb.bulkIn(n, timeoutMs: 3000));
      addr += n;
      length -= n;
    }
    return out.toBytes();
  }

  Future<int> readByte(int addr) async => (await readMem(addr, 1))[0];

  Future<void> writeMem(int addr, Uint8List data) =>
      _swimCmd([swimWriteMem, data.length >> 8, data.length & 0xFF, ..._addr(addr)], data: data);

  Future<void> writeByte(int addr, int v) => writeMem(addr, Uint8List.fromList([v & 0xFF]));

  // ------------------------------------------------------------------ session
  Future<void> open() async {
    final v = await _cmd([_getVersion], rx: 6);
    final ver = (v[0] << 8) | v[1];
    swimVersion = ver & 0x3F;
    if (swimVersion == 0) throw SwimException('this ST-Link has no SWIM support');
    final mode = (await _cmd([_getCurrentMode], rx: 2))[0];
    if (mode == modeDebug) await _cmd([_debug, 0x21]);
    if (mode == modeDfu || mode == modeMass || mode == modeBootloader) await _cmd([_dfu, 0x07]);
    if (mode != modeSwim) await _cmd([_swim, swimEnter]);
    final bs = await _cmd([_swim, swimReadBufSize], rx: 2);
    readBufSize = (bs[0] | (bs[1] << 8)) == 0 ? 6144 : (bs[0] | (bs[1] << 8));
    await _cmd([_swim, swimReadCap, 0x01], rx: 8);
    await _swimCmd([swimAssertReset]);
    await _swimCmd([swimEnterSeq]);
    await writeByte(swimCsr, 0xA1); // mask interrupts, whole memory, SWIM priority, stall the CPU
    await _swimCmd([swimDeassertReset]);
    await _sleep(const Duration(milliseconds: 1));
    var csr = 0;
    for (var i = 0; i < 10; i++) {
      csr = await readByte(swimCsr);
      if (csr & 0x02 != 0) break;
      await _sleep(const Duration(milliseconds: 1));
    }
    await _swimCmd([swimReset]);
    if (csr & 0x02 != 0) {
      await writeByte(swimCsr, csr | 0x10);
      await _swimCmd([swimSpeed, 1]);
    }
  }

  Future<void> resetTarget() async {
    await writeByte(swimCsr, (await readByte(swimCsr)) | 0x04);
    await _swimCmd([swimGenRst]);
  }

  Future<void> close() async {
    try {
      await _cmd([_swim, swimExit]);
    } catch (_) {}
  }

  // ------------------------------------------------------------------ programming
  Future<void> _waitEop(String what) async {
    for (var i = 0; i < 10; i++) {
      final sr = await readByte(flashIapsr);
      if (sr & 0x04 != 0) return;
      if (sr & 0x01 != 0) throw SwimException('$what: write protected - unlock the chip first');
      await _sleep(const Duration(milliseconds: 3));
    }
    throw SwimException('$what: programming timeout');
  }

  Future<void> _lock() async => writeByte(flashIapsr, (await readByte(flashIapsr)) & ~0x0A);

  /// Programs flash or EEPROM in 128 byte blocks; blocks that already have the right content are skipped.
  Future<void> writeBlocks(int start, Uint8List data, {void Function(int done, int total)? progress}) async {
    if (start % block != 0) throw SwimException('start address must be block aligned');
    if (data.length % block != 0) {
      data = Uint8List(data.length + block - data.length % block)..setRange(0, data.length, data);
    }
    final isFlash = start >= flashStart;
    await writeByte(clkCkdivr, 0x00);
    if (isFlash) {
      await writeByte(flashPukr, 0x56);
      await writeByte(flashPukr, 0xAE);
    } else {
      await writeByte(flashDukr, 0xAE);
      await writeByte(flashDukr, 0x56);
    }
    final current = await readMem(start, data.length);
    final n = data.length ~/ block;
    for (var i = 0; i < n; i++) {
      final blk = Uint8List.sublistView(data, i * block, (i + 1) * block);
      final old = Uint8List.sublistView(current, i * block, (i + 1) * block);
      if (!_same(blk, old)) {
        final fast = old.every((b) => b == 0);
        final mode = fast ? 0x10 : 0x01;
        await writeByte(dmCsr2, (await readByte(dmCsr2)) | 0x08);
        await writeByte(flashCr2, mode);
        await writeByte(flashNcr2, ~mode & 0xFF);
        await writeMem(start + i * block, Uint8List.fromList(blk));
        await _sleep(Duration(milliseconds: fast ? 3 : 6));
        await _waitEop('block 0x${(start + i * block).toRadixString(16)}');
      }
      progress?.call(i + 1, n);
    }
    await _lock();
  }

  Future<void> writeOptionBytes([Uint8List? values]) async {
    values ??= ktOptionBytes;
    await writeByte(clkCkdivr, 0x00);
    await writeByte(flashDukr, 0xAE);
    await writeByte(flashDukr, 0x56);
    await writeByte(flashCr2, 0x80);
    await writeByte(flashNcr2, 0x7F);
    for (var i = 0; i < values.length; i++) {
      await writeByte(optStart + i, values[i]);
      await _sleep(const Duration(milliseconds: 7));
      await _waitEop('option byte $i');
    }
    await _lock();
  }

  Future<Uint8List> readOptionBytes([int n = 15]) => readMem(optStart, n);
}

/// Intel hex (SDCC .ihx) -> flash image from 0x8000, padded to whole 128 byte blocks with 0x00.
Uint8List ihxToFlashImage(String ihx) {
  final mem = <int, int>{};
  var base = 0;
  for (var line in ihx.split('\n')) {
    line = line.trim();
    if (!line.startsWith(':')) continue;
    final raw = <int>[for (var i = 1; i + 1 < line.length; i += 2) int.parse(line.substring(i, i + 2), radix: 16)];
    if (raw.fold<int>(0, (a, b) => a + b) & 0xFF != 0) throw SwimException('hex file checksum error');
    final n = raw[0], addr = (raw[1] << 8) | raw[2], type = raw[3];
    if (type == 0) {
      for (var i = 0; i < n; i++) {
        mem[base + addr + i] = raw[4 + i];
      }
    } else if (type == 1) {
      break;
    } else if (type == 4) {
      base = ((raw[4] << 8) | raw[5]) << 16;
    }
  }
  final addrs = mem.keys.where((a) => a >= Swim.flashStart && a < Swim.flashStart + Swim.flashSize).toList();
  if (addrs.isEmpty) throw SwimException('hex file has no STM8 flash data');
  var end = addrs.reduce((a, b) => a > b ? a : b) + 1;
  end = (end + Swim.block - 1) ~/ Swim.block * Swim.block;
  final img = Uint8List(end - Swim.flashStart);
  for (final a in addrs) {
    img[a - Swim.flashStart] = mem[a]!;
  }
  return img;
}
