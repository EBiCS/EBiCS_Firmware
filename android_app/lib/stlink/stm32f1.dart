import 'dart:typed_data';

import 'stlink.dart';

/// STM32F103 (and GD32F103 clones) over SWD: halt/reset, flash programming, read-out protection.
///
/// Flash programming uses a tiny loader in RAM that copies half-words into the flash (like OpenOCD's
/// stm32f1x algorithm), because the flash only accepts 16-bit writes and many ST-Link V2 clones have no
/// 16-bit memory access.
///
/// SAFETY: pages 30 and 31 (0x08007800..0x08007FFF) hold the emulated EEPROM with the hall angles and KV.
/// Nothing in this class erases or writes them: every erase/write is checked against [protectedStart].
class Stm32F1 {
  static const flashBase = 0x08000000;
  static const pageSize = 1024;
  static const protectedStart = 0x08007800; // emulated EEPROM: never touch
  static const sramBase = 0x20000000;
  static const sramEnd = 0x20002800; // 10 KB (STM32F103x6)

  // Cortex-M debug registers
  static const _dhcsr = 0xE000EDF0;
  static const _demcr = 0xE000EDFC;
  static const _aircr = 0xE000ED0C;
  static const _dbgKey = 0xA05F0000;
  static const _cHalt = 1 << 1, _cDebugEn = 1 << 0;
  static const _sHalt = 1 << 17;
  static const _dbgmcuIdcode = 0xE0042000;
  static const _dbgmcuCr = 0xE0042004;
  static const _flashSizeReg = 0x1FFFF7E0;

  // FPEC
  static const _fBase = 0x40022000;
  static const _keyr = _fBase + 0x04, _optkeyr = _fBase + 0x08, _sr = _fBase + 0x0C, _cr = _fBase + 0x10;
  static const _ar = _fBase + 0x14, _obr = _fBase + 0x1C, _wrpr = _fBase + 0x20;
  static const _key1 = 0x45670123, _key2 = 0xCDEF89AB;
  static const _srBsy = 1 << 0, _srPgErr = 1 << 2, _srWrPrtErr = 1 << 4, _srEop = 1 << 5;
  static const _crPg = 1 << 0, _crPer = 1 << 1, _crOptPg = 1 << 4, _crOptEr = 1 << 5, _crStrt = 1 << 6;
  static const _crLock = 1 << 7, _crOptWre = 1 << 9;
  static const optionBytes = 0x1FFFF800;

  // RAM layout while flashing (firmware is halted)
  static const _loaderAddr = 0x20000000;
  static const _bufferAddr = 0x20000100;
  static const _loader = [
    0x30, 0xF8, 0x02, 0x4B, // ldrh.w r4, [r0], #2
    0x21, 0xF8, 0x02, 0x4B, // strh.w r4, [r1], #2
    0x1D, 0x68, //             wait: ldr r5, [r3]
    0x15, 0xF0, 0x01, 0x0F, // tst.w r5, #1      (BSY)
    0xFB, 0xD1, //             bne wait
    0x15, 0xF0, 0x14, 0x0F, // tst.w r5, #0x14   (PGERR | WRPRTERR)
    0x01, 0xD1, //             bne done
    0x01, 0x3A, //             subs r2, #1
    0xF2, 0xD1, //             bne start
    0x00, 0xBE, //             done: bkpt #0
    0x00, 0xBF, 0x00, 0xBF, // nop, nop (pad to 32 bytes)
  ];

  final StLink st;
  bool _loaderLoaded = false;
  Stm32F1(this.st);

  // ------------------------------------------------------------------ info

  Future<ChipInfo> info() async {
    final id = await st.read32(_dbgmcuIdcode);
    int flashKb = 0;
    try {
      flashKb = (await st.read32(_flashSizeReg)) & 0xFFFF;
    } catch (_) {}
    final obr = await st.read32(_obr);
    return ChipInfo(devId: id & 0xFFF, revId: (id >> 16) & 0xFFFF, flashKb: flashKb, readProtected: (obr & 0x2) != 0);
  }

  Future<bool> readProtected() async => ((await st.read32(_obr)) & 0x2) != 0;

  // ------------------------------------------------------------------ core control

  Future<void> halt() async {
    await st.write32(_dhcsr, _dbgKey | _cHalt | _cDebugEn);
    await _waitHalted(const Duration(milliseconds: 500));
  }

  Future<bool> isHalted() async => ((await st.read32(_dhcsr)) & _sHalt) != 0;

  Future<void> _waitHalted(Duration timeout) async {
    final sw = Stopwatch()..start();
    while (!await isHalted()) {
      if (sw.elapsed > timeout) throw StLinkException('the controller did not stop (halt)');
      await Future<void>.delayed(const Duration(milliseconds: 5));
    }
  }

  /// System reset, core stopped at the reset vector (firmware does not run).
  Future<void> resetHalt() async {
    await halt();
    final demcr = await st.read32(_demcr);
    await st.write32(_demcr, demcr | 1); // VC_CORERESET
    await st.write32(_aircr, 0x05FA0004); // SYSRESETREQ
    await Future<void>.delayed(const Duration(milliseconds: 50));
    try {
      await _waitHalted(const Duration(milliseconds: 500));
    } on StLinkException {
      await halt();
    }
    await st.write32(_demcr, demcr & ~1);
    // stop the watchdogs while the core is halted
    final cr = await st.read32(_dbgmcuCr);
    await st.write32(_dbgmcuCr, cr | (1 << 8) | (1 << 9));
    _loaderLoaded = false;
  }

  /// System reset, firmware runs (debug stays connected for live data).
  Future<void> resetRun() async {
    final demcr = await st.read32(_demcr);
    await st.write32(_demcr, demcr & ~1); // no halt at the reset vector
    await st.write32(_aircr, 0x05FA0004); // reset: PC = firmware reset handler, flash controller locked
    await Future<void>.delayed(const Duration(milliseconds: 50));
    await st.write32(_dhcsr, _dbgKey | _cDebugEn); // clear a halt request that survived the reset: run
    await Future<void>.delayed(const Duration(milliseconds: 50));
    _loaderLoaded = false;
  }

  // ------------------------------------------------------------------ FPEC

  Future<void> _unlockFlash() async {
    if ((await st.read32(_cr)) & _crLock != 0) {
      await st.write32(_keyr, _key1);
      await st.write32(_keyr, _key2);
      if ((await st.read32(_cr)) & _crLock != 0) throw StLinkException('could not unlock the flash');
    }
    await st.write32(_sr, _srEop | _srPgErr | _srWrPrtErr); // clear flags
  }

  Future<void> _lockFlash() async {
    await st.write32(_cr, _crLock);
  }

  Future<int> _waitBusy(Duration timeout) async {
    final sw = Stopwatch()..start();
    while (true) {
      final sr = await st.read32(_sr);
      if (sr & _srBsy == 0) return sr;
      if (sw.elapsed > timeout) throw StLinkException('flash busy for too long');
      await Future<void>.delayed(const Duration(milliseconds: 2));
    }
  }

  static void checkWritable(int addr, int length) {
    if (addr < flashBase || addr + length > protectedStart) {
      throw StLinkException('refusing to write 0x${addr.toRadixString(16)}..0x${(addr + length).toRadixString(16)}: '
          'outside the firmware area (pages 30-31 hold the motor calibration)');
    }
  }

  Future<void> erasePage(int addr) async {
    checkWritable(addr, pageSize);
    if (addr % pageSize != 0) throw ArgumentError('page address not aligned');
    await st.write32(_cr, _crPer);
    await st.write32(_ar, addr);
    await st.write32(_cr, _crPer | _crStrt);
    final sr = await _waitBusy(const Duration(seconds: 1));
    await st.write32(_cr, 0);
    if (sr & (_srPgErr | _srWrPrtErr) != 0) {
      await st.write32(_sr, _srEop | _srPgErr | _srWrPrtErr);
      throw StLinkException('erasing page 0x${addr.toRadixString(16)} failed (write protected?)');
    }
    await st.write32(_sr, _srEop);
  }

  Future<void> _loadLoader() async {
    if (_loaderLoaded) return;
    await st.writeMem(_loaderAddr, Uint8List.fromList(_loader));
    _loaderLoaded = true;
  }

  /// Copies [data] (even length) into flash or option bytes at [dest] with the RAM loader. FPEC must be
  /// unlocked and CR set to PG or OPTPG by the caller. The core must be halted.
  Future<void> _runLoader(int dest, Uint8List data) async {
    if (data.length.isOdd) throw ArgumentError('odd length');
    if (data.isEmpty) return; // the loader counts down: 0 half-words would never end
    final padded = Uint8List((data.length + 3) & ~3)..fillRange(0, (data.length + 3) & ~3, 0xFF);
    padded.setRange(0, data.length, data);
    if (_bufferAddr + padded.length > sramEnd - 0x100) throw ArgumentError('block too big for RAM');
    await _loadLoader();
    await st.writeMem(_bufferAddr, padded);
    await st.writeReg(0, _bufferAddr);
    await st.writeReg(1, dest);
    await st.writeReg(2, data.length ~/ 2);
    await st.writeReg(3, _sr);
    await st.writeReg(13, sramEnd); // SP
    await st.writeReg(16, 0x01000000); // xPSR: Thumb
    await st.writeReg(15, _loaderAddr); // PC
    await st.write32(_dhcsr, _dbgKey | _cDebugEn); // run until bkpt
    await _waitHalted(const Duration(seconds: 3));
    final left = await st.readReg(2);
    final sr = await st.read32(_sr);
    if (left != 0 || sr & (_srPgErr | _srWrPrtErr) != 0) {
      await st.write32(_sr, _srEop | _srPgErr | _srWrPrtErr);
      throw StLinkException('programming 0x${dest.toRadixString(16)} failed '
          '(${sr & _srWrPrtErr != 0 ? 'write protected' : sr & _srPgErr != 0 ? 'not erased' : '$left half-words left'})');
    }
  }

  Future<void> programPage(int addr, Uint8List data) async {
    checkWritable(addr, data.length);
    await st.write32(_cr, _crPg);
    try {
      await _runLoader(addr, data);
    } finally {
      await st.write32(_cr, 0);
    }
  }

  /// Writes a firmware image. [image] starts at 0x08000000 and must end before [protectedStart].
  /// Erases every page from 0 to the last page of the image, skips writing pages that are all 0xFF,
  /// verifies everything. Core is reset+halted first and left halted.
  Future<void> writeImage(Uint8List image, {void Function(String step, double progress)? progress}) async {
    checkWritable(flashBase, image.length);
    progress?.call('Stopping the controller', 0);
    await resetHalt();
    await _unlockFlash();
    final pages = (image.length + pageSize - 1) ~/ pageSize;
    try {
      for (var p = 0; p < pages; p++) {
        progress?.call('Erasing', p / pages);
        await erasePage(flashBase + p * pageSize);
      }
      for (var p = 0; p < pages; p++) {
        progress?.call('Writing', p / pages);
        final chunk = Uint8List.sublistView(image, p * pageSize, (p + 1) * pageSize > image.length ? image.length : (p + 1) * pageSize);
        if (chunk.every((b) => b == 0xFF)) continue;
        await programPage(flashBase + p * pageSize, chunk);
      }
    } finally {
      await _lockFlash();
    }
    for (var p = 0; p < pages; p++) {
      progress?.call('Verifying', p / pages);
      final start = p * pageSize;
      final end = (start + pageSize > image.length) ? image.length : start + pageSize;
      final len = (end - start + 3) & ~3;
      final back = await st.readMem(flashBase + start, len);
      for (var i = 0; i < end - start; i++) {
        if (back[i] != image[start + i]) {
          throw StLinkException('verify failed at 0x${(flashBase + start + i).toRadixString(16)}');
        }
      }
    }
    progress?.call('Done', 1);
  }

  /// Rewrites only [data] at [addr] inside one flash page (read page, patch, erase, write, verify).
  /// Used to update the settings block without reinstalling the firmware.
  Future<void> patchPage(int addr, Uint8List data) async {
    final page = addr & ~(pageSize - 1);
    if (addr + data.length > page + pageSize) throw ArgumentError('patch crosses a page');
    checkWritable(page, pageSize);
    await resetHalt();
    final content = await st.readMem(page, pageSize);
    content.setRange(addr - page, addr - page + data.length, data);
    await _unlockFlash();
    try {
      await erasePage(page);
      await programPage(page, content);
    } finally {
      await _lockFlash();
    }
    final back = await st.readMem(page, pageSize);
    for (var i = 0; i < pageSize; i++) {
      if (back[i] != content[i]) throw StLinkException('verify failed at 0x${(page + i).toRadixString(16)}');
    }
  }

  // ------------------------------------------------------------------ read-out protection

  /// Removes the read-out protection (RDP = 0xA5). The chip then erases its whole flash by itself
  /// (original firmware gone, cannot be undone). The new state is active after a power cycle.
  Future<void> removeReadProtection({void Function(String)? log}) async {
    log?.call('stopping the core');
    await resetHalt();
    log?.call('unlocking the flash controller');
    await _unlockFlash();
    await st.write32(_optkeyr, _key1);
    await st.write32(_optkeyr, _key2);
    if ((await st.read32(_cr)) & _crOptWre == 0) throw StLinkException('could not unlock the option bytes');
    log?.call('erasing the option bytes (this also erases the whole flash)');
    await st.write32(_cr, _crOptEr | _crOptWre);
    await st.write32(_cr, _crOptEr | _crStrt | _crOptWre);
    var sr = await _waitBusy(const Duration(seconds: 10));
    if (sr & _srWrPrtErr != 0) throw StLinkException('option byte erase failed');
    await st.write32(_sr, _srEop | _srPgErr | _srWrPrtErr);
    log?.call('writing RDP = 0xA5 (unprotected), no write protection');
    // RDP, USER, DATA0, DATA1, WRP0..3: the complement bytes are written by the flash controller
    final ob = Uint8List.fromList([0xA5, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00]);
    await st.write32(_cr, _crOptPg | _crOptWre);
    try {
      await _runLoader(optionBytes, ob);
    } on StLinkException {
      if (!st.hasMem16) rethrow;
      log?.call('loader failed, trying direct 16-bit writes');
      for (var i = 0; i < ob.length; i += 2) {
        await st.writeMem16(optionBytes + i, Uint8List.sublistView(ob, i, i + 2));
        await _waitBusy(const Duration(seconds: 1));
      }
    } finally {
      await st.write32(_cr, _crLock);
    }
    sr = await st.read32(_sr);
    final wrpr = await st.read32(_wrpr);
    log?.call('done (SR 0x${sr.toRadixString(16)}, WRPR 0x${wrpr.toRadixString(16)}). Power cycle the controller now.');
  }
}

class ChipInfo {
  final int devId, revId, flashKb;
  final bool readProtected;
  ChipInfo({required this.devId, required this.revId, required this.flashKb, required this.readProtected});

  String get name => switch (devId) {
        0x412 => 'STM32F103 low density',
        0x410 => 'STM32F103 medium density',
        0x414 => 'STM32F103 high density',
        _ => 'unknown chip 0x${devId.toRadixString(16)}',
      };

  bool get supported => devId == 0x412 || devId == 0x410;
}
