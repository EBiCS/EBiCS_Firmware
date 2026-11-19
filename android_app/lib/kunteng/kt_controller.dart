import 'package:flutter/services.dart';

import '../stlink/swim.dart';
import '../usb/usb_bridge.dart';
import 'kt_settings.dart';

/// Result of a SWIM connection check.
class KtProbe {
  final bool stlinkFound, permission, targetFound, locked;
  final String error;
  KtProbe({
    this.stlinkFound = false,
    this.permission = false,
    this.targetFound = false,
    this.locked = false,
    this.error = '',
  });

  String describe() {
    if (!stlinkFound) return 'ST-Link NOT found. Plug it into the phone with a USB OTG adapter.';
    if (!permission) return 'ST-Link found, but the app may not use it yet. Allow USB access and try again.';
    if (!targetFound) {
      return 'ST-Link found, but the KT controller does not answer.\n'
          '  - is the controller switched on (battery)?\n'
          '  - 4 wires on the KT programming header: 3.3V, SWIM, GND, RST'
          '${error.isNotEmpty ? '\n  ($error)' : ''}';
    }
    return locked
        ? 'Controller found - chip is LOCKED (original KT firmware). Unlock it before installing.'
        : 'Controller found - chip is unlocked, ready to install.';
  }
}

/// Kunteng firmware actions over SWIM. Each action opens its own session and closes it again.
class KtController {
  static const _fwAsset = 'assets/firmware/kunteng.ihx';
  final AssetBundle bundle;
  final UsbTransport Function() _transport;
  final Future<void> Function(Duration)? sleep;

  KtController({AssetBundle? bundle, UsbTransport Function()? transport, this.sleep})
    : bundle = bundle ?? rootBundle,
      _transport = transport ?? UsbBridge.new;

  Future<bool> hasFirmware() async {
    try {
      await bundle.loadString(_fwAsset);
      return true;
    } catch (_) {
      return false;
    }
  }

  Future<T> _session<T>(Future<T> Function(Swim s) f) async {
    final t = _transport();
    if (t is UsbBridge) {
      final dev = await UsbBridge.find();
      if (dev == null) throw SwimException('ST-Link not found. Plug it in with a USB OTG adapter.');
      if (!dev.permission && !await UsbBridge.requestPermission()) {
        throw SwimException('no permission to use the ST-Link');
      }
      await t.open();
    }
    final s = Swim(t, sleep: sleep);
    try {
      await s.open();
      return await f(s);
    } finally {
      await s.close();
      if (t is UsbBridge) await t.close();
    }
  }

  Future<KtProbe> probe() async {
    final t = _transport();
    if (t is UsbBridge) {
      final dev = await UsbBridge.find();
      if (dev == null) return KtProbe();
      if (!dev.permission && !await UsbBridge.requestPermission()) return KtProbe(stlinkFound: true);
    }
    try {
      return await _session((s) async {
        final opt = await s.readOptionBytes(4);
        return KtProbe(stlinkFound: true, permission: true, targetFound: true, locked: opt[0] == 0xAA);
      });
    } catch (e) {
      return KtProbe(stlinkFound: true, permission: true, error: '$e');
    }
  }

  /// Removes read-out protection (erases the chip) and sets the option bytes the KT firmware needs.
  Future<void> unlock(void Function(String) log) => _session((s) async {
    log('writing option bytes (erases the original firmware) ...');
    await s.writeOptionBytes();
    final got = await s.readOptionBytes();
    log(
      _eq(got, Swim.ktOptionBytes) ? 'option bytes ok' : 'option bytes read back differently - power cycle and check',
    );
    await s.resetTarget();
  });

  /// Installs the prebuilt firmware and writes the settings block.
  Future<void> install(KtSettings settings, void Function(String step, double progress) report) async {
    final img = ihxToFlashImage(await bundle.loadString(_fwAsset));
    await _session((s) async {
      final opt = await s.readOptionBytes(4);
      if (opt[0] == 0xAA) throw SwimException('the chip is locked (original firmware) - unlock it first');
      if (opt[3] != Swim.ktOptionBytes[3]) {
        report('setting option bytes', 0);
        await s.writeOptionBytes();
      }
      await s.writeBlocks(Swim.flashStart, img, progress: (d, n) => report('writing firmware', d / n));
      report('verifying', 1);
      final back = await s.readMem(Swim.flashStart, img.length);
      if (!_eq(back, img)) throw SwimException('verify failed');
      await _writeSettings(s, settings);
      await s.resetTarget();
    });
  }

  /// Writes only the settings block (no reinstall needed) and restarts the controller.
  Future<void> writeSettings(KtSettings settings) => _session((s) async {
    await _writeSettings(s, settings);
    await s.resetTarget();
  });

  Future<void> _writeSettings(Swim s, KtSettings settings) async {
    // whole EEPROM block 1 (0x4080..0x40FF), keep what is behind the settings
    final cur = await s.readMem(KtSettings.blockAddress, Swim.block);
    final blk = Uint8List.fromList(cur)..setRange(0, KtSettings.blockLength, settings.block());
    await s.writeBlocks(KtSettings.blockAddress, blk);
    // forget the display values of the old firmware so the new defaults apply until the display sends its own
    final b0 = await s.readMem(Swim.eepromStart, Swim.block);
    if (b0[0] != 0) {
      final z = Uint8List.fromList(b0)..[0] = 0;
      await s.writeBlocks(Swim.eepromStart, z);
    }
  }

  static bool _eq(Uint8List a, Uint8List b) {
    if (a.length != b.length) return false;
    for (var i = 0; i < a.length; i++) {
      if (a[i] != b[i]) return false;
    }
    return true;
  }
}
