import 'dart:async';

import 'package:flutter/foundation.dart';

import '../firmware/bike_settings.dart';
import '../firmware/firmware_repo.dart';
import '../firmware/settings_block.dart';
import '../stlink/stlink.dart';
import '../stlink/stm32f1.dart';
import '../usb/usb_bridge.dart';

const setupMagic = 0xEB1C5002;

class ProbeResult {
  final bool stlinkFound;
  final bool permission;
  final bool targetFound;
  final String stlinkVersion;
  final double voltage;
  final ChipInfo? chip;
  final String error;
  ProbeResult({
    this.stlinkFound = false,
    this.permission = false,
    this.targetFound = false,
    this.stlinkVersion = '',
    this.voltage = 0,
    this.chip,
    this.error = '',
  });

  String describe() {
    if (!stlinkFound) {
      return 'ST-Link NOT found. Plug it into the phone with a USB OTG adapter.'
          '${error.isNotEmpty ? '  ($error)' : ''}';
    }
    if (!permission) return 'ST-Link found, but the app may not use it yet. Tap "Allow USB access".';
    if (!targetFound) {
      return 'ST-Link $stlinkVersion found, but the controller does not answer.\n'
          '  - is the controller switched on (battery)?\n'
          '  - is GND connected? (Lishui plug: BLACK = SWDIO, YELLOW = GND, GREEN = SWCLK)\n'
          '  - try swapping SWDIO and SWCLK'
          '${error.isNotEmpty ? '\n  ($error)' : ''}';
    }
    final c = chip!;
    return 'Connected: ${c.name}, ${c.flashKb} KB flash'
        '${voltage > 0 ? ', ${voltage.toStringAsFixed(2)} V' : ''}  (ST-Link $stlinkVersion)'
        '${c.supported ? '' : '\nWARNING: unexpected chip, EBiCS needs an STM32F103C6/C8.'}';
  }
}

/// One ST-Link session to the controller. All USB traffic goes through [_locked], so live data polling
/// and actions never interleave.
class Controller extends ChangeNotifier {
  final FirmwareRepo repo;
  Controller(this.repo);

  final _usb = UsbBridge();
  StLink? _st;
  Stm32F1? _chip;
  Future<void> _lock = Future.value();

  /// Variant currently on the controller (identified by reading the flash back), null = unknown.
  FirmwareVariant? installed;
  Map<String, (int, int)> _syms = {};

  bool get connected => _st != null;

  Future<T> _locked<T>(Future<T> Function() f) {
    final prev = _lock;
    final done = Completer<void>();
    _lock = done.future;
    return prev.then((_) => f()).whenComplete(done.complete);
  }

  Future<void> disconnect() => _locked(() async {
        await _closeQuiet();
      });

  Future<void> _closeQuiet() async {
    try {
      await _st?.exitDebug();
    } catch (_) {}
    try {
      await _usb.close();
    } catch (_) {}
    _st = null;
    _chip = null;
  }

  /// Opens the ST-Link (asks for USB permission if needed) and connects to the controller.
  Future<ProbeResult> probe({bool askPermission = true}) => _locked(() async {
        StLinkDeviceInfo? dev;
        try {
          dev = await UsbBridge.find();
        } catch (e) {
          return ProbeResult(error: '$e');
        }
        if (dev == null) {
          await _closeQuiet();
          return ProbeResult();
        }
        var perm = dev.permission;
        if (!perm && askPermission) perm = await UsbBridge.requestPermission();
        if (!perm) return ProbeResult(stlinkFound: true);
        String ver = '';
        double volts = 0;
        for (final divisor in [3, 15]) {
          // 950 kHz, then 240 kHz for long or bad wires
          try {
            if (_st == null) {
              await _usb.open();
              _st = StLink(_usb);
            }
            final st = _st!;
            await st.enterSwd(freqDivisor: divisor);
            ver = '${st.version}';
            volts = await st.targetVoltage();
            await st.readIdCode();
            _chip = Stm32F1(st);
            final info = await _chip!.info();
            return ProbeResult(
                stlinkFound: true, permission: true, targetFound: true, stlinkVersion: ver, voltage: volts, chip: info);
          } catch (e) {
            if (divisor == 15) {
              final err = '$e';
              await _closeQuiet();
              return ProbeResult(stlinkFound: true, permission: true, stlinkVersion: ver, voltage: volts, error: err);
            }
            await _closeQuiet();
          }
        }
        return ProbeResult(stlinkFound: true, permission: true);
      });

  Future<Stm32F1> _need() async {
    if (_chip == null) {
      throw StLinkException('not connected to the controller - check the ST-Link and power, then Retry');
    }
    return _chip!;
  }

  /// true = locked (original firmware), false = unlocked
  Future<bool> readProtected() => _locked(() async => (await _need()).readProtected());

  Future<void> unlock(void Function(String) log) => _locked(() async {
        await (await _need()).removeReadProtection(log: log);
        installed = null;
      });

  /// Installs the firmware variant for these settings, with the settings block. Pages 30-31 (calibration) stay.
  Future<FirmwareVariant> install(BikeSettings s, void Function(String step, double progress) progress) =>
      _locked(() async {
        final chip = await _need();
        final v = await repo.variantFor(s);
        if (v == null) throw StateError('no prebuilt firmware for display ${s.display}');
        if (await chip.readProtected()) {
          throw StLinkException('the chip is still locked - unlock it first and switch the controller off and on');
        }
        final img = await repo.image(v, s);
        await chip.writeImage(img, progress: progress);
        progress('Starting the firmware', 1);
        await chip.resetRun();
        installed = v;
        _syms = await repo.symbols(v);
        return v;
      });

  /// Writes only the settings block (page 29) and restarts the controller. The firmware must be one
  /// of the app's variants (checked by [identify]).
  Future<void> updateSettings(BikeSettings s) => _locked(() async {
        final chip = await _need();
        final v = installed;
        if (v == null || v.display != s.display || v.sensorless != s.sensorless) {
          throw StateError('display type or motor type changed: install the firmware instead');
        }
        await chip.patchPage(SettingsBlock.address, SettingsBlock.build(s));
        await chip.resetRun();
      });

  /// Finds out which of the app's firmware variants runs on the controller, by comparing the flash.
  Future<FirmwareVariant?> identify() => _locked(() async {
        final chip = await _need();
        final st = chip.st;
        final head = await st.readMem(Stm32F1.flashBase, 1024);
        FirmwareVariant? found;
        for (final v in await repo.variants()) {
          final bin = await repo.binary(v);
          if (!_same(head, bin, 1024)) continue;
          final len = (bin.length + 3) & ~3;
          final all = await st.readMem(Stm32F1.flashBase, len);
          if (_same(all, bin, bin.length)) {
            found = v;
            break;
          }
        }
        installed = found;
        _syms = found == null ? {} : await repo.symbols(found);
        return found;
      });

  static bool _same(Uint8List a, Uint8List b, int n) {
    if (a.length < n || b.length < n) return false;
    for (var i = 0; i < n; i++) {
      if (a[i] != b[i]) return false;
    }
    return true;
  }

  /// The settings block currently in the controller's flash (decoded), null if none/invalid.
  Future<Map<String, int>?> readSettingsBlock() => _locked(() async {
        final chip = await _need();
        return SettingsBlock.decode(await chip.st.readMem(SettingsBlock.address, SettingsBlock.size));
      });

  // ------------------------------------------------------------------ live data / setup mailbox

  static const liveVars = <String, String>{
    'ui32_setup_magic': 'u32', 'ui8_setup_state': 'u8', 'ui8_setup_progress': 'u8', 'ui8_setup_halls_found': 'u8',
    'ui8_hall_calibrated': 'u8', 'ui32_hall_events': 'u32', 'ui32_pas_events': 'u32', 'brake_flag': 'u8',
    'ui16_throttle': 'u16', 'ui16_throttle_offset': 'u16', 'uint16_full_rotation_counter': 'u16',
    'i16_hall_order': 'i16', 'ui32_KV': 'i16', 'ui8_watchdog_reset': 'u8', 'ui16_motor_temp_adc': 'u16',
    'ui8_sensorless_running': 'u8', 'ui8_settings_source': 'u8', 'ui8_motor_temp_sensor_ok': 'u8',
  };
  static const msFields = <String, (String, int)>{
    'Voltage': ('i32', 0), 'Speed': ('u32', 4), 'i_q': ('i32', 12), 'i_q_setpoint': ('i32', 16),
    'u_abs': ('i32', 44), 'Battery_Current': ('i32', 48), 'assist_level': ('u8', 54),
    'Temperature': ('i16', 56), 'int_Temperature': ('i16', 58),
  };
  static const halls = ['Hall_45', 'Hall_51', 'Hall_13', 'Hall_32', 'Hall_26', 'Hall_64'];
  static const _sizes = {'u8': 1, 'u16': 2, 'i16': 2, 'u32': 4, 'i32': 4};

  bool get hasSymbols => _syms.isNotEmpty;

  /// Reads all live values in one go (RAM is small: one block read of the used range).
  Future<Map<String, int>?> snapshot({bool withHalls = false}) => _locked(() async {
        final chip = await _need();
        if (_syms.isEmpty) return null;
        final plan = <(String, int, String)>[];
        for (final e in liveVars.entries) {
          final s = _syms[e.key];
          if (s != null) plan.add((e.key, s.$1, e.value));
        }
        final ms = _syms['MS'];
        if (ms != null) {
          for (final e in msFields.entries) {
            plan.add(('MS.${e.key}', ms.$1 + e.value.$2, e.value.$1));
          }
        }
        if (withHalls) {
          for (final h in halls) {
            final s = _syms[h];
            if (s != null) plan.add((h, s.$1, 'u32'));
          }
        }
        final adc = _syms['adcData'];
        if (adc != null) {
          for (var n = 0; n < 9; n++) {
            plan.add(('adc$n', adc.$1 + 2 * n, 'u16'));
          }
        }
        if (plan.isEmpty) return null;
        var lo = plan.map((p) => p.$2).reduce((a, b) => a < b ? a : b) & ~3;
        var hi = plan.map((p) => p.$2 + _sizes[p.$3]!).reduce((a, b) => a > b ? a : b);
        hi = (hi + 3) & ~3;
        final mem = await chip.st.readMem(lo, hi - lo);
        final bd = ByteData.sublistView(mem);
        final out = <String, int>{};
        for (final (name, addr, typ) in plan) {
          final o = addr - lo;
          out[name] = switch (typ) {
            'u8' => bd.getUint8(o),
            'u16' => bd.getUint16(o, Endian.little),
            'i16' => bd.getInt16(o, Endian.little),
            'u32' => bd.getUint32(o, Endian.little),
            _ => bd.getInt32(o, Endian.little),
          };
        }
        return out;
      });

  /// Asks the firmware to start the hall + KV calibration (ui8_setup_cmd = 1).
  Future<void> startCalibration() => _locked(() async {
        final chip = await _need();
        final s = _syms['ui8_setup_cmd'];
        if (s == null) throw StateError('firmware information missing');
        await chip.st.writeMem8(s.$1, Uint8List.fromList([1]));
      });

  Future<void> restart() => _locked(() async => (await _need()).resetRun());
}

// ------------------------------------------------------------------ helpers like ebics_core

double speedKmh(Map<String, int>? snap, BikeSettings s) {
  if (snap == null || (snap['uint16_full_rotation_counter'] ?? 8000) >= 8000) return 0;
  final tics = snap['MS.Speed'] ?? 0;
  if (tics <= 0) return 0;
  final g = s.i('gear_ratio') < 1 ? 1 : s.i('gear_ratio');
  final tWheel = tics * 6 * g / 500000.0;
  return tWheel > 0 ? s.wheelCircumference / 1000.0 / tWheel * 3.6 : 0;
}

List<double>? hallAnglesDeg(Map<String, int>? snap) {
  if (snap == null) return null;
  final out = <double>[];
  for (final h in Controller.halls) {
    final v = snap[h];
    if (v == null) return null;
    out.add(v * 360.0 / 4294967296.0);
  }
  return out;
}

bool checkHallAngles(List<double>? angles) {
  if (angles == null) return false;
  final a = [...angles]..sort();
  for (var i = 0; i < 6; i++) {
    final g = (a[(i + 1) % 6] - a[i]) % 360;
    if (g < 35 || g > 85) return false;
  }
  return true;
}
