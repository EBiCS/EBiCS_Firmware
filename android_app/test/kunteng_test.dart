import 'dart:convert';
import 'dart:io';

import 'package:ebics_setup/app_state.dart';
import 'package:ebics_setup/firmware/firmware_repo.dart';
import 'package:ebics_setup/kunteng/kt_controller.dart';
import 'package:ebics_setup/kunteng/kt_live.dart';
import 'package:ebics_setup/kunteng/kt_settings.dart';
import 'package:ebics_setup/main.dart';
import 'package:ebics_setup/stlink/swim.dart';
import 'package:ebics_setup/usb/usb_bridge.dart';
import 'package:flutter/services.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:material_ui/material_ui.dart';
import 'package:shared_preferences/shared_preferences.dart';

String hex(Uint8List b) => b.map((x) => x.toRadixString(16).padLeft(2, '0')).join();

/// Simulated ST-Link + STM8S105 over SWIM (same as setup_app/test_swim.py).
class FakeStm8 implements UsbTransport {
  final mem = Uint8List(0x10000);
  bool locked;
  int mode = Swim.modeDfu;
  int _expect = 0, _waddr = 0;
  Uint8List _pending = Uint8List(0), _readbuf = Uint8List(0);
  int blockWrites = 0;
  int resets = 0;

  FakeStm8({this.locked = false}) {
    if (locked) {
      mem[Swim.optStart] = 0xAA;
      mem.fillRange(Swim.flashStart, Swim.flashStart + 64, 0xA5);
    }
  }

  @override
  Future<void> bulkOut(Uint8List d, {int timeoutMs = 1000}) async {
    if (_expect > 0) {
      _store(d.sublist(0, d.length < _expect ? d.length : _expect));
      _expect -= d.length;
      return;
    }
    switch (d[0]) {
      case 0xF1:
        _pending = Uint8List.fromList([0x26, 0x07, 0x83, 0x04, 0x48, 0x37]);
      case 0xF5:
        _pending = Uint8List.fromList([mode, 0]);
      case 0xF3:
        mode = Swim.modeMass;
      case 0xF4:
        _swim(d);
      default:
        throw StateError('unexpected command ${d[0]}');
    }
  }

  void _swim(Uint8List c) {
    switch (c[1]) {
      case Swim.swimEnter:
        mode = Swim.modeSwim;
      case Swim.swimReadBufSize:
        _pending = Uint8List.fromList([0x00, 0x18]);
      case Swim.swimAssertReset:
        resets++;
      case Swim.swimReadCap:
        _pending = Uint8List(8);
      case Swim.swimReadStatus:
        _pending = Uint8List(4);
      case Swim.swimReadMem:
        final n = (c[2] << 8) | c[3];
        final a = (c[5] << 16) | (c[6] << 8) | c[7];
        _readbuf = Uint8List.fromList([for (var i = 0; i < n; i++) _read(a + i)]);
      case Swim.swimReadBuf:
        _pending = _readbuf;
      case Swim.swimWriteMem:
        final n = (c[2] << 8) | c[3];
        _waddr = (c[5] << 16) | (c[6] << 8) | c[7];
        final first = c.sublist(8, 8 + (n < 8 ? n : 8));
        _store(first);
        _expect = n - first.length;
        if (n == Swim.block) blockWrites++;
    }
  }

  int _read(int a) {
    if (locked && a >= Swim.flashStart) return 0x71;
    if (a == Swim.flashIapsr) return mem[a] | 0x04;
    if (a == Swim.swimCsr) return mem[a] | 0x02;
    return mem[a];
  }

  void _store(List<int> data) {
    for (final b in data) {
      final a = _waddr++;
      if (a >= Swim.optStart && a < Swim.optStart + 0x80) {
        expect(mem[Swim.flashCr2], 0x80, reason: 'option byte without OPT mode');
        if (a == Swim.optStart && b == 0 && locked) {
          mem.fillRange(Swim.flashStart, 0x10000, 0);
          mem.fillRange(Swim.eepromStart, Swim.eepromStart + Swim.eepromSize, 0);
          locked = false;
        }
      } else if (a >= Swim.flashStart) {
        expect(locked, isFalse);
        expect(mem[Swim.flashCr2] == 0x01 || mem[Swim.flashCr2] == 0x10, isTrue, reason: 'flash without block mode');
      }
      mem[a] = b;
    }
  }

  @override
  Future<Uint8List> bulkIn(int length, {int timeoutMs = 1000}) async {
    expect(_pending.length, greaterThanOrEqualTo(length));
    final r = _pending.sublist(0, length);
    _pending = _pending.sublist(length);
    return r;
  }
}

class FwBundle extends CachingAssetBundle {
  final String ihx;
  final String syms;
  FwBundle(this.ihx, {this.syms = '{}'});
  @override
  Future<ByteData> load(String key) async {
    if (key == 'assets/firmware/kunteng.ihx') return ByteData.sublistView(Uint8List.fromList(utf8.encode(ihx)));
    if (key == 'assets/firmware/kunteng.sym.json') return ByteData.sublistView(Uint8List.fromList(utf8.encode(syms)));
    throw FlutterError('no asset $key');
  }
}

String makeIhx(int start, Uint8List data) {
  final sb = StringBuffer();
  for (var off = 0; off < data.length; off += 16) {
    final chunk = data.sublist(off, off + 16 > data.length ? data.length : off + 16);
    final a = start + off;
    final rec = [chunk.length, a >> 8, a & 0xFF, 0, ...chunk];
    final sum = (-rec.fold<int>(0, (x, y) => x + y)) & 0xFF;
    sb.writeln(
      ':${rec.map((x) => x.toRadixString(16).padLeft(2, '0')).join().toUpperCase()}'
      '${sum.toRadixString(16).padLeft(2, '0').toUpperCase()}',
    );
  }
  sb.writeln(':00000001FF');
  return sb.toString();
}

Future<void> noSleep(Duration _) async {}

void main() {
  test('settings block matches the desktop app (test vectors from build_variants.py)', () {
    final f = File('assets/firmware/kunteng_test_vectors.json');
    if (!f.existsSync()) {
      markTestSkipped('kunteng_test_vectors.json not built');
      return;
    }
    for (final v in jsonDecode(f.readAsStringSync()) as List) {
      final m = v as Map<String, dynamic>;
      final s = KtSettings.decode(jsonEncode(m['settings']));
      expect(hex(s.block()), m['block_hex'], reason: m['name'] as String);
    }
  });

  test('settings block layout and checksum', () {
    final b = KtSettings.defaults().block();
    expect(b.length, 32);
    expect(b.sublist(0, 4), [0x4B, 0x54, 1, 32]);
    expect(b[4], 10);
    expect(b[11], 20); // assist 1 = 0.2
    final sum = b.sublist(0, 31).fold<int>(0, (a, x) => a + x);
    expect(b[31], ~sum & 0xFF);
  });

  test('unlock, install and settings over SWIM on the simulated chip', () async {
    final chip = FakeStm8(locked: true);
    final fw = Uint8List.fromList([for (var i = 0; i < 1000; i++) (i * 7 + 3) & 0xFF]);
    final c = KtController(bundle: FwBundle(makeIhx(0x8000, fw)), transport: () => chip, sleep: noSleep);
    final pr = await c.probe();
    expect(pr.targetFound, isTrue);
    expect(pr.locked, isTrue);
    await c.unlock((_) {});
    expect(chip.locked, isFalse);
    expect(chip.mem.sublist(Swim.optStart, Swim.optStart + 15), Swim.ktOptionBytes);
    final s = KtSettings.defaults()..['cells'] = 13;
    await c.install(s, (step, progress) {});
    expect(chip.mem.sublist(0x8000, 0x8000 + fw.length), fw);
    expect(chip.mem.sublist(KtSettings.blockAddress, KtSettings.blockAddress + 32), s.block());
    chip.blockWrites = 0;
    s['motor_current_a'] = 12;
    await c.writeSettings(s);
    expect(chip.blockWrites, 1);
    expect(chip.mem[KtSettings.blockAddress + 5], 12);
  });

  test('live data over SWIM without reset', () async {
    final chip = FakeStm8();
    chip.mem[0x51] = 140; // battery 38.1 V
    chip.mem[0x4D] = 0xFF;
    chip.mem[0x4E] = 0xE8; // -24 = -3 A
    chip.mem.setRange(0x80, 0x84, (ByteData(4)..setFloat32(0, 23.5)).buffer.asUint8List());
    chip.mem.setRange(0x2F, 0x37, [0, 1, 0xFE, 3, 0xFC, 2, 0, 0]);
    final syms = jsonEncode({
      'ui8_adc_battery_voltage_filtered': [0x51, 1],
      'i16_motor_current_filtered_10b': [0x4D, 2],
      'f_wheel_speed': [0x80, 4],
      'i8_hall_offset': [0x2F, 8],
    });
    final c = KtController(
      bundle: FwBundle('', syms: syms),
      transport: () => chip,
      sleep: noSleep,
    );
    final snap = await c.liveSnapshot();
    await c.stopLive();
    final v = {for (final x in ktLiveVars) x.name: x};
    expect(ktDescribe(v['ui8_adc_battery_voltage_filtered']!, snap['ui8_adc_battery_voltage_filtered']!), '38.08 V');
    expect(ktDescribe(v['i16_motor_current_filtered_10b']!, snap['i16_motor_current_filtered_10b']!), '-3.00 A');
    expect(ktDescribe(v['f_wheel_speed']!, snap['f_wheel_speed']!), '23.50 km/h');
    expect(ktDescribe(v['i8_hall_offset']!, snap['i8_hall_offset']!), '-4 +0 -2 +3 +1 +2');
    expect(chip.resets, 0, reason: 'live: no reset');
  });

  testWidgets('start screen leads to the Kunteng screens', (tester) async {
    tester.view.physicalSize = const Size(1080, 2400);
    tester.view.devicePixelRatio = 3.0;
    addTearDown(tester.view.reset);
    SharedPreferences.setMockInitialValues({});
    final st = AppState(await SharedPreferences.getInstance(), FirmwareRepo());
    await tester.pumpWidget(EbicsApp(state: st));
    await tester.pump();
    expect(find.text('Which controller do you want to set up?'), findsOneWidget);
    await tester.tap(find.text('Kunteng / KT (open source KT firmware)'));
    await tester.pump();
    await tester.ensureVisible(find.text('Continue'));
    await tester.pumpAndSettle();
    await tester.tap(find.text('Continue'));
    await tester.pumpAndSettle();
    expect(find.text('Kunteng (KT) setup'), findsOneWidget);
    await tester.scrollUntilVisible(find.text('Settings'), 100, scrollable: find.byType(Scrollable).first);
    await tester.pumpAndSettle();
    await tester.tap(find.text('Settings'));
    await tester.pumpAndSettle();
    expect(find.text('KT settings'), findsOneWidget);
    await tester.tap(find.text('Expert'));
    await tester.pumpAndSettle();
    expect(find.text('Learn hall sensor positions'), findsOneWidget);
    await tester.pageBack();
    await tester.pumpAndSettle();
    await tester.tap(find.byTooltip('Change controller'));
    await tester.pumpAndSettle();
    expect(find.text('Which controller do you want to set up?'), findsOneWidget);
  });
}
