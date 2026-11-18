import 'dart:convert';
import 'dart:io';
import 'dart:typed_data';

import 'package:ebics_setup/firmware/bike_settings.dart';
import 'package:ebics_setup/firmware/settings_block.dart';
import 'package:flutter_test/flutter_test.dart';

String hex(Uint8List b) => b.map((x) => x.toRadixString(16).padLeft(2, '0')).join();

void main() {
  test('CRC-32 like zlib', () {
    expect(SettingsBlock.crc32(Uint8List.fromList(utf8.encode('123456789'))), 0xCBF43926);
    expect(SettingsBlock.crc32(Uint8List(0)), 0);
  });

  test('Python round() halves to even', () {
    expect(SettingsBlock.pyRound(0.5), 0);
    expect(SettingsBlock.pyRound(1.5), 2);
    expect(SettingsBlock.pyRound(2.5), 2);
    expect(SettingsBlock.pyRound(-0.5), 0);
    expect(SettingsBlock.pyRound(-1.5), -2);
    expect(SettingsBlock.pyRound(394.7), 395);
  });

  test('block layout', () {
    final b = SettingsBlock.build(BikeSettings.defaults());
    expect(b.length, 256);
    final d = ByteData.sublistView(b);
    expect(d.getUint32(0, Endian.little), 0xEB1C5E77);
    expect(d.getUint16(4, Endian.little), 1);
    expect(d.getUint16(6, Endian.little), 256);
    final v = SettingsBlock.decode(b)!;
    expect(v['speed_limit'], 25);
    expect(v['wheel_circumference'], 2240);
    expect(v['battery_voltage_min'], 31000);
    expect(v['battery_voltage_max'], 42000);
    expect(v['reverse'], 1);
    expect(v['flags']! & 1, 1, reason: 'throttle on by default');
    b[20] ^= 1;
    expect(SettingsBlock.decode(b), isNull, reason: 'CRC must catch changes');
  });

  // Vectors from setup_app/build_variants.py (Python reference, ebics_core.settings_block). CI builds the
  // firmware assets before running the tests.
  final vectors = File('assets/firmware/test_vectors.json');
  test('same blocks as the desktop app (test_vectors.json)', () {
    final list = jsonDecode(vectors.readAsStringSync()) as List;
    expect(list, isNotEmpty);
    for (final v in list.cast<Map<String, dynamic>>()) {
      final s = BikeSettings.fromJson(v['settings'] as Map<String, dynamic>);
      expect(hex(SettingsBlock.build(s)), v['block_hex'], reason: v['name'] as String);
    }
  }, skip: vectors.existsSync() ? false : 'run setup_app/build_variants.py first');

  test('problems() catches bad battery voltages', () {
    final s = BikeSettings.defaults()..['battery'] = 0;
    expect(s.problems(), isNotEmpty);
    s['full_voltage_v'] = 54.6;
    s['voltage_min_v'] = 39.0;
    expect(s.problems(), isEmpty);
  });
}
