import 'dart:convert';

import 'package:flutter/services.dart';

import '../stlink/stm32f1.dart';
import 'bike_settings.dart';
import 'settings_block.dart';

/// Prebuilt firmware variants shipped in assets/firmware (setup_app/build_variants.py).
class FirmwareVariant {
  final String name, display, displayLabel, file, symbolsFile, sha256;
  final bool sensorless;
  final int size;
  FirmwareVariant.fromJson(Map<String, dynamic> j)
      : name = j['name'] as String,
        display = j['display'] as String,
        displayLabel = j['display_label'] as String? ?? j['display'] as String,
        sensorless = j['sensorless'] == true,
        file = j['file'] as String,
        symbolsFile = j['symbols'] as String,
        size = j['size'] as int,
        sha256 = j['sha256'] as String? ?? '';
}

class FirmwareRepo {
  static const _dir = 'assets/firmware';
  final AssetBundle bundle;
  FirmwareRepo([AssetBundle? b]) : bundle = b ?? rootBundle;

  Map<String, dynamic>? _manifest;
  final _bins = <String, Uint8List>{};
  final _syms = <String, Map<String, (int, int)>>{};

  Future<Map<String, dynamic>> manifest() async =>
      _manifest ??= jsonDecode(await bundle.loadString('$_dir/manifest.json')) as Map<String, dynamic>;

  Future<String> version() async => (await manifest())['firmware_version'] as String? ?? '?';

  Future<List<FirmwareVariant>> variants() async => [
        for (final v in (await manifest())['variants'] as List) FirmwareVariant.fromJson(v as Map<String, dynamic>)
      ];

  Future<FirmwareVariant?> variantFor(BikeSettings s) async {
    for (final v in await variants()) {
      if (v.display == s.display && v.sensorless == s.sensorless) return v;
    }
    return null;
  }

  Future<FirmwareVariant?> byName(String? name) async {
    for (final v in await variants()) {
      if (v.name == name) return v;
    }
    return null;
  }

  Future<Uint8List> binary(FirmwareVariant v) async =>
      _bins[v.name] ??= (await bundle.load('$_dir/${v.file}')).buffer.asUint8List();

  /// name -> (address, size)
  Future<Map<String, (int, int)>> symbols(FirmwareVariant v) async {
    final cached = _syms[v.name];
    if (cached != null) return cached;
    final j = jsonDecode(await bundle.loadString('$_dir/${v.symbolsFile}')) as Map<String, dynamic>;
    return _syms[v.name] = {
      for (final e in j.entries) e.key: ((e.value as List)[0] as int, (e.value as List)[1] as int),
    };
  }

  /// Full flash image 0x08000000..0x080077FF: firmware, 0xFF padding, settings block at the end.
  Future<Uint8List> image(FirmwareVariant v, BikeSettings s) async {
    final bin = await binary(v);
    const len = SettingsBlock.address + SettingsBlock.size - Stm32F1.flashBase;
    if (bin.length > SettingsBlock.address - Stm32F1.flashBase) {
      throw StateError('firmware ${v.name} is too big');
    }
    final img = Uint8List(len)..fillRange(0, len, 0xFF);
    img.setRange(0, bin.length, bin);
    img.setRange(SettingsBlock.address - Stm32F1.flashBase, len, SettingsBlock.build(s));
    return img;
  }
}
