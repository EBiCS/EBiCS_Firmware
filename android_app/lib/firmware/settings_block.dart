import 'dart:typed_data';

import 'bike_settings.dart';

/// The 256 byte runtime settings block the firmware reads at boot (Inc/settings.h, Src/settings.c).
/// Same conversions as ebics_core.settings_block_values() in the desktop app; the unit test compares
/// against test_vectors.json made by build_variants.py from the Python code.
class SettingsBlock {
  static const address = 0x08007700;
  static const size = 256;
  static const magic = 0xEB1C5E77;
  static const version = 1;

  static const flagNames = [
    'throttle', 'legal', 'torque_sensor', 'torque_on_ad1', 'dirdet', 'adc_brake',
    'speed_external', 'speed_from_wire', 'temp_from_wire',
  ];
  static const fieldNames = [
    'ph_current_max', 'battery_current_max', 'regen_current', 'regen_current_max',
    'battery_voltage_min', 'battery_voltage_max', 'voltage_ramp_mv',
    'battery_level_1', 'battery_level_2', 'battery_level_3', 'battery_level_4', 'battery_level_5',
    'speed_limit', 'wheel_circumference', 'gear_ratio', 'pas_imp_per_turn', 'pulses_per_revolution',
    'pushassist_current', 'ts_coef', 'pas_timeout', 'ramp_end', 'throttle_offset', 'throttle_max',
    'motor_temp_threshold', 'motor_temp_max', 'frac_high', 'frac_low', 'reverse',
    'speed_temp_adc', 'speed_temp_threshold', 'ntc_pullup_ohm',
    'obs_inductance', 'obs_resistance', 'obs_flux_linkage', 'obs_gamma', 'obs_angle_offset',
  ];

  /// Python round(): halves to even.
  static int pyRound(double x) {
    final f = x.floorToDouble();
    final diff = x - f;
    if (diff > 0.5) return f.toInt() + 1;
    if (diff < 0.5) return f.toInt();
    return f.toInt().isEven ? f.toInt() : f.toInt() + 1;
  }

  /// Leading integer of a C expression like "38LL<<8" (ebics_core.cal_i_ma / _c_int).
  static int cInt(String s, int fallback) {
    final m = RegExp(r'^\s*(-?\d+)').firstMatch(s);
    return m == null ? fallback : int.parse(m.group(1)!);
  }

  static (Map<String, bool>, Map<String, int>) values(BikeSettings s) {
    final calI = cInt(s.s('cal_i'), 38);
    final (cutoff, vmax, bars) = s.batteryMv();
    int units(String k) => pyRound(s.d(k) * 1000 / calI);
    int ma(String k) => pyRound(s.d(k) * 1000);
    int atLeast1(int v) => v < 1 ? 1 : v;
    final st = s.i('speed_temp_input');
    final speedFromWire = st != 0 && s.b('speed_on_wire');
    final tempFromWire = st != 0 && s.b('temp_on_wire');
    final flags = {
      'throttle': s.b('throttle'),
      'legal': s.b('legal_mode'),
      'torque_sensor': s.b('torque_sensor'),
      'torque_on_ad1': s.b('torque_on_ad1'),
      'dirdet': s.b('dirdet'),
      'adc_brake': s.b('linear_regen'),
      'speed_external': s.b('speed_sensor_external') || speedFromWire,
      'speed_from_wire': speedFromWire,
      'temp_from_wire': tempFromWire,
    };
    final derate = s.values.containsKey('motor_temp_derate') ? s.i('motor_temp_derate') : 100;
    final tmax = s.values.containsKey('motor_temp_max') ? s.i('motor_temp_max') : 130;
    final ramp = (s.values.containsKey('voltage_ramp_v') ? s.d('voltage_ramp_v') : 1.0) * 1000;
    var angle = pyRound(s.d('obs_angle_deg') / 180.0 * 2147483648.0) & 0xFFFFFFFF;
    if (angle >= 0x80000000) angle -= 0x100000000;
    final v = <String, int>{
      'ph_current_max': units('phase_current_a'),
      'battery_current_max': ma('battery_current_a'),
      'regen_current': units('regen_current_a'),
      'regen_current_max': ma('regen_max_a'),
      'battery_voltage_min': cutoff,
      'battery_voltage_max': vmax,
      'voltage_ramp_mv': ramp.toInt() < 100 ? 100 : ramp.toInt(),
      'speed_limit': s.i('speed_limit'),
      'wheel_circumference': s.wheelCircumference,
      'gear_ratio': atLeast1(s.i('gear_ratio')),
      'pas_imp_per_turn': atLeast1(s.i('pas_magnets')),
      'pulses_per_revolution': atLeast1(s.i('pulses_per_rev')),
      'pushassist_current': units('walk_current_a'),
      'ts_coef': s.i('ts_coef'),
      'pas_timeout': s.i('pas_timeout'),
      'ramp_end': s.i('ramp_end'),
      'throttle_offset': s.i('throttle_offset'),
      'throttle_max': s.i('throttle_max'),
      'motor_temp_threshold': derate,
      'motor_temp_max': (derate + 5) > tmax ? derate + 5 : tmax,
      'frac_high': s.i('frac_high'),
      'frac_low': s.i('frac_low'),
      'reverse': s.b('reverse') ? -1 : 1,
      'speed_temp_adc': st != 0 ? st : 6,
      'speed_temp_threshold': s.values.containsKey('speed_temp_threshold') ? s.i('speed_temp_threshold') : 100,
      'ntc_pullup_ohm': tempFromWire ? atLeast1(s.values.containsKey('ntc_pullup_ohm') ? s.i('ntc_pullup_ohm') : 3000) : 0,
      'obs_inductance': cInt(s.s('inductance'), 0),
      'obs_resistance': cInt(s.s('resistance'), 0),
      'obs_flux_linkage': cInt(s.s('flux_linkage'), 0),
      'obs_gamma': cInt(s.s('gamma'), 0),
      'obs_angle_offset': angle,
    };
    for (var n = 0; n < 5; n++) {
      v['battery_level_${n + 1}'] = bars[n] * 10;
    }
    return (flags, v);
  }

  static Uint8List build(BikeSettings s) {
    final (flags, v) = values(s);
    final b = ByteData(size);
    var f = 0;
    for (var i = 0; i < flagNames.length; i++) {
      if (flags[flagNames[i]]!) f |= 1 << i;
    }
    b.setUint32(0, magic, Endian.little);
    b.setUint16(4, version, Endian.little);
    b.setUint16(6, size, Endian.little);
    b.setUint32(8, f, Endian.little);
    for (var i = 0; i < fieldNames.length; i++) {
      b.setInt32(12 + 4 * i, v[fieldNames[i]]!, Endian.little);
    }
    final bytes = b.buffer.asUint8List();
    b.setUint32(size - 4, crc32(Uint8List.sublistView(bytes, 0, size - 4)), Endian.little);
    return bytes;
  }

  /// Decodes a block read back from the controller (null if not a valid block).
  static Map<String, int>? decode(Uint8List bytes) {
    if (bytes.length < size) return null;
    final b = ByteData.sublistView(bytes);
    if (b.getUint32(0, Endian.little) != magic || b.getUint16(4, Endian.little) != version) return null;
    if (b.getUint32(size - 4, Endian.little) != crc32(Uint8List.sublistView(bytes, 0, size - 4))) return null;
    final out = <String, int>{'flags': b.getUint32(8, Endian.little)};
    for (var i = 0; i < fieldNames.length; i++) {
      out[fieldNames[i]] = b.getInt32(12 + 4 * i, Endian.little);
    }
    return out;
  }

  static final _table = List<int>.generate(256, (n) {
    var c = n;
    for (var k = 0; k < 8; k++) {
      c = (c & 1) != 0 ? 0xEDB88320 ^ (c >> 1) : c >> 1;
    }
    return c;
  });

  /// CRC-32 (IEEE 802.3, same as zlib.crc32 and Src/settings.c)
  static int crc32(Uint8List data) {
    var crc = 0xFFFFFFFF;
    for (final x in data) {
      crc = _table[(crc ^ x) & 0xFF] ^ (crc >> 8);
    }
    return (crc ^ 0xFFFFFFFF) & 0xFFFFFFFF;
  }
}
