import 'dart:convert';
import 'dart:typed_data';

import '../firmware/settings_block.dart' show SettingsBlock;

/// Kunteng (KT) settings with the same keys, defaults and block layout as the desktop app
/// (setup_app/kunteng_core.py). Profiles (my_kunteng.json) work in both apps.
enum KtKind { integer, decimal, boolean, choice }

class KtField {
  final String key, label, unit, help, section;
  final KtKind kind;
  final num def; // booleans: 0 / 1
  const KtField(this.key, this.label, this.kind, this.def, this.section, {this.unit = '', this.help = ''});
}

const ktChoices = <String, List<(String, int)>>{
  'controller': [('S06S / KT 6 MOSFET (up to 15 A)', 1), ('S12S / KT 12 MOSFET (up to 25 A)', 2)],
  'throttle_type': [('Throttle and/or PAS', 1), ('Torque sensor (on the throttle input)', 2)],
  'wheel': [
    ('16 inch', 0x00),
    ('20 inch', 0x08),
    ('24 inch', 0x10),
    ('26 inch', 0x14),
    ('700c', 0x18),
    ('28 inch', 0x1C),
    ('29 inch', 0x1E),
  ],
  'pas_direction': [('Right (normal)', 0), ('Left', 1)],
  'display': [
    ('Kunteng KT-LCD3 / LCD5 / LCD8', 0),
    ('KingMeter 901U / KM5S', 1),
    ('Bafang LCD (1200 baud)', 2),
    ('Bafang 850C / 860C (9600 baud)', 3),
  ],
};

const ktFields = <KtField>[
  KtField(
    'controller',
    'Controller',
    KtKind.choice,
    1,
    'basic',
    help: 'Which KT controller you have. The 12 MOSFET boards (S12S) allow more current.',
  ),
  KtField(
    'display',
    'Display',
    KtKind.choice,
    0,
    'basic',
    help:
        'Display protocol. Multi protocol displays: set the same in the display menu. KingMeter and Bafang '
        'assist levels are mapped to the 5 levels.',
  ),
  KtField(
    'cells',
    'Battery cells in series',
    KtKind.integer,
    10,
    'basic',
    unit: 'S',
    help: '7S = 24 V, 10S = 36 V, 13S = 48 V, 14S = 52 V. Sets the low voltage cutoff and the battery bars.',
  ),
  KtField(
    'motor_current_a',
    'Max motor current',
    KtKind.integer,
    15,
    'basic',
    unit: 'A',
    help: 'Battery side current limit. S06S boards: up to 15 A, S12S: up to 25 A.',
  ),
  KtField(
    'throttle_type',
    'Throttle / torque sensor',
    KtKind.choice,
    1,
    'basic',
    help: 'Throttle and PAS: power from the throttle or the pedal cadence. Torque sensor: on the throttle input.',
  ),
  KtField(
    'max_speed',
    'Speed limit before the display sends its own',
    KtKind.integer,
    25,
    'basic',
    unit: 'km/h',
    help: 'Used at power up until the display has sent its settings.',
  ),
  KtField(
    'wheel',
    'Wheel size before the display sends its own',
    KtKind.choice,
    0x14,
    'basic',
    help: 'Used at power up until the display has sent its settings.',
  ),
  KtField(
    'regen_current_a',
    'Regen current while braking',
    KtKind.integer,
    0,
    'advanced',
    unit: 'A',
    help: 'Electric braking with the brake lever. Only for direct drive motors. 0 = off.',
  ),
  KtField(
    'direct_drive',
    'Direct drive hub motor',
    KtKind.boolean,
    0,
    'advanced',
    help: 'The PWM stays on while braking so regen can work.',
  ),
  KtField('pas_magnets', 'PAS magnets', KtKind.integer, 8, 'advanced', help: 'Magnets on the PAS disc (8 or 12).'),
  KtField(
    'pas_direction',
    'PAS direction',
    KtKind.choice,
    0,
    'advanced',
    help: 'Switch this if the motor only assists when pedalling backwards.',
  ),
  KtField('pas_max_cadence', 'Cadence for full PAS power', KtKind.integer, 90, 'advanced', unit: 'rpm'),
  KtField('assist_1', 'Assist level 1', KtKind.decimal, 0.2, 'advanced', unit: 'x', help: '0 ... 1'),
  KtField('assist_2', 'Assist level 2', KtKind.decimal, 0.4, 'advanced', unit: 'x', help: '0 ... 1'),
  KtField('assist_3', 'Assist level 3', KtKind.decimal, 0.6, 'advanced', unit: 'x', help: '0 ... 1'),
  KtField('assist_4', 'Assist level 4', KtKind.decimal, 0.8, 'advanced', unit: 'x', help: '0 ... 1'),
  KtField('assist_5', 'Assist level 5', KtKind.decimal, 1.0, 'advanced', unit: 'x', help: '0 ... 1'),
  KtField(
    'ramp_s',
    'Power ramp (time to full power)',
    KtKind.decimal,
    0.4,
    'advanced',
    unit: 's',
    help: 'How fast the power comes up. 0.4 s is the old S06S behaviour, longer is gentler.',
  ),
  KtField(
    'motor_characteristic',
    'Motor constant for non-KT displays',
    KtKind.integer,
    202,
    'advanced',
    help: 'Like P1 of a KT display: magnets x gear ratio (202 for Q85/Q100). Used with KingMeter/Bafang displays.',
  ),
  KtField(
    'over_voltage_protection',
    'Over voltage protection',
    KtKind.boolean,
    0,
    'advanced',
    help: 'Stops the motor above 4.2 V per cell (error 05), restarts when the voltage drops.',
  ),
  KtField(
    'rotor_offset_angle',
    'Hall sensor angle offset',
    KtKind.integer,
    202,
    'expert',
    unit: '/256',
    help: '202 fits the Q85/Q100 geared motors. Wrong values: weak or no torque at start up.',
  ),
  KtField('foc_angle_adjust', 'FOC current sample angle', KtKind.integer, 137, 'expert', unit: '/256'),
  KtField('erps_interpolation', 'Speed for sine wave mode', KtKind.integer, 15, 'expert', unit: 'erps'),
  KtField(
    'foc_delay_us',
    'FOC current sample delay compensation',
    KtKind.integer,
    0,
    'expert',
    unit: 'us',
    help: 'Moves the FOC sample angle with the speed. 0 = off. Signed, tune it on the bike with Live data.',
  ),
  KtField(
    'phase_current_limit',
    'Limit phase current at low speed',
    KtKind.boolean,
    1,
    'expert',
    help: 'Protects the MOSFETs when starting under load.',
  ),
  KtField(
    'hall_learning',
    'Learn hall sensor positions',
    KtKind.boolean,
    1,
    'expert',
    help: 'Measures the real angle of each hall sensor at steady speed and corrects for it.',
  ),
];

class KtSettings {
  static const blockAddress = 0x4080, blockLength = 32, blockVersion = 1;
  final Map<String, num> values;
  KtSettings(this.values);

  factory KtSettings.defaults() => KtSettings({for (final f in ktFields) f.key: f.def});
  KtSettings copy() => KtSettings(Map.of(values));

  num operator [](String k) => values[k] ?? ktFields.firstWhere((f) => f.key == k).def;
  void operator []=(String k, num v) => values[k] = v;
  int i(String k) => this[k].round();
  double d(String k) => this[k].toDouble();
  bool b(String k) => this[k] != 0;

  /// JSON like my_kunteng.json of the desktop app (booleans as true/false).
  String encode() => jsonEncode({for (final f in ktFields) f.key: f.kind == KtKind.boolean ? b(f.key) : this[f.key]});

  factory KtSettings.decode(String text) {
    final j = jsonDecode(text) as Map<String, dynamic>;
    final s = KtSettings.defaults();
    for (final f in ktFields) {
      final v = j[f.key];
      if (v is bool) s[f.key] = v ? 1 : 0;
      if (v is num) s[f.key] = v;
    }
    return s;
  }

  List<String> problems() {
    final p = <String>[];
    if (i('cells') < 7 || i('cells') > 16) p.add('Battery cells must be 7 ... 16.');
    final imax = i('controller') == 1 ? 18 : 30;
    if (i('motor_current_a') < 1 || i('motor_current_a') > imax) p.add('Max motor current must be 1 ... $imax A.');
    if (i('regen_current_a') < 0 || i('regen_current_a') > 15) p.add('Regen current must be 0 ... 15 A.');
    if (i('pas_magnets') < 1 || i('pas_magnets') > 64) p.add('PAS magnets must be 1 ... 64.');
    if (i('pas_max_cadence') < 30 || i('pas_max_cadence') > 150) p.add('Cadence must be 30 ... 150 rpm.');
    for (var n = 1; n <= 5; n++) {
      if (d('assist_$n') < 0 || d('assist_$n') > 1) p.add('Assist level $n must be between 0 and 1.');
    }
    if (i('max_speed') < 5 || i('max_speed') > 70) p.add('Speed limit must be 5 ... 70 km/h.');
    for (final k in ['rotor_offset_angle', 'foc_angle_adjust']) {
      if (i(k) < 0 || i(k) > 255) p.add('${ktFields.firstWhere((f) => f.key == k).label} must be 0 ... 255.');
    }
    if (i('erps_interpolation') < 5 || i('erps_interpolation') > 100) p.add('Speed for sine wave mode: 5 ... 100.');
    if (d('ramp_s') < 0.1 || d('ramp_s') > 5) p.add('Power ramp must be 0.1 ... 5 s.');
    if (i('motor_characteristic') < 1 || i('motor_characteristic') > 255) p.add('Motor constant must be 1 ... 255.');
    if (i('foc_delay_us') < -500 || i('foc_delay_us') > 500) p.add('FOC delay compensation must be -500 ... 500 us.');
    return p;
  }

  /// 32 byte block the firmware reads from the data EEPROM (kunteng/settings.h).
  Uint8List block() {
    final out = Uint8List(blockLength);
    out.setAll(0, [0x4B, 0x54, blockVersion, blockLength]);
    out[4] = i('cells');
    out[5] = i('motor_current_a');
    out[6] = i('regen_current_a');
    out[7] = i('throttle_type');
    out[8] =
        (b2i('direct_drive') * 1) |
        (i('pas_direction') == 1 ? 2 : 0) |
        (b2i('over_voltage_protection') * 4) |
        (b('phase_current_limit') ? 0 : 8) |
        (b('hall_learning') ? 0 : 16);
    out[9] = i('pas_magnets');
    out[10] = i('pas_max_cadence');
    for (var n = 0; n < 5; n++) {
      out[11 + n] = SettingsBlock.pyRound(d('assist_${n + 1}') * 100);
    }
    out[16] = i('rotor_offset_angle') & 0xFF;
    out[17] = i('foc_angle_adjust') & 0xFF;
    out[18] = i('erps_interpolation');
    out[19] = i('max_speed');
    out[20] = i('wheel');
    out[21] = i('controller');
    out[22] = 25;
    out[23] = i('display');
    out[24] = SettingsBlock.pyRound(d('ramp_s') * 10);
    out[25] = (i('foc_delay_us') ~/ 4) & 0xFF; // int8, truncated toward zero like the desktop app
    out[26] = i('motor_characteristic');
    var sum = 0;
    for (var n = 0; n < 31; n++) {
      sum += out[n];
    }
    out[31] = ~sum & 0xFF;
    return out;
  }

  int b2i(String k) => b(k) ? 1 : 0;
}
