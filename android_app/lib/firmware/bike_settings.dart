import 'dart:convert';

/// Bike settings with the same keys, defaults and conversions as the desktop app
/// (setup_app/ebics_core.py, FIELDS). Profiles (my_bike.json) work in both apps.
enum FieldKind { integer, decimal, boolean, display, battery, wheel, speedTemp, text }

class Field {
  final String key, label, unit, help, section;
  final FieldKind kind;
  final Object def;

  /// false = compile-time option: the phone uses the prebuilt default (change it with the desktop app)
  final bool runtime;
  const Field(this.key, this.label, this.kind, this.def, this.section,
      {this.unit = '', this.help = '', this.runtime = true});
}

class DisplayType {
  final String key, label, description;
  const DisplayType(this.key, this.label, this.description);
}

const displays = [
  DisplayType('KM5S', 'KingMeter 5S / KM5S / 901U (recommended)',
      "Most complete protocol in EBiCS. Use this if your display can be set to '5S' or 'KM5S'."),
  DisplayType('JLCD', 'KingMeter J-LCD / 618U', 'Older KingMeter protocol.'),
  DisplayType('BAFANG_LCD', 'Bafang LCD (1200 baud)', "Bafang displays incl. 'Blaupunkt'."),
  DisplayType('BAFANG_TFT', 'Bafang 850C / 860C (9600 baud)', 'Bafang color displays.'),
  DisplayType('KUNTENG', 'Kunteng KT-LCD3 / LCD5 / LCD8', 'KT protocol.'),
  DisplayType('NO2', 'UART No.2 (S866, EN06 ...)', "Chinese 'protocol 2'. Assist level, light and walk assist only."),
  DisplayType('TONGSHENG', 'Tongsheng VLCD5 / VLCD6 / XH18 (new)',
      'Tongsheng factory protocol. New in this version, not yet tested on many displays.'),
  DisplayType('VESC', 'VESC UART (115200 baud) (new)',
      'Live data for VESC displays and apps (via a UART-BLE bridge). Not a full VESC: no VESC Tool configuration.'),
  DisplayType('EBICS', 'EBiCS app / BLE module', 'For the EBiCS Flutter app with the EBiCS BLE module.'),
  DisplayType('DEBUG', 'No display (debug text on UART)', 'Prints debug values at 56000 baud.'),
];

const wheels = <(String, int?)>[
  ('16"', 1270), ('20"', 1590), ('24"', 1900), ('26"', 2070), ('27.5"', 2200),
  ('28" / 700C', 2240), ('29"', 2290), ('custom', null),
];

const batteries = <(String, int)>[
  ('24 V (7S)', 7), ('36 V (10S)', 10), ('48 V (13S)', 13), ('52 V (14S)', 14), ('custom (set voltages below)', 0),
];

const speedTempInputs = <(String, int)>[('not used', 0), ('AD1 (PB1)', 6), ('AD2 (PB0)', 5)];

const fields = <Field>[
  // --- basic: asked in the setup wizard
  Field('display', 'Display protocol', FieldKind.display, 'KM5S', 'basic',
      help: 'Which protocol your display speaks. Multi-protocol displays: choose the same in the display menu.'),
  Field('battery', 'Battery', FieldKind.battery, 10, 'basic',
      help: 'Nominal battery voltage. Sets the low-voltage cutoff and the battery bars.'),
  Field('full_voltage_v', 'Full charge voltage (0 = automatic)', FieldKind.decimal, 0.0, 'basic',
      unit: 'V',
      help: 'Battery voltage when fully charged. Regen is ramped down to zero at this voltage. '
          '0 = cells x 4.2 V. Must be set for a custom battery.'),
  Field('voltage_min_v', 'Cutoff voltage (0 = automatic)', FieldKind.decimal, 0.0, 'basic',
      unit: 'V',
      help: 'Below this voltage the motor stops (soft ramp just above it). 0 = cells x 3.1 V. '
          'Must be set for a custom battery.'),
  Field('wheel', 'Wheel size', FieldKind.wheel, '28" / 700C', 'basic', help: 'Used for speed and the speed limit.'),
  Field('wheel_mm', 'Wheel circumference', FieldKind.integer, 2240, 'basic',
      unit: 'mm', help: "Only used when wheel size is 'custom'. Roll the bike one turn and measure."),
  Field('speed_limit', 'Speed limit', FieldKind.integer, 25, 'basic', unit: 'km/h', help: 'Assist stops above this speed.'),
  Field('phase_current_a', 'Motor current max', FieldKind.decimal, 15.0, 'basic',
      unit: 'A', help: 'Maximum motor phase current = motor power. Start low (10-15 A) and increase slowly.'),
  Field('battery_current_a', 'Battery current max', FieldKind.decimal, 15.0, 'basic',
      unit: 'A', help: 'Maximum current taken from the battery. Must not exceed what your battery/BMS allows.'),
  Field('pas_magnets', 'Pedal sensor magnets', FieldKind.integer, 12, 'basic',
      help: 'Number of magnets/pulses per crank turn of the pedal (PAS) sensor. Usually 12 or 32.'),
  Field('torque_sensor', 'Torque sensor', FieldKind.boolean, false, 'basic',
      help: 'On if you have a torque sensor instead of a plain cadence PAS sensor.'),
  Field('throttle', 'Throttle', FieldKind.boolean, true, 'basic', help: 'On if a throttle is connected.'),
  Field('legal_mode', 'Legal mode (EU)', FieldKind.boolean, false, 'basic',
      help: "Ramps the motor down at the speed limit and limits to 6 km/h when you don't pedal."),
  Field('walk_current_a', 'Walk assist current', FieldKind.decimal, 6.0, 'basic',
      unit: 'A', help: 'Motor current for the walk / push assist button.'),
  Field('sensorless', 'Sensorless motor (no hall sensors) - experimental', FieldKind.boolean, false, 'basic',
      help: 'Run the motor without hall sensors. Needs the motor parameters in Expert. '
          'Hall calibration and gearing measurement are not possible. Test carefully.'),
  Field('reverse', 'Reverse motor direction', FieldKind.boolean, false, 'basic',
      help: 'On if the wheel turns backwards. The setup wizard asks you and sets this.'),
  Field('gear_ratio', 'Pole pairs x gear ratio', FieldKind.integer, 60, 'basic',
      help: 'Motor electrical turns per wheel turn. The setup wizard measures it when you turn the wheel by hand.'),
  // --- advanced
  Field('speed_sensor_external', 'External speed sensor', FieldKind.boolean, false, 'advanced',
      help: 'Use a spoke magnet speed sensor instead of the motor hall sensors for speed.'),
  Field('pulses_per_rev', 'Speed sensor magnets', FieldKind.integer, 1, 'advanced',
      help: 'Magnets on the wheel for the external speed sensor.'),
  Field('speed_temp_input', 'Speed/temperature wire on', FieldKind.speedTemp, 0, 'advanced',
      help: 'Motors with one wire for speed sensor AND temperature (NTC between the signal and GND): connect that '
          'wire to an analog input (AD1 or AD2) with a pull-up resistor to 3.3 V.'),
  Field('speed_on_wire', '  use it for speed', FieldKind.boolean, true, 'advanced',
      help: "Speed pulses from this wire (the motor's speed sensor). Sets the speed source to external."),
  Field('temp_on_wire', '  use it for motor temperature', FieldKind.boolean, true, 'advanced',
      help: "Motor temperature from the NTC on this wire. The motor current is reduced above 'derating start'."),
  Field('ntc_pullup_ohm', '  NTC pull-up resistor', FieldKind.integer, 3000, 'advanced',
      unit: 'ohm', help: 'Pull-up resistor from the input to 3.3 V. NTC: 10k B3950.'),
  Field('speed_temp_threshold', '  speed pulse threshold', FieldKind.integer, 100, 'advanced',
      unit: 'ADC', help: 'Readings below = speed pulse, above = temperature. See the raw value in Live data.'),
  Field('motor_temp_derate', 'Motor temp: derating start', FieldKind.integer, 100, 'advanced',
      unit: '°C', help: 'Motor current is reduced from here ...'),
  Field('motor_temp_max', 'Motor temp: maximum', FieldKind.integer, 130, 'advanced',
      unit: '°C', help: '... down to zero at this temperature.'),
  Field('torque_on_ad1', 'Torque signal on AD1', FieldKind.boolean, false, 'advanced',
      help: 'Torque sensor wired to AD1 instead of the throttle input.'),
  Field('ts_coef', 'Torque sensor gain', FieldKind.integer, 60000, 'advanced', help: 'Higher = more assist per torque.'),
  Field('pas_timeout', 'PAS stop timeout', FieldKind.integer, 8000, 'advanced',
      unit: 'ticks', help: 'Motor stops after this time without PAS pulse. 8000 = 1 s.'),
  Field('ramp_end', 'PAS full assist at', FieldKind.integer, 1600, 'advanced',
      unit: 'ticks', help: 'PAS pulse interval for full assist (8 ticks = 1 ms).'),
  Field('dirdet', 'Stop on backpedalling', FieldKind.boolean, false, 'advanced',
      help: 'PAS direction detection (needs an asymmetric PAS sensor).'),
  Field('frac_high', 'Direction window max', FieldKind.integer, 30, 'advanced'),
  Field('frac_low', 'Direction window min', FieldKind.integer, 15, 'advanced'),
  Field('throttle_offset', 'Throttle min (ADC)', FieldKind.integer, 750, 'advanced',
      help: 'Default only: measured automatically at power-up.'),
  Field('throttle_max', 'Throttle max (ADC)', FieldKind.integer, 2600, 'advanced',
      help: 'ADC value at full throttle (see Live data).'),
  Field('speed_throttle', 'Throttle sets target speed', FieldKind.boolean, false, 'advanced', runtime: false),
  Field('regen_current_a', 'Brake regen current', FieldKind.decimal, 0.0, 'advanced',
      unit: 'A', help: 'Phase current for regenerative braking (0 = off).'),
  Field('regen_max_a', 'Regen max into battery', FieldKind.decimal, 0.0, 'advanced', unit: 'A'),
  Field('linear_regen', 'Proportional regen on AD2', FieldKind.boolean, false, 'advanced', help: 'Analog brake sensor on AD2.'),
  Field('voltage_ramp_v', 'Soft voltage ramp width', FieldKind.decimal, 1.0, 'advanced',
      unit: 'V', help: 'Current is reduced smoothly within this voltage above the cutoff and below full charge.'),
  Field('individual_profile', 'Individual assist profile', FieldKind.boolean, false, 'advanced', runtime: false),
  // --- expert
  Field('pll', 'Rotor position PLL', FieldKind.boolean, false, 'expert', runtime: false),
  Field('p_pll', 'PLL P (shift)', FieldKind.integer, 10, 'expert', runtime: false),
  Field('i_pll', 'PLL I (shift)', FieldKind.integer, 10, 'expert', runtime: false),
  Field('spdshft', 'PLL speed shift', FieldKind.integer, 0, 'expert', runtime: false),
  Field('p_iq', 'Iq P gain', FieldKind.text, '500', 'expert', runtime: false),
  Field('i_iq', 'Iq I gain', FieldKind.text, '20', 'expert', runtime: false),
  Field('p_id', 'Id P gain', FieldKind.text, '500', 'expert', runtime: false),
  Field('i_id', 'Id I gain', FieldKind.text, '20', 'expert', runtime: false),
  Field('p_speed', 'Speed P gain', FieldKind.integer, 100, 'expert', runtime: false),
  Field('i_speed', 'Speed I gain', FieldKind.integer, 10, 'expert', runtime: false),
  Field('speedfilter', 'Speed filter (shift)', FieldKind.integer, 1, 'expert', runtime: false),
  Field('sixstep', '6-step threshold', FieldKind.integer, 3500, 'expert', runtime: false),
  Field('trigger_offset', 'ADC trigger offset', FieldKind.integer, 50, 'expert', runtime: false),
  Field('trigger_default', 'ADC trigger default', FieldKind.integer, 2020, 'expert', runtime: false),
  Field('timer_period', 'PWM timer period', FieldKind.integer, 2028, 'expert', runtime: false),
  Field('disable_dyn_adc', 'Fixed ADC trigger', FieldKind.boolean, false, 'expert', runtime: false),
  Field('fast_loop_log', 'Fast loop logging', FieldKind.boolean, false, 'expert', runtime: false),
  Field('cal_bat_v', 'Battery voltage cal.', FieldKind.integer, 256, 'expert', runtime: false),
  Field('cal_v', 'Voltage cal. (mV/count)', FieldKind.integer, 25, 'expert', runtime: false),
  Field('cal_i', 'Current cal. (mA/count <<8)', FieldKind.text, '38LL<<8', 'expert', runtime: false),
  Field('inductance', 'Sensorless: inductance', FieldKind.text, '11', 'expert',
      help: 'Observer inductance = phase inductance [uH] x 0.0437. Phase value = half of the value between two motor wires. 250 uH -> 11.'),
  Field('resistance', 'Sensorless: resistance', FieldKind.text, '51', 'expert',
      help: 'Observer resistance = phase resistance [mOhm] x 0.341. Phase value = half of the value between two motor wires. 150 mOhm -> 51.'),
  Field('flux_linkage', 'Sensorless: flux linkage', FieldKind.text, '2560', 'expert',
      help: 'Observer flux = flux linkage [mWb] x 256 (flux = battery V / (1.732 x electrical no-load rad/s)). 10 mWb -> 2560.'),
  Field('gamma', 'Sensorless: observer gain (shift)', FieldKind.text, '13', 'expert',
      help: 'Observer correction gain as a shift, 12..14. Only used in sensorless mode.'),
  Field('obs_angle_deg', 'Sensorless: angle offset', FieldKind.decimal, 90.0, 'expert',
      unit: 'deg', help: '+90 matches the open loop start direction. Fine tune in small steps for smooth running.'),
  Field('spec_angle', 'Hall angle offset (unused)', FieldKind.text, '-715827882', 'expert', runtime: false),
  Field('autodetect_at_boot', 'Autodetect at every power-up', FieldKind.boolean, false, 'expert', runtime: false),
];

final fieldByKey = {for (final f in fields) f.key: f};

class BikeSettings {
  final Map<String, Object?> values;
  BikeSettings(this.values);

  factory BikeSettings.defaults() => BikeSettings({for (final f in fields) f.key: f.def});

  /// Like ebics_core.load_settings: defaults, overwritten by the profile (unknown keys are kept).
  factory BikeSettings.fromJson(Map<String, dynamic> json) {
    final s = BikeSettings.defaults();
    s.values.addAll(json);
    return s;
  }

  factory BikeSettings.decode(String text) => BikeSettings.fromJson(jsonDecode(text) as Map<String, dynamic>);

  String encode() => const JsonEncoder.withIndent('  ').convert(values);

  BikeSettings copy() => BikeSettings(Map.of(values));

  Object? operator [](String k) => values[k];
  void operator []=(String k, Object? v) => values[k] = v;

  bool b(String k) {
    final v = values[k];
    if (v is bool) return v;
    if (v is num) return v != 0;
    if (v is String) return v.toLowerCase() == 'true' || v == '1';
    return false;
  }

  double d(String k) {
    final v = values[k];
    if (v is num) return v.toDouble();
    return double.tryParse('$v'.replaceAll(',', '.')) ?? 0.0;
  }

  /// Python int(): truncates floats, parses "12" and "12.0".
  int i(String k) {
    final v = values[k];
    if (v is int) return v;
    if (v is num) return v.toInt();
    final s = '$v'.trim();
    return int.tryParse(s) ?? double.tryParse(s)?.toInt() ?? 0;
  }

  String s(String k) => '${values[k] ?? ''}';

  String get display => s('display');
  bool get sensorless => b('sensorless');
  DisplayType get displayType => displays.firstWhere((d) => d.key == display, orElse: () => displays.first);

  int get wheelCircumference {
    for (final (name, mm) in wheels) {
      if (name == values['wheel'] && mm != null) return mm;
    }
    return values.containsKey('wheel_mm') ? i('wheel_mm') : 2240;
  }

  /// (cutoff mV, full mV, battery bars mV) like ebics_core.battery_mv
  (int, int, List<int>) batteryMv() {
    final cells = values.containsKey('battery') ? i('battery') : 10;
    var cutoff = cells * 3100;
    var full = cells * 4200;
    if (d('voltage_min_v') > 0) cutoff = (d('voltage_min_v') * 1000).toInt();
    if (d('full_voltage_v') > 0) full = (d('full_voltage_v') * 1000).toInt();
    final span = (full - cutoff) < 1 ? 1 : full - cutoff;
    final bars = [0.10, 0.25, 0.45, 0.65, 0.85].map((f) => (cutoff + span * f).toInt()).toList();
    return (cutoff, full, bars);
  }

  /// Problems that would make a bad firmware (ebics_core.check_settings), empty if fine.
  List<String> problems() {
    final p = <String>[];
    final (cutoff, full, _) = batteryMv();
    if (i('battery') == 0 && (d('voltage_min_v') <= 0 || d('full_voltage_v') <= 0)) {
      p.add("Custom battery: set both 'Full charge voltage' and 'Cutoff voltage'.");
    }
    if (full <= cutoff + 1000) p.add("'Full charge voltage' must be clearly above the 'Cutoff voltage'.");
    if (d('voltage_ramp_v') * 1000 >= (full - cutoff) / 2) {
      p.add("'Soft voltage ramp width' is too big for this battery voltage range.");
    }
    final idx = i('speed_temp_input');
    if (idx == 6 && b('torque_on_ad1')) p.add("Speed/temperature wire and torque sensor can't both use AD1.");
    if (idx == 5 && b('linear_regen')) p.add("Speed/temperature wire and proportional regen can't both use AD2.");
    if (idx != 0 && !(b('speed_on_wire') || b('temp_on_wire'))) {
      p.add('Speed/temperature wire is selected but neither speed nor temperature is ticked.');
    }
    // limits the firmware checks before it accepts the settings block (Src/settings.c)
    if (cutoff < 10000 || full > 100000) p.add('Battery voltages must be between 10 V and 100 V.');
    final sl = i('speed_limit');
    if (sl < 1 || sl > 100) p.add('Speed limit must be 1..100 km/h.');
    final wc = wheelCircumference;
    if (wc < 500 || wc > 3500) p.add('Wheel circumference must be 500..3500 mm.');
    final pm = i('pas_magnets');
    if (pm < 1 || pm > 64) p.add('Pedal sensor magnets must be 1..64.');
    if (i('pas_timeout') <= i('ramp_end') || i('ramp_end') < 1) p.add("'PAS stop timeout' must be above 'PAS full assist at'.");
    final g = i('gear_ratio');
    if (g < 1 || g > 1000) p.add("'Pole pairs x gear ratio' must be 1..1000.");
    if (d('phase_current_a') <= 0) p.add('Motor current must be above 0.');
    if (d('battery_current_a') <= 0 || d('battery_current_a') > 60) p.add('Battery current must be 0..60 A.');
    return p;
  }
}
