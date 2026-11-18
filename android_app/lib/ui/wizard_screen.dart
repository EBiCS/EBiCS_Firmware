import 'dart:async';

import 'package:material_ui/material_ui.dart';

import '../app_state.dart';
import '../controller/controller.dart';
import '../firmware/bike_settings.dart';
import '../usb/usb_bridge.dart';
import 'common.dart';

/// First setup, same steps as the desktop wizard (setup_app/ebics_setup.py, WizardPage).
/// Every step has Back, Retry and Skip, so the user can never get stuck.
class WizardScreen extends StatefulWidget {
  const WizardScreen({super.key});
  @override
  State<WizardScreen> createState() => _WizardScreenState();
}

/// What a step tells the wizard.
class StepHost {
  final void Function(bool done) setDone;
  final void Function(Future<bool> Function()? f) setBeforeNext;
  final VoidCallback next;
  final void Function() markSettingsChanged;
  StepHost(this.setDone, this.setBeforeNext, this.next, this.markSettingsChanged);
}

class _StepDef {
  final String title;
  final Widget Function(StepHost host) build;
  const _StepDef(this.title, this.build);
}

class _WizardScreenState extends State<WizardScreen> {
  int _i = 0;
  int _attempt = 0; // Retry = new key = fresh step state
  bool _done = false;
  bool _settingsChanged = false;
  Future<bool> Function()? _beforeNext;

  late final List<_StepDef> _steps = [
    _StepDef('First setup', (h) => _WelcomeStep(h)),
    _StepDef('Connect', (h) => _ConnectStep(h)),
    _StepDef('Unlock the chip', (h) => _UnlockStep(h)),
    _StepDef('Your bike', (h) => _BikeStep(h)),
    _StepDef('Install the firmware', (h) => _InstallStep(h)),
    _StepDef('Motor calibration', (h) => _CalibrateStep(h)),
    _StepDef('Motor direction', (h) => _DirectionStep(h)),
    _StepDef('Measure the motor gearing', (h) => _GearStep(h)),
    _StepDef('Update the settings', (h) => _UpdateStep(h)),
    _StepDef('Test', (h) => _TestStep(h)),
    _StepDef('Finished', (h) => _DoneStep(h)),
  ];

  @override
  void initState() {
    super.initState();
    UsbBridge.keepScreenOn(true);
  }

  @override
  void dispose() {
    UsbBridge.keepScreenOn(false);
    super.dispose();
  }

  void _go(int i) {
    setState(() {
      _i = i.clamp(0, _steps.length - 1);
      _attempt++;
      _done = false;
      _beforeNext = null;
    });
  }

  Future<void> _next({bool skip = false}) async {
    if (!skip && _beforeNext != null && !await _beforeNext!()) return;
    if (_i == _steps.length - 1) {
      if (mounted) Navigator.of(context).pop();
      return;
    }
    var n = _i + 1;
    // the settings update step is only needed when direction or gearing changed
    if (_steps[n].title == 'Update the settings' && !_settingsChanged) n++;
    _go(n);
  }

  late final _host = StepHost(
    (d) {
      if (mounted && d != _done) setState(() => _done = d);
    },
    (f) => _beforeNext = f,
    () => _next(),
    () => _settingsChanged = true,
  );

  void _settingsUpdated() => _settingsChanged = false;

  @override
  Widget build(BuildContext context) {
    final step = _steps[_i];
    final last = _i == _steps.length - 1;
    return Scaffold(
      appBar: AppBar(
        title: Text(step.title),
        bottom: PreferredSize(
          preferredSize: const Size.fromHeight(4),
          child: LinearProgressIndicator(value: (_i + 1) / _steps.length),
        ),
      ),
      body: SafeArea(
        child: Column(children: [
          Padding(
            padding: const EdgeInsets.fromLTRB(16, 8, 16, 0),
            child: Align(
              alignment: Alignment.centerLeft,
              child: Text('Step ${_i + 1} of ${_steps.length}', style: Theme.of(context).textTheme.labelMedium),
            ),
          ),
          Expanded(
            child: _UpdateScope(
              onUpdated: _settingsUpdated,
              child: KeyedSubtree(key: ValueKey('$_i/$_attempt'), child: step.build(_host)),
            ),
          ),
          const Divider(height: 1),
          Padding(
            padding: const EdgeInsets.fromLTRB(8, 8, 8, 8),
            child: Row(children: [
              Flexible(child: TextButton(onPressed: _i > 0 ? () => _go(_i - 1) : null, child: const _BarText('Back'))),
              Flexible(child: TextButton(onPressed: () => _go(_i), child: const _BarText('Retry'))),
              const Spacer(),
              if (!last && !_done)
                Flexible(child: TextButton(onPressed: () => _next(skip: true), child: const _BarText('Skip'))),
              const SizedBox(width: 4),
              Flexible(
                flex: 2,
                child: FilledButton(
                  onPressed: _done || last ? () => _next() : null,
                  child: _BarText(last ? 'Close' : 'Next'),
                ),
              ),
            ]),
          ),
        ]),
      ),
    );
  }
}

class _BarText extends StatelessWidget {
  final String text;
  const _BarText(this.text);
  @override
  Widget build(BuildContext context) => Text(text, maxLines: 1, overflow: TextOverflow.ellipsis, softWrap: false);
}

class _UpdateScope extends InheritedWidget {
  final VoidCallback onUpdated;
  const _UpdateScope({required this.onUpdated, required super.child});
  static _UpdateScope of(BuildContext c) =>
      c.getElementForInheritedWidgetOfExactType<_UpdateScope>()!.widget as _UpdateScope;
  @override
  bool updateShouldNotify(_UpdateScope old) => false;
}

/// Scrollable page body.
class _Page extends StatelessWidget {
  final List<Widget> children;
  const _Page(this.children);
  @override
  Widget build(BuildContext context) => ListView(padding: const EdgeInsets.all(16), children: children);
}

Widget _p(BuildContext context, String t, {bool muted = false}) => Padding(
      padding: const EdgeInsets.only(bottom: 10),
      child: Text(t,
          style: muted
              ? Theme.of(context).textTheme.bodyMedium?.copyWith(color: Theme.of(context).colorScheme.onSurfaceVariant)
              : Theme.of(context).textTheme.bodyLarge),
    );

// ======================================================================== steps

class _WelcomeStep extends StatefulWidget {
  final StepHost host;
  const _WelcomeStep(this.host);
  @override
  State<_WelcomeStep> createState() => _WelcomeStepState();
}

class _WelcomeStepState extends State<_WelcomeStep> {
  @override
  void initState() {
    super.initState();
    WidgetsBinding.instance.addPostFrameCallback((_) => widget.host.setDone(true));
  }

  @override
  Widget build(BuildContext context) => _Page([
        _p(context, 'This guides you through everything, one step at a time. You need:'),
        const Bullets([
          'An ST-Link V2 programmer, plugged into the phone with a USB OTG adapter, wired to the controller\'s '
              'programming plug: SWDIO, SWCLK and GND (don\'t connect 3.3 V when the battery powers the controller)',
          'The controller connected to the motor and switched on from the battery',
          'The wheel lifted off the ground, so it can spin freely',
          'About 10 minutes',
        ]),
        _p(context,
            'Lishui programming plug: BLACK = SWDIO, YELLOW = GND, GREEN = SWCLK, RED = 3.3 V. '
            'Check with a multimeter, colours can differ.',
            muted: true),
        _p(context,
            'The motor calibration (hall angles, KV) is kept in the last 2 KB of the chip. This app never erases it '
            'when installing or updating.',
            muted: true),
      ]);
}

// ------------------------------------------------------------------------ connect

class _ConnectStep extends StatefulWidget {
  final StepHost host;
  const _ConnectStep(this.host);
  @override
  State<_ConnectStep> createState() => _ConnectStepState();
}

class _ConnectStepState extends State<_ConnectStep> {
  ProbeResult? _r;
  Timer? _t;
  bool _busy = false;

  @override
  void initState() {
    super.initState();
    _check(ask: true);
  }

  @override
  void dispose() {
    _t?.cancel();
    super.dispose();
  }

  Future<void> _check({bool ask = false}) async {
    if (_busy) return;
    _busy = true;
    final r = await AppScope.read(context).controller.probe(askPermission: ask);
    _busy = false;
    if (!mounted) return;
    setState(() => _r = r);
    widget.host.setDone(r.targetFound);
    if (!r.targetFound) _t = Timer(const Duration(seconds: 2), _check);
  }

  @override
  Widget build(BuildContext context) {
    final r = _r;
    return _Page([
      _p(context, 'Plug in the ST-Link and switch the controller on. This page checks every 2 seconds.'),
      StatusLine(r == null ? 'Checking ...' : r.describe(),
          tone: r == null ? Tone.busy : (r.targetFound ? Tone.ok : Tone.bad)),
      if (r != null && r.stlinkFound && !r.permission)
        Align(
          alignment: Alignment.centerLeft,
          child: FilledButton(onPressed: () => _check(ask: true), child: const Text('Allow USB access')),
        ),
    ]);
  }
}

// ------------------------------------------------------------------------ unlock

class _UnlockStep extends StatefulWidget {
  final StepHost host;
  const _UnlockStep(this.host);
  @override
  State<_UnlockStep> createState() => _UnlockStepState();
}

class _UnlockStepState extends State<_UnlockStep> {
  String _status = 'Reading the chip protection ...';
  Tone _tone = Tone.busy;
  bool? _locked;
  bool _agree = false;
  bool _busy = false;
  final _log = <String>[];

  @override
  void initState() {
    super.initState();
    _read();
  }

  Future<void> _read() async {
    final c = AppScope.read(context).controller;
    try {
      if (!c.connected) await c.probe();
      final locked = await c.readProtected();
      if (!mounted) return;
      setState(() {
        _locked = locked;
        _status = locked ? 'The chip is locked (original Lishui firmware).' : 'The chip is unlocked - nothing to do.';
        _tone = locked ? Tone.warn : Tone.ok;
      });
      widget.host.setDone(!locked);
    } catch (e) {
      if (!mounted) return;
      setState(() {
        _locked = true;
        _status = 'Could not read the protection state: $e';
        _tone = Tone.bad;
      });
    }
  }

  Future<void> _unlock() async {
    setState(() {
      _busy = true;
      _log.clear();
      _status = 'Unlocking ...';
      _tone = Tone.busy;
    });
    final c = AppScope.read(context).controller;
    try {
      if (!c.connected) await c.probe();
      await c.unlock((l) {
        if (mounted) setState(() => _log.add(l));
      });
      await c.disconnect();
      if (!mounted) return;
      setState(() {
        _status = 'Unlocked. Now switch the controller OFF and ON again, then tap Next.';
        _tone = Tone.ok;
      });
      widget.host.setDone(true);
    } catch (e) {
      if (!mounted) return;
      setState(() {
        _status = 'Unlock failed: $e\nCheck wiring and power, then tap Retry.';
        _tone = Tone.bad;
      });
    } finally {
      if (mounted) setState(() => _busy = false);
    }
  }

  @override
  Widget build(BuildContext context) => _Page([
        StatusLine(_status, tone: _tone),
        if (_locked == true && _tone != Tone.ok) ...[
          _p(context,
              'To install EBiCS the chip must be unlocked once. This ERASES the original firmware for good - '
              'it cannot be backed up or restored.'),
          CheckboxListTile(
            contentPadding: EdgeInsets.zero,
            value: _agree,
            onChanged: _busy ? null : (v) => setState(() => _agree = v == true),
            title: const Text('I understand the original firmware will be erased'),
          ),
          Align(
            alignment: Alignment.centerLeft,
            child: FilledButton(onPressed: _agree && !_busy ? _unlock : null, child: const Text('Unlock chip')),
          ),
        ],
        LogBox(_log),
      ]);
}

// ------------------------------------------------------------------------ bike

class _BikeStep extends StatefulWidget {
  final StepHost host;
  const _BikeStep(this.host);
  @override
  State<_BikeStep> createState() => _BikeStepState();
}

class _BikeStepState extends State<_BikeStep> {
  late BikeSettings _s;
  static const _keys = [
    'display', 'battery', 'full_voltage_v', 'voltage_min_v', 'wheel', 'wheel_mm', 'speed_limit',
    'phase_current_a', 'battery_current_a', 'pas_magnets', 'torque_sensor', 'throttle', 'legal_mode',
    'sensorless', 'speed_temp_input', 'speed_on_wire', 'temp_on_wire', 'ntc_pullup_ohm',
  ];

  @override
  void initState() {
    super.initState();
    _s = AppScope.read(context).settings.copy();
    widget.host.setBeforeNext(_save);
    WidgetsBinding.instance.addPostFrameCallback((_) => widget.host.setDone(true));
  }

  Future<bool> _save() async {
    final p = _s.problems();
    if (p.isNotEmpty) {
      await showDialog<void>(
        context: context,
        builder: (c) => AlertDialog(
          title: const Text('Please check'),
          content: Text(p.join('\n\n')),
          actions: [TextButton(onPressed: () => Navigator.pop(c), child: const Text('OK'))],
        ),
      );
      return false;
    }
    AppScope.read(context).save(_s);
    return true;
  }

  bool _visible(String k) {
    if (k == 'wheel_mm') return _s.s('wheel') == 'custom';
    if (k == 'speed_on_wire' || k == 'temp_on_wire') return _s.i('speed_temp_input') != 0;
    if (k == 'ntc_pullup_ohm') return _s.i('speed_temp_input') != 0 && _s.b('temp_on_wire');
    return true;
  }

  @override
  Widget build(BuildContext context) => _Page([
        _p(context, 'A few questions. Tap ? for help. You can change all of this later in Settings.'),
        for (final k in _keys)
          if (_visible(k))
            FieldEditor(
              key: ValueKey(k),
              field: fieldByKey[k]!,
              settings: _s,
              onChanged: (v) => setState(() => _s[k] = v),
            ),
        _p(context, _s.displayType.description, muted: true),
      ]);
}

// ------------------------------------------------------------------------ install

class _InstallStep extends StatefulWidget {
  final StepHost host;
  const _InstallStep(this.host);
  @override
  State<_InstallStep> createState() => _InstallStepState();
}

class _InstallStepState extends State<_InstallStep> {
  String _status = 'Connecting ...';
  Tone _tone = Tone.busy;
  double? _progress;
  final _log = <String>[];

  @override
  void initState() {
    super.initState();
    WidgetsBinding.instance.addPostFrameCallback((_) => _run());
  }

  Future<void> _run() async {
    final app = AppScope.read(context);
    final c = app.controller;
    final s = app.settings;
    try {
      final v = await app.repo.variantFor(s);
      if (v == null) throw StateError('no prebuilt firmware for ${s.display}');
      _add('firmware: ${v.displayLabel}, ${v.sensorless ? 'sensorless' : 'hall sensors'}, '
          '${v.size} bytes (${await app.repo.version()})');
      final pr = await c.probe();
      if (!pr.targetFound) throw Exception(pr.describe());
      _add(pr.describe());
      String last = '';
      await c.install(s, (step, p) {
        if (!mounted) return;
        if (step != last) _add(step);
        last = step;
        setState(() {
          _status = '$step ...';
          _progress = p;
        });
      });
      if (!mounted) return;
      _add('firmware installed and verified, controller restarted');
      setState(() {
        _status = 'Firmware installed.';
        _tone = Tone.ok;
        _progress = 1;
      });
      widget.host.setDone(true);
    } catch (e) {
      _add('ERROR: $e');
      if (!mounted) return;
      setState(() {
        _status = 'Installing failed - see below. Check the ST-Link wiring and that the controller is on, then Retry.';
        _tone = Tone.bad;
        _progress = 0;
      });
    }
  }

  void _add(String l) {
    if (mounted) setState(() => _log.add(l));
  }

  @override
  Widget build(BuildContext context) => _Page([
        _p(context,
            'The firmware for your display is installed on the controller with your settings. '
            'This takes about half a minute. The motor calibration stays.'),
        StatusLine(_status, tone: _tone),
        LinearProgressIndicator(value: _progress),
        LogBox(_log, height: 220),
      ]);
}

// ------------------------------------------------------------------------ calibrate

class _CalibrateStep extends StatefulWidget {
  final StepHost host;
  const _CalibrateStep(this.host);
  @override
  State<_CalibrateStep> createState() => _CalibrateStepState();
}

class _CalibrateStepState extends State<_CalibrateStep> {
  final _checks = [false, false, false];
  bool _running = false;
  String _status = '';
  String _detail = '';
  Tone _tone = Tone.muted;
  double _progress = 0;
  Timer? _t;
  DateTime _t0 = DateTime.now();
  DateTime? _stoppedSince;
  int? _lastHalls;

  @override
  void initState() {
    super.initState();
    if (AppScope.read(context).settings.sensorless) {
      WidgetsBinding.instance.addPostFrameCallback((_) => widget.host.setDone(true));
    }
  }

  @override
  void dispose() {
    _t?.cancel();
    super.dispose();
  }

  void _set(String status, Tone tone, [String? detail]) {
    if (!mounted) return;
    setState(() {
      _status = status;
      _tone = tone;
      if (detail != null) _detail = detail;
    });
  }

  Future<void> _start() async {
    final c = AppScope.read(context).controller;
    setState(() => _running = true);
    _set('Starting ...', Tone.busy, '');
    try {
      if (!c.connected) {
        final pr = await c.probe();
        if (!pr.targetFound) throw Exception(pr.describe());
      }
      if (!c.hasSymbols && await c.identify() == null) {
        throw Exception('the firmware on the controller is not one of this app\'s - go back and install the firmware');
      }
      final snap = await c.snapshot();
      if (snap == null) throw Exception('no connection to the controller');
      if (snap['ui32_setup_magic'] != setupMagic) {
        throw Exception('the controller runs an old firmware - go back and install the firmware');
      }
      await c.startCalibration();
      _t0 = DateTime.now();
      _stoppedSince = null;
      _lastHalls = null;
      _poll();
    } catch (e) {
      _set('Could not start: $e', Tone.bad);
      if (mounted) setState(() => _running = false);
    }
  }

  Future<void> _poll() async {
    if (!mounted) return;
    final c = AppScope.read(context).controller;
    Map<String, int>? snap;
    try {
      snap = await c.snapshot();
    } catch (_) {}
    if (!mounted) return;
    if (snap == null) {
      setState(() => _detail = '(no answer from the controller this time, retrying ...)');
      _t = Timer(const Duration(milliseconds: 500), _poll);
      return;
    }
    final state = snap['ui8_setup_state'];
    final prog = snap['ui8_setup_progress'] ?? 0;
    final halls = snap['ui32_hall_events'];
    switch (state) {
      case 0:
        if (DateTime.now().difference(_t0).inSeconds > 8) {
          _set('The controller did not start the calibration.', Tone.bad,
              'Is the motor running (throttle/pedals)? Release everything and tap Retry.');
          setState(() => _running = false);
          return;
        }
        _set('Starting ...', Tone.busy);
      case 1:
        _set('1/2  Finding the hall sensor positions ...', Tone.busy,
            'The motor turns slowly and may click or buzz - that is normal. About 6 seconds.');
        setState(() => _progress = prog / 100);
      case 2:
        _set('2/2  Measuring the motor speed ...', Tone.busy,
            'The wheel speeds up to full speed. This can take a few minutes. Keep clear of the wheel.');
        setState(() => _progress = prog / 100);
      case 4:
        final found = snap['ui8_setup_halls_found'] ?? 0;
        final n = List.generate(6, (i) => (found >> i) & 1).reduce((a, b) => a + b);
        _set('Calibration failed: not all hall sensor positions were found.', Tone.bad,
            'Found $n of 6. Check the hall sensor plug (5 thin wires) and that the wheel can turn freely, '
            'then switch the controller off and on and tap Retry.');
        setState(() => _running = false);
        return;
      case 3:
        setState(() => _progress = 1);
        if (halls == _lastHalls) {
          _stoppedSince ??= DateTime.now();
        } else {
          _stoppedSince = null;
        }
        if (_stoppedSince != null && DateTime.now().difference(_stoppedSince!).inMilliseconds > 1500) {
          Map<String, int>? full;
          try {
            full = await c.snapshot(withHalls: true);
          } catch (_) {}
          _finish(full);
          return;
        }
        _set('Measured. Please wait until the wheel has stopped ...', Tone.warn,
            "Don't touch the wheel - it is still coasting.");
    }
    _lastHalls = halls;
    _t = Timer(const Duration(milliseconds: 300), _poll);
  }

  void _finish(Map<String, int>? snap) {
    final ang = hallAnglesDeg(snap);
    var msg = '';
    if (ang != null) {
      msg = 'Hall angles: ${ang.map((a) => '${a.round()}°').join(', ')}';
      msg += checkHallAngles(ang)
          ? '  - look good.'
          : '  - unusual spacing. It may still work; if the motor runs rough, calibrate again.';
    }
    final kv = snap?['ui32_KV'] ?? 0;
    if (kv != 0) msg += '\nMotor KV value: $kv';
    _set('Calibration done, the wheel has stopped.', Tone.ok, msg);
    widget.host.setDone(true);
  }

  @override
  Widget build(BuildContext context) {
    final s = AppScope.of(context).settings;
    if (s.sensorless) {
      return _Page([
        _p(context, 'Sensorless motor: there are no hall sensors to calibrate.'),
        const Bullets([
          "Check the motor parameters in Settings > Expert > 'Sensorless' (inductance, resistance, flux linkage, "
              'gain, angle offset).',
          'The motor starts with a short open loop spin before the observer takes over.',
          'Sensorless is experimental: test with the wheel lifted, low current first.',
        ]),
      ]);
    }
    const texts = [
      'The wheel is lifted and can spin freely',
      'Nothing touches the wheel, chain or pedals',
      'Hands and cables are away from the wheel',
    ];
    return _Page([
      _p(context, "The controller now finds the position of the motor's hall sensors and measures the motor. Before you start:"),
      for (var i = 0; i < 3; i++)
        CheckboxListTile(
          contentPadding: EdgeInsets.zero,
          value: _checks[i],
          onChanged: _running ? null : (v) => setState(() => _checks[i] = v == true),
          title: Text(texts[i]),
        ),
      Align(
        alignment: Alignment.centerLeft,
        child: FilledButton(
          onPressed: !_running && _checks.every((x) => x) ? _start : null,
          child: const Text('Start calibration'),
        ),
      ),
      StatusLine(_status, tone: _tone),
      if (_running || _progress > 0) LinearProgressIndicator(value: _progress),
      const SizedBox(height: 8),
      _p(context, _detail, muted: true),
    ]);
  }
}

// ------------------------------------------------------------------------ direction

class _DirectionStep extends StatefulWidget {
  final StepHost host;
  const _DirectionStep(this.host);
  @override
  State<_DirectionStep> createState() => _DirectionStepState();
}

class _DirectionStepState extends State<_DirectionStep> {
  String _status = '';
  Tone _tone = Tone.muted;

  void _answer(bool forward) {
    final app = AppScope.read(context);
    if (forward) {
      setState(() {
        _status = 'Direction is correct.';
        _tone = Tone.ok;
      });
    } else {
      final s = app.settings.copy();
      s['reverse'] = !s.b('reverse');
      app.save(s);
      widget.host.markSettingsChanged();
      setState(() {
        _status = 'OK - the direction will be reversed. The settings are written to the controller in a moment.';
        _tone = Tone.warn;
      });
    }
    widget.host.setDone(true);
  }

  @override
  Widget build(BuildContext context) {
    final s = AppScope.of(context).settings;
    return _Page([
      _p(
          context,
          s.sensorless || !s.b('throttle')
              ? 'Give the motor a short spin (throttle, or turn the pedals with the wheel lifted). Which way did the wheel turn?'
              : 'During the calibration the wheel spun up. Which way did it turn?'),
      Wrap(spacing: 12, children: [
        FilledButton(onPressed: () => _answer(true), child: const Text('Forward (correct)')),
        OutlinedButton(onPressed: () => _answer(false), child: const Text('Backward')),
      ]),
      StatusLine(_status, tone: _tone),
    ]);
  }
}

// ------------------------------------------------------------------------ gear ratio

class _GearStep extends StatefulWidget {
  final StepHost host;
  const _GearStep(this.host);
  @override
  State<_GearStep> createState() => _GearStepState();
}

class _GearStepState extends State<_GearStep> {
  int? _start;
  String _status = '';
  Tone _tone = Tone.muted;

  @override
  void initState() {
    super.initState();
    if (AppScope.read(context).settings.sensorless) {
      WidgetsBinding.instance.addPostFrameCallback((_) => widget.host.setDone(true));
    }
  }

  Future<Map<String, int>?> _snap() async {
    final c = AppScope.read(context).controller;
    try {
      if (!c.connected) await c.probe();
      if (!c.hasSymbols) await c.identify();
      return await c.snapshot();
    } catch (_) {
      return null;
    }
  }

  Future<void> _begin() async {
    final snap = await _snap();
    if (!mounted) return;
    if (snap == null || snap['ui32_hall_events'] == null) {
      setState(() {
        _status = 'No connection to the controller (or unknown firmware).';
        _tone = Tone.bad;
      });
      return;
    }
    setState(() {
      _start = snap['ui32_hall_events'];
      _status = 'Measuring - turn the wheel backwards 3 full turns, then tap Done.';
      _tone = Tone.busy;
    });
  }

  Future<void> _finish() async {
    final snap = await _snap();
    if (!mounted) return;
    if (snap == null || _start == null) {
      setState(() {
        _status = 'No connection to the controller.';
        _tone = Tone.bad;
      });
      return;
    }
    final events = (snap['ui32_hall_events']! - _start!) & 0xFFFFFFFF;
    final ratio = (events / 18.0).round(); // 6 hall events per electrical turn, 3 wheel turns
    if (ratio < 4) {
      setState(() {
        _status = 'Only $events hall events counted - did the motor turn? Try again (turn backwards, not forwards).';
        _tone = Tone.bad;
      });
      return;
    }
    final app = AppScope.read(context);
    if (ratio != app.settings.i('gear_ratio')) {
      final s = app.settings.copy();
      s['gear_ratio'] = ratio;
      app.save(s);
      widget.host.markSettingsChanged();
    }
    setState(() {
      _status = 'Measured: $ratio  ($events hall events in 3 turns). Saved.';
      _tone = Tone.ok;
    });
    widget.host.setDone(true);
  }

  @override
  Widget build(BuildContext context) {
    final s = AppScope.of(context).settings;
    if (s.sensorless) {
      return _Page([
        _p(context,
            "Without hall sensors the gearing can't be measured by turning the wheel. Enter 'Pole pairs x gear ratio' "
            'in Settings, or use an external speed sensor / the speed wire (Settings > Advanced) for a correct speed.'),
      ]);
    }
    return _Page([
      _p(context, 'This makes the speed on the display correct. Put a mark on the tyre (e.g. at the valve).'),
      const Bullets(["Tap 'Start measuring'", 'Turn the wheel slowly BACKWARDS by hand, exactly 3 full turns', "Tap 'Done'"]),
      _p(context, '(Backwards, because geared hub motors freewheel when the wheel turns forward.)', muted: true),
      Wrap(spacing: 12, children: [
        FilledButton(onPressed: _begin, child: const Text('Start measuring')),
        OutlinedButton(onPressed: _start == null ? null : _finish, child: const Text('Done')),
      ]),
      StatusLine(_status.isEmpty ? 'Current value: ${s.i('gear_ratio')}   (you can also skip this step)' : _status,
          tone: _status.isEmpty ? Tone.muted : _tone),
    ]);
  }
}

// ------------------------------------------------------------------------ settings update

class _UpdateStep extends StatefulWidget {
  final StepHost host;
  const _UpdateStep(this.host);
  @override
  State<_UpdateStep> createState() => _UpdateStepState();
}

class _UpdateStepState extends State<_UpdateStep> {
  String _status = 'Writing the settings ...';
  Tone _tone = Tone.busy;

  @override
  void initState() {
    super.initState();
    WidgetsBinding.instance.addPostFrameCallback((_) => _run());
  }

  Future<void> _run() async {
    final app = AppScope.read(context);
    final c = app.controller;
    final onUpdated = _UpdateScope.of(context).onUpdated;
    try {
      if (!c.connected) {
        final pr = await c.probe();
        if (!pr.targetFound) throw Exception(pr.describe());
      }
      if (c.installed == null) await c.identify();
      await c.updateSettings(app.settings);
      onUpdated();
      if (!mounted) return;
      setState(() {
        _status = 'Settings written, controller restarted. The motor calibration is kept.';
        _tone = Tone.ok;
      });
      widget.host.setDone(true);
    } catch (e) {
      if (!mounted) return;
      setState(() {
        _status = 'Writing the settings failed: $e';
        _tone = Tone.bad;
      });
    }
  }

  @override
  Widget build(BuildContext context) => _Page([
        _p(context, 'Your answers changed the settings (direction or gearing), so they are written to the controller.'),
        StatusLine(_status, tone: _tone),
        if (_tone == Tone.busy) const LinearProgressIndicator(),
      ]);
}

// ------------------------------------------------------------------------ test

class _TestStep extends StatefulWidget {
  final StepHost host;
  const _TestStep(this.host);
  @override
  State<_TestStep> createState() => _TestStepState();
}

class _TestStepState extends State<_TestStep> {
  final _seen = <String>{};
  Map<String, int>? _base;
  Timer? _t;
  String _status = 'Waiting for input ...';
  Tone _tone = Tone.muted;
  bool _stopped = false;

  @override
  void initState() {
    super.initState();
    WidgetsBinding.instance.addPostFrameCallback((_) {
      widget.host.setDone(true);
      _poll();
    });
  }

  @override
  void dispose() {
    _stopped = true;
    _t?.cancel();
    super.dispose();
  }

  Map<String, String> _items(BikeSettings s) => {
        'display': 'Display: change the assist level on the display',
        if (s.b('throttle')) 'throttle': 'Throttle: twist it - the wheel should turn',
        'brake': 'Brake: pull a brake lever',
        'pas': 'Pedal sensor: turn the pedals forward by hand',
      };

  Future<void> _poll() async {
    if (_stopped || !mounted) return;
    final app = AppScope.read(context);
    final c = app.controller;
    Map<String, int>? snap;
    try {
      if (!c.connected) await c.probe(askPermission: false);
      if (!c.hasSymbols) await c.identify();
      snap = await c.snapshot();
    } catch (_) {}
    if (_stopped || !mounted) return;
    if (snap != null) {
      _base ??= snap;
      final b = _base!;
      if (snap['MS.assist_level'] != b['MS.assist_level']) _seen.add('display');
      if ((snap['ui16_throttle'] ?? 0) > (snap['ui16_throttle_offset'] ?? 0) + 150) _seen.add('throttle');
      if ((snap['brake_flag'] ?? 0) != 0) _seen.add('brake');
      if ((snap['ui32_pas_events'] ?? 0) - (b['ui32_pas_events'] ?? 0) > 4) _seen.add('pas');
      final all = _seen.containsAll(_items(app.settings).keys);
      setState(() {
        _status = all ? 'All good!' : 'Waiting for input ...';
        _tone = all ? Tone.ok : Tone.muted;
      });
    } else {
      setState(() {
        _status = 'No connection to the controller ...';
        _tone = Tone.bad;
      });
    }
    _t = Timer(const Duration(milliseconds: 300), _poll);
  }

  @override
  Widget build(BuildContext context) {
    final items = _items(AppScope.of(context).settings);
    return _Page([
      _p(context, 'Try each item. It gets a tick when the controller sees it. (Wheel still lifted!)'),
      for (final e in items.entries)
        ListTile(
          contentPadding: EdgeInsets.zero,
          leading: Icon(_seen.contains(e.key) ? Icons.check_circle : Icons.radio_button_unchecked,
              color: _seen.contains(e.key) ? okColor : null),
          title: Text(e.value),
        ),
      StatusLine(_status, tone: _tone),
      _p(context, "If something doesn't get a tick, you can still finish and check it later in Live data.", muted: true),
    ]);
  }
}

// ------------------------------------------------------------------------ done

class _DoneStep extends StatefulWidget {
  final StepHost host;
  const _DoneStep(this.host);
  @override
  State<_DoneStep> createState() => _DoneStepState();
}

class _DoneStepState extends State<_DoneStep> {
  @override
  void initState() {
    super.initState();
    WidgetsBinding.instance.addPostFrameCallback((_) => widget.host.setDone(true));
  }

  @override
  Widget build(BuildContext context) => _Page([
        _p(context, 'Your controller is set up. Lower the wheel and go for a careful test ride.'),
        const Bullets([
          "Change power, speed limit and more later in Settings, then 'Write to controller'",
          'Live data shows what the controller sees (battery, throttle, brake, speed ...)',
          'If you change the motor or swap wires, run this setup again',
        ]),
      ]);
}
