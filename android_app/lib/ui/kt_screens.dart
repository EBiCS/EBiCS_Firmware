import 'package:flutter/services.dart';
import 'package:material_ui/material_ui.dart';

import '../app_state.dart';
import '../kunteng/kt_live.dart';
import '../kunteng/kt_settings.dart';
import 'common.dart';

/// Kunteng (KT, STM8S105) controller: check, unlock, settings, install - over the ST-Link SWIM pin.
class KtHomeScreen extends StatefulWidget {
  const KtHomeScreen({super.key});
  @override
  State<KtHomeScreen> createState() => _KtHomeScreenState();
}

class _KtHomeScreenState extends State<KtHomeScreen> {
  final _log = <String>[];
  bool _busy = false;

  void _add(String l) {
    if (mounted) setState(() => _log.add(l));
  }

  Future<void> _run(String title, Future<void> Function() f) async {
    setState(() {
      _busy = true;
      _log.add('--- $title');
    });
    try {
      await UsbBridgeKeepAwake.on();
      await f();
    } catch (e) {
      _add('ERROR: $e');
    } finally {
      await UsbBridgeKeepAwake.off();
      if (mounted) setState(() => _busy = false);
    }
  }

  Future<void> _check() => _run('check connection', () async {
    _add((await AppScope.read(context).ktController.probe()).describe());
  });

  Future<void> _unlock() async {
    final ok = await confirm(
      context,
      'Unlock chip',
      'Unlocking erases the whole chip, including the original KT firmware. It cannot be read out or restored.\n'
          'Continue?',
      yes: 'Unlock',
    );
    if (!ok || !mounted) return;
    await _run('unlock', () async {
      await AppScope.read(context).ktController.unlock(_add);
      _add('unlocked - switch the controller off and on again, then install');
    });
  }

  Future<void> _install() async {
    final app = AppScope.read(context);
    final p = app.kt.problems();
    if (p.isNotEmpty) {
      _add('fix the settings first: ${p.join(' ')}');
      return;
    }
    if (!await app.ktController.hasFirmware()) {
      _add('ERROR: this app build has no Kunteng firmware (assets/firmware/kunteng.ihx)');
      return;
    }
    if (!mounted) return;
    final ok = await confirm(
      context,
      'Install firmware',
      'Install the KT firmware with the current settings? Test with the wheel lifted first.',
      yes: 'Install',
    );
    if (!ok || !mounted) return;
    await _run('install firmware', () async {
      var last = '';
      var lastPct = -1;
      await app.ktController.install(app.kt, (step, prog) {
        final pct = (prog * 10).floor() * 10;
        if (step != last || pct != lastPct) _add('$step ${step == 'writing firmware' ? '$pct %' : ''}');
        last = step;
        lastPct = pct;
      });
      _add('firmware installed, verified, settings written, controller restarted');
    });
  }

  void _open(Widget w) => Navigator.of(context).push(MaterialPageRoute<void>(builder: (_) => w));

  @override
  Widget build(BuildContext context) {
    final app = AppScope.of(context);
    final s = app.kt;
    final theme = Theme.of(context);
    Widget step(int n, String text, String button, VoidCallback f) => Card(
      color: theme.colorScheme.surfaceContainer,
      child: Padding(
        padding: const EdgeInsets.all(12),
        child: Row(
          children: [
            CircleAvatar(radius: 14, child: Text('$n')),
            const SizedBox(width: 12),
            Expanded(child: Text(text)),
            const SizedBox(width: 8),
            OutlinedButton(onPressed: _busy ? null : f, child: Text(button)),
          ],
        ),
      ),
    );
    final ctrl = ktChoices['controller']!.firstWhere((c) => c.$2 == s.i('controller'), orElse: () => ('?', 0)).$1;
    return Scaffold(
      appBar: AppBar(
        title: const Text('Kunteng (KT) setup'),
        actions: [
          IconButton(
            icon: const Icon(Icons.swap_horiz),
            tooltip: 'Change controller',
            onPressed: () => app.chooseKind(null),
          ),
        ],
      ),
      body: ListView(
        padding: const EdgeInsets.all(16),
        children: [
          Text('Kunteng controller', style: theme.textTheme.headlineSmall),
          const SizedBox(height: 6),
          const Text(
            'Installs the open source KT firmware (casainho\'s, with bug fixes) on a KT controller with an '
            'STM8S105 chip, using the SWIM pin of an ST-Link V2 on the phone\'s USB port (OTG adapter).',
          ),
          const SizedBox(height: 12),
          step(
            1,
            'Wire 3.3V, SWIM, GND, RST to the KT programming header. Power the controller from the battery.',
            'Check',
            _check,
          ),
          const SizedBox(height: 8),
          step(2, 'Unlock the chip (erases the original firmware for good).', 'Unlock', _unlock),
          const SizedBox(height: 8),
          step(3, 'Settings: controller, battery, current ...', 'Settings', () => _open(const KtSettingsScreen())),
          const SizedBox(height: 8),
          step(4, 'Install the firmware with these settings.', 'Install', _install),
          const SizedBox(height: 8),
          step(5, 'Live data: what the controller sees while riding.', 'Live', () => _open(const KtLiveScreen())),
          if (_busy) const Padding(padding: EdgeInsets.only(top: 12), child: LinearProgressIndicator()),
          LogBox(_log, height: 260),
          const SizedBox(height: 8),
          Text('Current settings', style: theme.textTheme.titleMedium),
          Text(
            '$ctrl\n${s.i('cells')}S battery, motor current ${s.i('motor_current_a')} A, '
            'regen ${s.i('regen_current_a')} A, ${s.i('pas_magnets')} PAS magnets',
          ),
        ],
      ),
    );
  }
}

/// Keeps the screen on during long SWIM jobs (no-op where the platform channel is missing).
class UsbBridgeKeepAwake {
  static const _ch = MethodChannel('cz.ebics/usb');
  static Future<void> on() => _set(true);
  static Future<void> off() => _set(false);
  static Future<void> _set(bool v) async {
    try {
      await _ch.invokeMethod('keepScreenOn', v);
    } catch (_) {}
  }
}

class KtSettingsScreen extends StatefulWidget {
  const KtSettingsScreen({super.key});
  @override
  State<KtSettingsScreen> createState() => _KtSettingsScreenState();
}

class _KtSettingsScreenState extends State<KtSettingsScreen> {
  late KtSettings _s = AppScope.read(context).kt.copy();
  int _gen = 0;
  bool _busy = false;
  String _status = '';
  Tone _tone = Tone.muted;

  bool _check() {
    final p = _s.problems();
    if (p.isEmpty) return true;
    showDialog<void>(
      context: context,
      builder: (c) => AlertDialog(
        title: const Text('Please check'),
        content: Text(p.join('\n\n')),
        actions: [TextButton(onPressed: () => Navigator.pop(c), child: const Text('OK'))],
      ),
    );
    return false;
  }

  void _save() {
    if (!_check()) return;
    AppScope.read(context).saveKt(_s.copy());
    snack(context, 'Saved on the phone');
  }

  Future<void> _write() async {
    if (!_check()) return;
    final app = AppScope.read(context);
    app.saveKt(_s.copy());
    setState(() {
      _busy = true;
      _status = 'Writing the settings ...';
      _tone = Tone.busy;
    });
    try {
      await app.ktController.writeSettings(_s);
      if (mounted) {
        setState(() {
          _status = 'Settings written, controller restarted. (The firmware must already be installed.)';
          _tone = Tone.ok;
        });
      }
    } catch (e) {
      if (mounted) {
        setState(() {
          _status = 'Failed: $e';
          _tone = Tone.bad;
        });
      }
    } finally {
      if (mounted) setState(() => _busy = false);
    }
  }

  Future<void> _export() async {
    await Clipboard.setData(ClipboardData(text: _s.encode()));
    if (mounted) snack(context, 'Profile copied (same JSON as my_kunteng.json of the desktop app)');
  }

  Future<void> _import() async {
    final t = (await Clipboard.getData('text/plain'))?.text;
    if (!mounted) return;
    try {
      final s = KtSettings.decode(t ?? '');
      setState(() {
        _s = s;
        _gen++;
      });
      snack(context, 'Profile loaded - check it, then Save');
    } catch (e) {
      snack(context, 'Not a valid profile: $e');
    }
  }

  Widget _section(String section) => ListView(
    padding: const EdgeInsets.all(16),
    children: [
      if (section == 'expert') const StatusLine('Only change these if you know what they do.', tone: Tone.warn),
      for (final f in ktFields)
        if (f.section == section)
          KtFieldEditor(
            key: ValueKey('${f.key}/$_gen'),
            field: f,
            settings: _s,
            onChanged: (v) => setState(() => _s[f.key] = v),
          ),
    ],
  );

  @override
  Widget build(BuildContext context) {
    return DefaultTabController(
      length: 3,
      child: Scaffold(
        appBar: AppBar(
          title: const Text('KT settings'),
          actions: [
            PopupMenuButton<String>(
              onSelected: (v) => v == 'export' ? _export() : _import(),
              itemBuilder: (_) => const [
                PopupMenuItem(value: 'export', child: Text('Copy profile (JSON)')),
                PopupMenuItem(value: 'import', child: Text('Paste profile (JSON)')),
              ],
            ),
          ],
          bottom: const TabBar(
            tabs: [
              Tab(text: 'Basic'),
              Tab(text: 'Advanced'),
              Tab(text: 'Expert'),
            ],
          ),
        ),
        body: Column(
          children: [
            Expanded(child: TabBarView(children: [_section('basic'), _section('advanced'), _section('expert')])),
            if (_status.isNotEmpty)
              Padding(
                padding: const EdgeInsets.symmetric(horizontal: 16),
                child: StatusLine(_status, tone: _tone),
              ),
            if (_busy) const LinearProgressIndicator(),
            SafeArea(
              top: false,
              child: Padding(
                padding: const EdgeInsets.all(8),
                child: Row(
                  children: [
                    OutlinedButton(onPressed: _busy ? null : _save, child: const Text('Save')),
                    const SizedBox(width: 12),
                    Expanded(
                      child: FilledButton.icon(
                        onPressed: _busy ? null : _write,
                        icon: const Icon(Icons.upload),
                        label: const Text('Write to controller', maxLines: 1, overflow: TextOverflow.ellipsis),
                      ),
                    ),
                  ],
                ),
              ),
            ),
          ],
        ),
      ),
    );
  }
}

class KtFieldEditor extends StatefulWidget {
  final KtField field;
  final KtSettings settings;
  final ValueChanged<num> onChanged;
  const KtFieldEditor({super.key, required this.field, required this.settings, required this.onChanged});
  @override
  State<KtFieldEditor> createState() => _KtFieldEditorState();
}

class _KtFieldEditorState extends State<KtFieldEditor> {
  late final TextEditingController _tc = TextEditingController(text: '${widget.settings[widget.field.key]}');

  @override
  void dispose() {
    _tc.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    final f = widget.field;
    final s = widget.settings;
    final Widget editor = switch (f.kind) {
      KtKind.boolean => SwitchListTile(
        contentPadding: EdgeInsets.zero,
        title: Text(f.label),
        value: s.b(f.key),
        onChanged: (v) => widget.onChanged(v ? 1 : 0),
      ),
      KtKind.choice => DropdownButtonFormField<int>(
        initialValue: ktChoices[f.key]!.any((c) => c.$2 == s.i(f.key)) ? s.i(f.key) : ktChoices[f.key]!.first.$2,
        isExpanded: true,
        decoration: InputDecoration(labelText: f.label, border: const OutlineInputBorder()),
        items: [
          for (final (label, v) in ktChoices[f.key]!)
            DropdownMenuItem(
              value: v,
              child: Text(label, overflow: TextOverflow.ellipsis),
            ),
        ],
        onChanged: (v) {
          if (v != null) widget.onChanged(v);
        },
      ),
      _ => TextField(
        controller: _tc,
        keyboardType: TextInputType.numberWithOptions(decimal: f.kind == KtKind.decimal),
        decoration: InputDecoration(labelText: f.label, suffixText: f.unit, border: const OutlineInputBorder()),
        onChanged: (t) {
          final txt = t.trim().replaceAll(',', '.');
          final v = f.kind == KtKind.integer
              ? (int.tryParse(txt) ?? double.tryParse(txt)?.toInt())
              : double.tryParse(txt);
          if (v != null) widget.onChanged(v);
        },
      ),
    };
    return Padding(
      padding: const EdgeInsets.symmetric(vertical: 6),
      child: Row(
        children: [
          Expanded(child: editor),
          if (f.help.isNotEmpty)
            IconButton(
              icon: const Icon(Icons.help_outline),
              onPressed: () => showDialog<void>(
                context: context,
                builder: (c) => AlertDialog(
                  title: Text(f.label),
                  content: Text(f.help),
                  actions: [TextButton(onPressed: () => Navigator.pop(c), child: const Text('OK'))],
                ),
              ),
            ),
        ],
      ),
    );
  }
}

/// Live data of the running KT firmware over SWIM (no reset).
class KtLiveScreen extends StatefulWidget {
  const KtLiveScreen({super.key});
  @override
  State<KtLiveScreen> createState() => _KtLiveScreenState();
}

class _KtLiveScreenState extends State<KtLiveScreen> {
  Map<String, Object> _snap = {};
  String _status = 'Connecting ...';
  Tone _tone = Tone.busy;
  bool _running = true;

  @override
  void initState() {
    super.initState();
    _loop();
  }

  @override
  void dispose() {
    _running = false;
    super.dispose();
  }

  Future<void> _loop() async {
    final c = AppScope.read(context).ktController;
    await UsbBridgeKeepAwake.on();
    while (_running) {
      try {
        final s = await c.liveSnapshot();
        if (!mounted) break;
        setState(() {
          _snap = s;
          _status = 'Live';
          _tone = Tone.ok;
        });
        await Future<void>.delayed(const Duration(milliseconds: 400));
      } catch (e) {
        if (!mounted) break;
        setState(() {
          _status = 'No connection: $e';
          _tone = Tone.bad;
        });
        await Future<void>.delayed(const Duration(milliseconds: 1500));
      }
    }
    await c.stopLive();
    await UsbBridgeKeepAwake.off();
  }

  @override
  Widget build(BuildContext context) {
    final theme = Theme.of(context);
    return Scaffold(
      appBar: AppBar(title: const Text('KT live data')),
      body: ListView(
        padding: const EdgeInsets.all(16),
        children: [
          StatusLine(_status, tone: _tone),
          const Text('Read over the ST-Link while the controller runs. Needs the firmware installed by this app.'),
          const SizedBox(height: 8),
          for (final v in ktLiveVars)
            ListTile(
              dense: true,
              title: Text(v.label),
              trailing: Text(
                _snap.containsKey(v.name) ? ktDescribe(v, _snap[v.name]!) : '-',
                style: theme.textTheme.titleMedium,
              ),
            ),
        ],
      ),
    );
  }
}
