import 'package:flutter/services.dart';
import 'package:material_ui/material_ui.dart';

import '../app_state.dart';
import '../firmware/bike_settings.dart';
import 'common.dart';

/// All settings the phone can change (the runtime settings block). Compile-time options
/// (control loop gains, PLL, ...) are fixed in the prebuilt firmware: change them with the desktop app.
class SettingsScreen extends StatefulWidget {
  const SettingsScreen({super.key});
  @override
  State<SettingsScreen> createState() => _SettingsScreenState();
}

class _SettingsScreenState extends State<SettingsScreen> {
  late BikeSettings _s = AppScope.read(context).settings.copy();
  int _gen = 0; // rebuild editors after load/reset
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
    AppScope.read(context).save(_s.copy());
    snack(context, 'Saved on the phone');
  }

  /// Writes the settings block only, or installs the firmware if the display or motor type changed.
  void _safe(VoidCallback f) {
    if (mounted) setState(f);
  }

  Future<void> _write() async {
    if (!_check()) return;
    final app = AppScope.read(context);
    app.save(_s.copy());
    final c = app.controller;
    _safe(() {
      _busy = true;
      _status = 'Connecting ...';
      _tone = Tone.busy;
    });
    try {
      final pr = await c.probe();
      if (!pr.targetFound) throw Exception(pr.describe());
      final v = await c.identify();
      final want = await app.repo.variantFor(_s);
      if (v != null && want != null && v.name == want.name) {
        _safe(() => _status = 'Writing the settings ...');
        await c.updateSettings(_s);
        _safe(() {
          _status = 'Settings written, controller restarted.';
          _tone = Tone.ok;
        });
      } else {
        if (!mounted) return;
        final go = await confirm(
            context,
            'Install firmware',
            v == null
                ? 'The controller does not run firmware from this app. Install the firmware with these settings? '
                    '(The motor calibration is kept.)'
                : 'Display or motor type changed: the firmware must be installed again. Continue? '
                    '(The motor calibration is kept.)',
            yes: 'Install');
        if (!go) {
          _safe(() {
            _status = 'Nothing written.';
            _tone = Tone.muted;
          });
          return;
        }
        await c.install(_s, (step, p) {
          if (mounted) setState(() => _status = '$step ... ${(p * 100).round()} %');
        });
        _safe(() {
          _status = 'Firmware installed with these settings.';
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
    if (mounted) snack(context, 'Profile copied to the clipboard (JSON, same as my_bike.json of the desktop app)');
  }

  Future<void> _import() async {
    final t = (await Clipboard.getData('text/plain'))?.text;
    if (!mounted) return;
    if (t == null || t.trim().isEmpty) {
      snack(context, 'Clipboard is empty: copy a profile (my_bike.json content) first');
      return;
    }
    try {
      final s = BikeSettings.decode(t);
      setState(() {
        _s = s;
        _gen++;
      });
      snack(context, 'Profile loaded - check it, then Save');
    } catch (e) {
      snack(context, 'Not a valid profile: $e');
    }
  }

  Future<void> _reset() async {
    if (await confirm(context, 'Reset', 'Reset all settings to the defaults?', yes: 'Reset')) {
      setState(() {
        _s = BikeSettings.defaults();
        _gen++;
      });
    }
  }

  bool _visible(Field f) {
    if (!f.runtime) return false;
    switch (f.key) {
      case 'wheel_mm':
        return _s.s('wheel') == 'custom';
      case 'speed_on_wire' || 'temp_on_wire' || 'speed_temp_threshold':
        return _s.i('speed_temp_input') != 0;
      case 'ntc_pullup_ohm':
        return _s.i('speed_temp_input') != 0 && _s.b('temp_on_wire');
      case 'pulses_per_rev':
        return _s.b('speed_sensor_external');
      case 'frac_high' || 'frac_low':
        return _s.b('dirdet');
      case 'ts_coef':
        return _s.b('torque_sensor');
    }
    return true;
  }

  Widget _section(String section) => ListView(
        padding: const EdgeInsets.all(16),
        children: [
          if (section == 'expert') ...[
            const StatusLine(
                'Only change these if you know what they do. Wrong values can damage the motor or controller.',
                tone: Tone.warn),
            const Text('The control loop settings (gains, PLL, calibration) are fixed in the prebuilt firmware. '
                'Change them with the desktop app (setup_app) if needed.'),
            const SizedBox(height: 8),
          ],
          for (final f in fields)
            if (f.section == section && _visible(f))
              FieldEditor(
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
          title: const Text('Settings'),
          actions: [
            PopupMenuButton<String>(
              onSelected: (v) => switch (v) {
                'export' => _export(),
                'import' => _import(),
                _ => _reset(),
              },
              itemBuilder: (_) => const [
                PopupMenuItem(value: 'export', child: Text('Copy profile (JSON)')),
                PopupMenuItem(value: 'import', child: Text('Paste profile (JSON)')),
                PopupMenuItem(value: 'reset', child: Text('Reset to defaults')),
              ],
            ),
          ],
          bottom: const TabBar(tabs: [Tab(text: 'Basic'), Tab(text: 'Advanced'), Tab(text: 'Expert')]),
        ),
        body: Column(children: [
          Expanded(child: TabBarView(children: [_section('basic'), _section('advanced'), _section('expert')])),
          if (_status.isNotEmpty)
            Padding(padding: const EdgeInsets.symmetric(horizontal: 16), child: StatusLine(_status, tone: _tone)),
          if (_busy) const LinearProgressIndicator(),
          SafeArea(
            top: false,
            child: Padding(
              padding: const EdgeInsets.all(8),
              child: Row(children: [
                OutlinedButton(onPressed: _busy ? null : _save, child: const Text('Save')),
                const SizedBox(width: 12),
                Expanded(
                  child: FilledButton.icon(
                    onPressed: _busy ? null : _write,
                    icon: const Icon(Icons.upload),
                    label: const Text('Write to controller', maxLines: 1, overflow: TextOverflow.ellipsis),
                  ),
                ),
              ]),
            ),
          ),
        ]),
      ),
    );
  }
}
