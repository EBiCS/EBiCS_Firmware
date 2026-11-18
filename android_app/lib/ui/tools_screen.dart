import 'package:material_ui/material_ui.dart';

import '../app_state.dart';
import '../firmware/settings_block.dart';
import 'common.dart';

class ToolsScreen extends StatefulWidget {
  const ToolsScreen({super.key});
  @override
  State<ToolsScreen> createState() => _ToolsScreenState();
}

class _ToolsScreenState extends State<ToolsScreen> {
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
      await f();
    } catch (e) {
      _add('ERROR: $e');
    } finally {
      if (mounted) setState(() => _busy = false);
    }
  }

  Future<void> _check() => _run('check connection', () async {
        final pr = await AppScope.read(context).controller.probe();
        _add(pr.describe());
      });

  Future<void> _lockState() => _run('chip lock', () async {
        final c = AppScope.read(context).controller;
        if (!c.connected) await c.probe();
        final locked = await c.readProtected();
        _add(locked ? 'chip is LOCKED (original firmware) - unlock it before installing EBiCS' : 'chip is unlocked');
      });

  Future<void> _unlock() async {
    final ok = await confirm(context, 'Unlock chip',
        'Unlocking erases the whole flash, including the original firmware.\nThis cannot be undone. Continue?',
        yes: 'Unlock');
    if (!ok || !mounted) return;
    await _run('unlock', () async {
      final c = AppScope.read(context).controller;
      if (!c.connected) await c.probe();
      await c.unlock(_add);
      await c.disconnect();
      _add('unlocked - switch the controller off and on again');
    });
  }

  Future<void> _identify() => _run('firmware on the controller', () async {
        final app = AppScope.read(context);
        final c = app.controller;
        if (!c.connected) await c.probe();
        final v = await c.identify();
        _add(v == null
            ? 'not one of this app\'s firmwares (original, desktop build or another version)'
            : '${v.displayLabel}, ${v.sensorless ? 'sensorless' : 'hall sensors'} (${await app.repo.version()})');
        final b = await c.readSettingsBlock();
        if (b == null) {
          _add('settings block: none (the firmware uses its built-in config.h values)');
        } else {
          _add('settings block: motor ${b['ph_current_max']} units, battery ${b['battery_current_max']} mA, '
              'cutoff ${b['battery_voltage_min']} mV, full ${b['battery_voltage_max']} mV, '
              '${b['speed_limit']} km/h, wheel ${b['wheel_circumference']} mm, gear ${b['gear_ratio']}, '
              'PAS ${b['pas_imp_per_turn']}, reverse ${b['reverse'] == -1 ? 'yes' : 'no'}, '
              'flags 0x${b['flags']!.toRadixString(16)}');
          final mine = SettingsBlock.decode(SettingsBlock.build(app.settings));
          final same = mine != null && mine.entries.every((e) => b[e.key] == e.value);
          _add(same ? 'same as the settings in this app' : 'DIFFERENT from the settings in this app');
        }
      });

  Future<void> _install() async {
    final app = AppScope.read(context);
    final ok = await confirm(context, 'Install firmware',
        'Install the firmware for "${app.settings.displayType.label}" with the current settings? '
        'The motor calibration is kept.',
        yes: 'Install');
    if (!ok || !mounted) return;
    await _run('install firmware', () async {
      final c = app.controller;
      final pr = await c.probe();
      if (!pr.targetFound) throw Exception(pr.describe());
      var last = '';
      final v = await c.install(app.settings, (step, p) {
        if (step != last) _add(step);
        last = step;
      });
      _add('installed ${v.name}, verified, controller restarted');
    });
  }

  Future<void> _restart() => _run('restart controller', () async {
        final c = AppScope.read(context).controller;
        if (!c.connected) await c.probe();
        await c.restart();
        _add('restarted');
      });

  @override
  Widget build(BuildContext context) {
    Widget b(String text, VoidCallback f) => OutlinedButton(onPressed: _busy ? null : f, child: Text(text));
    return Scaffold(
      appBar: AppBar(title: const Text('Tools')),
      body: ListView(padding: const EdgeInsets.all(16), children: [
        Wrap(spacing: 8, runSpacing: 8, children: [
          b('Check connection', _check),
          b('Check chip lock', _lockState),
          b('Unlock chip (erases flash)', _unlock),
          b('Firmware + settings on the controller', _identify),
          b('Install firmware', _install),
          b('Restart controller', _restart),
        ]),
        if (_busy) const Padding(padding: EdgeInsets.only(top: 12), child: LinearProgressIndicator()),
        LogBox(_log, height: 380),
      ]),
    );
  }
}
