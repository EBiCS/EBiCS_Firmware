import 'package:material_ui/material_ui.dart';

import '../app_state.dart';
import 'live_screen.dart';
import 'settings_screen.dart';
import 'tools_screen.dart';
import 'wizard_screen.dart';

class HomeScreen extends StatelessWidget {
  const HomeScreen({super.key});

  void _open(BuildContext context, Widget w) =>
      Navigator.of(context).push(MaterialPageRoute<void>(builder: (_) => w));

  @override
  Widget build(BuildContext context) {
    final app = AppScope.of(context);
    final s = app.settings;
    final theme = Theme.of(context);
    Widget tile(IconData icon, String title, String sub, Widget page) => Card(
          color: theme.colorScheme.surfaceContainer,
          child: ListTile(
            leading: Icon(icon, color: theme.colorScheme.primary),
            title: Text(title),
            subtitle: Text(sub),
            trailing: const Icon(Icons.chevron_right),
            onTap: () => _open(context, page),
          ),
        );
    return Scaffold(
      appBar: AppBar(title: const Text('EBiCS Setup'), actions: [
        IconButton(
          icon: const Icon(Icons.swap_horiz),
          tooltip: 'Change controller',
          onPressed: () => app.chooseKind(null),
        ),
      ]),
      body: ListView(
        padding: const EdgeInsets.all(16),
        children: [
          Text('Lishui controller setup', style: theme.textTheme.headlineSmall),
          const SizedBox(height: 6),
          const Text('Unlock the chip, install the open source EBiCS firmware, calibrate the motor and test everything '
              '- with an ST-Link V2 on the phone\'s USB port (OTG adapter).'),
          const SizedBox(height: 16),
          FilledButton.icon(
            onPressed: () => _open(context, const WizardScreen()),
            icon: const Icon(Icons.play_arrow),
            label: const Text('Start first setup'),
          ),
          const SizedBox(height: 16),
          tile(Icons.tune, 'Settings', 'Change power, speed limit, battery ... and write them to the controller',
              const SettingsScreen()),
          const SizedBox(height: 8),
          tile(Icons.speed, 'Live data', 'What the controller sees right now', const LiveScreen()),
          const SizedBox(height: 8),
          tile(Icons.build, 'Tools', 'Connection check, chip lock, install firmware, settings in the controller',
              const ToolsScreen()),
          const SizedBox(height: 20),
          Text('Current settings', style: theme.textTheme.titleMedium),
          const SizedBox(height: 4),
          Text('Display: ${s.displayType.label}\n'
              'Motor: ${s.sensorless ? 'sensorless (experimental)' : 'hall sensors'}, '
              'current ${s.d('phase_current_a')} A, battery ${s.d('battery_current_a')} A\n'
              'Speed limit ${s.i('speed_limit')} km/h, wheel ${s.wheelCircumference} mm'),
          const SizedBox(height: 16),
          FutureBuilder<String>(
            future: app.repo.version(),
            builder: (_, v) => Text('Firmware in this app: ${v.data ?? '...'}',
                style: theme.textTheme.bodySmall?.copyWith(color: theme.colorScheme.onSurfaceVariant)),
          ),
          Text('Keep the controller powered from the battery while connected.',
              style: theme.textTheme.bodySmall?.copyWith(color: theme.colorScheme.onSurfaceVariant)),
        ],
      ),
    );
  }
}
