import 'package:material_ui/material_ui.dart';
import 'package:shared_preferences/shared_preferences.dart';

import 'app_state.dart';
import 'firmware/firmware_repo.dart';
import 'ui/home_screen.dart';
import 'ui/kt_screens.dart';

Future<void> main() async {
  WidgetsFlutterBinding.ensureInitialized();
  final prefs = await SharedPreferences.getInstance();
  runApp(EbicsApp(state: AppState(prefs, FirmwareRepo())));
}

class EbicsApp extends StatelessWidget {
  final AppState state;
  const EbicsApp({super.key, required this.state});

  ThemeData _theme(Brightness b) {
    final scheme = ColorScheme.fromSeed(seedColor: const Color(0xFF1F6FEB), brightness: b);
    return ThemeData(
      colorScheme: scheme,
      useMaterial3: true,
      cardTheme: const CardThemeData(elevation: 0, margin: EdgeInsets.zero),
    );
  }

  @override
  Widget build(BuildContext context) {
    return AppScope(
      state: state,
      child: MaterialApp(
        title: 'EBiCS Setup',
        debugShowCheckedModeBanner: false,
        theme: _theme(Brightness.light),
        darkTheme: _theme(Brightness.dark),
        themeMode: ThemeMode.dark,
        home: const RootScreen(),
      ),
    );
  }
}

/// Asks which controller to set up at every start (last choice preselected), then shows its home screen.
class RootScreen extends StatelessWidget {
  const RootScreen({super.key});
  @override
  Widget build(BuildContext context) {
    final app = AppScope.of(context);
    return switch (app.kind) {
      'lishui' => const HomeScreen(),
      'kunteng' => const KtHomeScreen(),
      _ => const ChooserScreen(),
    };
  }
}

class ChooserScreen extends StatefulWidget {
  const ChooserScreen({super.key});
  @override
  State<ChooserScreen> createState() => _ChooserScreenState();
}

class _ChooserScreenState extends State<ChooserScreen> {
  late String _sel = AppScope.read(context).lastKind;

  @override
  Widget build(BuildContext context) {
    final theme = Theme.of(context);
    Widget option(String key, String title, String text) => Card(
      color: theme.colorScheme.surfaceContainer,
      child: RadioListTile<String>(
        value: key,
        title: Text(title, style: theme.textTheme.titleMedium),
        subtitle: Text(text),
      ),
    );
    return Scaffold(
      appBar: AppBar(title: const Text('EBiCS Setup')),
      body: RadioGroup<String>(
        groupValue: _sel,
        onChanged: (v) => setState(() => _sel = v ?? _sel),
        child: ListView(
          padding: const EdgeInsets.all(16),
          children: [
            Text('Which controller do you want to set up?', style: theme.textTheme.headlineSmall),
            const SizedBox(height: 6),
            const Text('Look at the label and the chip on the controller board.'),
            const SizedBox(height: 16),
            option(
              'lishui',
              'Lishui (EBiCS firmware)',
              'Lishui / LSW controllers with an STM32F103 chip. FOC, setup wizard, motor calibration, live data. '
                  'ST-Link SWD pins.',
            ),
            const SizedBox(height: 8),
            option(
              'kunteng',
              'Kunteng / KT (open source KT firmware)',
              'KT controllers (S06S, S12S, KT36/48 ...) with an STM8S105 chip. ST-Link SWIM pin. '
                  'Works with KT LCD3 / LCD5 displays.',
            ),
            const SizedBox(height: 20),
            FilledButton.icon(
              onPressed: () => AppScope.read(context).chooseKind(_sel),
              icon: const Icon(Icons.arrow_forward),
              label: const Text('Continue'),
            ),
          ],
        ),
      ),
    );
  }
}
