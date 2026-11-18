import 'dart:async';

import 'package:material_ui/material_ui.dart';

import '../app_state.dart';
import '../controller/controller.dart';
import '../firmware/bike_settings.dart';
import '../firmware/settings_block.dart';
import '../usb/usb_bridge.dart';
import 'common.dart';

/// Values read from the running controller over the ST-Link (like the desktop Live data page).
class LiveScreen extends StatefulWidget {
  const LiveScreen({super.key});
  @override
  State<LiveScreen> createState() => _LiveScreenState();
}

class _LiveScreenState extends State<LiveScreen> {
  Map<String, int>? _snap;
  String _status = 'Connecting ...';
  Tone _tone = Tone.busy;
  Timer? _t;
  bool _stopped = false;
  bool _first = true;
  String _halls = '';
  Tone _hallTone = Tone.muted;

  @override
  void initState() {
    super.initState();
    UsbBridge.keepScreenOn(true);
    WidgetsBinding.instance.addPostFrameCallback((_) => _poll());
  }

  @override
  void dispose() {
    _stopped = true;
    _t?.cancel();
    UsbBridge.keepScreenOn(false);
    super.dispose();
  }

  Future<void> _poll() async {
    if (_stopped || !mounted) return;
    final c = AppScope.read(context).controller;
    try {
      if (!c.connected) {
        final pr = await c.probe();
        if (!pr.targetFound) throw Exception(pr.describe());
      }
      if (!c.hasSymbols) {
        if (await c.identify() == null) {
          throw Exception('The controller does not run firmware from this app (old firmware or a desktop build). '
              'Install the firmware from this app to see live data.');
        }
      }
      final snap = await c.snapshot(withHalls: _first);
      if (!mounted) return;
      if (snap == null) throw Exception('no answer');
      setState(() {
        _snap = snap;
        _status = 'connected  ${TimeOfDay.now().format(context)}';
        _tone = Tone.ok;
        if (snap['ui32_setup_magic'] != setupMagic) {
          _status = 'old firmware on the controller - install the new one for all values';
          _tone = Tone.warn;
        }
        if (_first) {
          final ang = hallAnglesDeg(snap);
          if (ang != null) {
            final ok = checkHallAngles(ang);
            _halls = 'Hall angles: ${ang.map((a) => '${a.round()}°').join(', ')}'
                '${ok ? '   (good, about 60° apart)' : '   (unusual spacing - run the motor calibration again)'}';
            _hallTone = ok ? Tone.ok : Tone.warn;
          }
        }
      });
      _first = false;
    } catch (e) {
      if (!mounted) return;
      setState(() {
        _status = '$e';
        _tone = Tone.bad;
      });
      await c.disconnect();
      _t = Timer(const Duration(seconds: 2), _poll);
      return;
    }
    _t = Timer(const Duration(milliseconds: 300), _poll);
  }

  List<(String, String, String)> _tiles(BikeSettings s, Map<String, int>? m) {
    String v(String k) => m?[k]?.toString() ?? '-';
    final calV = s.i('cal_v');
    final calI = SettingsBlock.cInt(s.s('cal_i'), 38);
    final thr = m?['ui16_throttle'] ?? 0, off = m?['ui16_throttle_offset'] ?? 0;
    final span = (s.i('throttle_max') - off) < 1 ? 1 : s.i('throttle_max') - off;
    final st = s.i('speed_temp_input');
    final src = m?['ui8_settings_source'];
    return [
      ('Battery', m == null ? '-' : ((m['MS.Voltage'] ?? 0) * calV / 1000).toStringAsFixed(1), 'V'),
      ('Speed', m == null ? '-' : speedKmh(m, s).toStringAsFixed(1), 'km/h'),
      ('Assist level', v('MS.assist_level'), ''),
      ('Throttle', m == null ? '-' : '${((thr - off) * 100 ~/ span).clamp(0, 100)}  ($thr)', '%'),
      ('Brake', m == null ? '-' : ((m['brake_flag'] ?? 0) != 0 ? 'PRESSED' : 'off'), ''),
      ('Motor current', m == null ? '-' : ((m['MS.i_q'] ?? 0) * calI / 1000).toStringAsFixed(1), 'A'),
      ('Battery current', m == null ? '-' : ((m['MS.Battery_Current'] ?? 0) / 1000).toStringAsFixed(1), 'A'),
      ('Pedal pulses', v('ui32_pas_events'), ''),
      ('Hall events', v('ui32_hall_events'), ''),
      ('Controller temp', v('MS.int_Temperature'), '°C'),
      ('Motor calibrated', s.sensorless ? 'not needed' : (m == null ? '-' : ((m['ui8_hall_calibrated'] ?? 0) != 0 ? 'yes' : 'NO')), ''),
      ('KV', v('ui32_KV'), ''),
      ('Motor temp', st != 0 && s.b('temp_on_wire') ? v('MS.Temperature') : 'no sensor', '°C'),
      ('Speed/temp wire (raw)', st != 0 ? v('adc$st') : 'not used', 'ADC'),
      ('Sensorless observer', s.sensorless ? ((m?['ui8_sensorless_running'] ?? 0) != 0 ? 'running' : 'stopped') : 'off (hall)', ''),
      ('Last reset', m == null ? '-' : ((m['ui8_watchdog_reset'] ?? 0) != 0 ? 'WATCHDOG' : 'normal'), ''),
      ('Settings used', switch (src) { 1 => 'from app', 0 => 'built in', 2 => 'REJECTED', _ => '-' }, ''),
    ];
  }

  @override
  Widget build(BuildContext context) {
    final s = AppScope.of(context).settings;
    final theme = Theme.of(context);
    final tiles = _tiles(s, _snap);
    return Scaffold(
      appBar: AppBar(title: const Text('Live data')),
      body: ListView(
        padding: const EdgeInsets.all(12),
        children: [
          StatusLine(_status, tone: _tone),
          GridView.count(
            crossAxisCount: MediaQuery.of(context).size.width > 600 ? 4 : 2,
            shrinkWrap: true,
            physics: const NeverScrollableScrollPhysics(),
            mainAxisSpacing: 8,
            crossAxisSpacing: 8,
            childAspectRatio: 1.8,
            children: [
              for (final (label, value, unit) in tiles)
                Card(
                  color: theme.colorScheme.surfaceContainer,
                  child: Padding(
                    padding: const EdgeInsets.all(10),
                    child: Column(crossAxisAlignment: CrossAxisAlignment.start, children: [
                      Text(label,
                          maxLines: 1,
                          overflow: TextOverflow.ellipsis,
                          style: theme.textTheme.labelMedium?.copyWith(color: theme.colorScheme.onSurfaceVariant)),
                      const Spacer(),
                      FittedBox(
                        fit: BoxFit.scaleDown,
                        alignment: Alignment.centerLeft,
                        child: Text(value, style: theme.textTheme.headlineSmall?.copyWith(fontWeight: FontWeight.w600)),
                      ),
                      Text(unit, style: theme.textTheme.labelSmall),
                    ]),
                  ),
                ),
            ],
          ),
          StatusLine(_halls, tone: _hallTone),
        ],
      ),
    );
  }
}
