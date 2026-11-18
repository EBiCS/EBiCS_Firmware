import 'package:flutter/widgets.dart';
import 'package:shared_preferences/shared_preferences.dart';

import 'controller/controller.dart';
import 'firmware/bike_settings.dart';
import 'firmware/firmware_repo.dart';

/// Settings (saved on the phone as the same JSON profile as the desktop app's my_bike.json),
/// the firmware assets and the controller session.
class AppState extends ChangeNotifier {
  static const _key = 'bike_settings';
  final SharedPreferences prefs;
  final FirmwareRepo repo;
  final Controller controller;
  BikeSettings settings;

  AppState(this.prefs, this.repo)
      : controller = Controller(repo),
        settings = _load(prefs);

  static BikeSettings _load(SharedPreferences p) {
    final t = p.getString(_key);
    if (t == null) return BikeSettings.defaults();
    try {
      return BikeSettings.decode(t);
    } catch (_) {
      return BikeSettings.defaults();
    }
  }

  void save([BikeSettings? s]) {
    if (s != null) settings = s;
    prefs.setString(_key, settings.encode());
    notifyListeners();
  }

  void reset() => save(BikeSettings.defaults());
}

class AppScope extends InheritedNotifier<AppState> {
  const AppScope({super.key, required AppState state, required super.child}) : super(notifier: state);

  static AppState of(BuildContext context) => context.dependOnInheritedWidgetOfExactType<AppScope>()!.notifier!;
  static AppState read(BuildContext context) =>
      (context.getElementForInheritedWidgetOfExactType<AppScope>()!.widget as AppScope).notifier!;
}
