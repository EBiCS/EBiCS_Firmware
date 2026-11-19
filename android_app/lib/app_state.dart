import 'package:flutter/widgets.dart';
import 'package:shared_preferences/shared_preferences.dart';

import 'controller/controller.dart';
import 'firmware/bike_settings.dart';
import 'firmware/firmware_repo.dart';
import 'kunteng/kt_controller.dart';
import 'kunteng/kt_settings.dart';

/// Settings (saved on the phone as the same JSON profile as the desktop app's my_bike.json),
/// the firmware assets and the controller session.
class AppState extends ChangeNotifier {
  static const _key = 'bike_settings';
  static const _ktKey = 'kt_settings';
  static const _kindKey = 'controller_kind';
  final SharedPreferences prefs;
  final FirmwareRepo repo;
  final Controller controller;
  final KtController ktController;
  BikeSettings settings;
  KtSettings kt;

  /// Controller chosen on the start screen this session: 'lishui', 'kunteng' or null (show the chooser).
  String? kind;

  AppState(this.prefs, this.repo, {KtController? kt})
      : controller = Controller(repo),
        ktController = kt ?? KtController(),
        settings = _load(prefs),
        kt = _loadKt(prefs);

  static KtSettings _loadKt(SharedPreferences p) {
    final t = p.getString(_ktKey);
    if (t == null) return KtSettings.defaults();
    try {
      return KtSettings.decode(t);
    } catch (_) {
      return KtSettings.defaults();
    }
  }

  /// Last choice, preselected on the start screen.
  String get lastKind => prefs.getString(_kindKey) ?? 'lishui';

  void chooseKind(String? k) {
    kind = k;
    if (k != null) prefs.setString(_kindKey, k);
    notifyListeners();
  }

  void saveKt([KtSettings? s]) {
    if (s != null) kt = s;
    prefs.setString(_ktKey, kt.encode());
    notifyListeners();
  }

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
