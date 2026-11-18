import 'package:flutter/services.dart';

/// Raw USB bulk transfers to the ST-Link (Kotlin side: MainActivity.kt).
abstract class UsbTransport {
  Future<void> bulkOut(Uint8List data, {int timeoutMs = 1000});
  Future<Uint8List> bulkIn(int length, {int timeoutMs = 1000});
}

class UsbException implements Exception {
  final String message;
  UsbException(this.message);
  @override
  String toString() => message;
}

class StLinkDeviceInfo {
  final String name;
  final int pid;
  final bool permission;
  StLinkDeviceInfo(this.name, this.pid, this.permission);
}

class UsbBridge implements UsbTransport {
  static const _ch = MethodChannel('cz.ebics/usb');

  /// The connected ST-Link, or null.
  static Future<StLinkDeviceInfo?> find() async {
    final m = await _ch.invokeMapMethod<String, dynamic>('find');
    if (m == null) return null;
    return StLinkDeviceInfo(m['name'] as String? ?? 'ST-Link', m['pid'] as int? ?? 0, m['permission'] == true);
  }

  static Future<bool> requestPermission() async =>
      await _ch.invokeMethod<bool>('requestPermission') ?? false;

  static Future<void> keepScreenOn(bool on) async {
    try {
      await _ch.invokeMethod('keepScreenOn', on);
    } on MissingPluginException {
      // tests / other platforms
    }
  }

  /// Opens the ST-Link. Returns the USB product id.
  Future<int> open() async {
    final m = await _ch.invokeMapMethod<String, dynamic>('open') ?? {};
    if (m['ok'] != true) throw UsbException(m['error'] as String? ?? 'could not open the ST-Link');
    return m['pid'] as int? ?? 0;
  }

  Future<void> close() => _ch.invokeMethod('close');

  @override
  Future<void> bulkOut(Uint8List data, {int timeoutMs = 1000}) async {
    final n = await _ch.invokeMethod<int>('bulkOut', {'data': data, 'timeout': timeoutMs}) ?? -1;
    if (n != data.length) throw UsbException('USB write failed - is the ST-Link still connected?');
  }

  @override
  Future<Uint8List> bulkIn(int length, {int timeoutMs = 1000}) async {
    final r = await _ch.invokeMethod<Uint8List>('bulkIn', {'length': length, 'timeout': timeoutMs});
    if (r == null) throw UsbException('USB read failed - is the ST-Link still connected?');
    return r;
  }
}
