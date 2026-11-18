# EBiCS Setup for Android

Flutter app that installs, configures and calibrates EBiCS on a Lishui controller (STM32F103) from an Android
phone, through an ST-Link V2 on USB OTG. User guide: see the main [README](../README.md#android-app-setup-from-a-phone).

## Build

Needs Flutter (stable, 3.50 tested), JDK 17, the Android SDK and the ARM GCC toolchain for the firmware.

```bash
# 1. firmware variants + symbol maps + test vectors into android_app/assets/firmware/
python setup_app/build_variants.py
# 2. the app
cd android_app
flutter pub get
flutter analyze
flutter test
flutter build apk --release
```

The APK is `build/app/outputs/flutter-apk/app-release.apk`, signed with the debug key (fine for installing
directly, not for Google Play). `assets/firmware/` is generated, not in git.

## Where things are

| What | File |
|---|---|
| USB bulk transfers (Android USB host, permission, endpoints) | `android/app/src/main/kotlin/cz/ebics/ebics_setup/MainActivity.kt`, `lib/usb/usb_bridge.dart` |
| ST-Link V2 protocol (SWD, memory, core registers) | `lib/stlink/stlink.dart` |
| STM32F103: halt/reset, flash erase/program/verify with a RAM loader, RDP unlock | `lib/stlink/stm32f1.dart` |
| Settings (same keys as `setup_app/my_bike.json`), settings block + CRC | `lib/firmware/bike_settings.dart`, `lib/firmware/settings_block.dart` |
| Firmware assets, flash image | `lib/firmware/firmware_repo.dart` |
| Session, install, settings update, live data, calibration mailbox | `lib/controller/controller.dart` |
| Wizard and screens | `lib/ui/` |
| Simulated ST-Link + STM32F103 for the tests | `test/fake_target.dart` |

## How flashing works

1. SWD connect without reset (hot plug), read the chip id and the read-out protection (`FLASH_OBR`).
2. Install: build the image `0x08000000`-`0x080077FF` (firmware, `0xFF`, 256 byte settings block at
   `0x08007700`), reset and halt the core (watchdog frozen), unlock the flash controller, erase pages 0-29,
   program each page with a 32 byte loader in RAM (half-word writes, works on every ST-Link V2 firmware),
   read everything back and compare, reset and run.
3. Settings only: read page 29, replace the last 256 bytes, erase + program page 29, verify, restart.
4. Unlock: erase the option bytes (the chip mass-erases itself), write RDP = `0xA5` and no write protection,
   then the controller must be switched off and on.

Pages 30-31 (`0x08007800`-`0x08007FFF`) hold the emulated EEPROM with the hall angles and KV: every erase and
write goes through `Stm32F1.checkWritable`, which refuses anything at or above `0x08007800`. The app never uses
mass erase.

Live data and calibration read/write the firmware's variables over SWD while it runs. The addresses come from the
symbol map of the installed variant; the app recognises the installed variant by reading the flash back.
