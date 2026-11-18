import 'dart:typed_data';

import 'package:ebics_setup/firmware/bike_settings.dart';
import 'package:ebics_setup/firmware/settings_block.dart';
import 'package:ebics_setup/stlink/stlink.dart';
import 'package:ebics_setup/stlink/stm32f1.dart';
import 'package:flutter_test/flutter_test.dart';

import 'fake_target.dart';

Future<(FakeTarget, Stm32F1)> connect() async {
  final t = FakeTarget();
  final st = StLink(t);
  await st.enterSwd();
  expect(await st.readIdCode(), 0x1BA01477);
  return (t, Stm32F1(st));
}

Uint8List eepromOf(FakeTarget t) => Uint8List.fromList(t.flash.sublist(0x7800));

void main() {
  test('ST-Link version and target voltage', () async {
    final t = FakeTarget();
    final st = StLink(t);
    await st.enterSwd();
    expect(st.version.toString(), 'V2J37S7');
    expect(await st.targetVoltage(), closeAsDouble(3.3));
  });

  test('chip info', () async {
    final (_, chip) = await connect();
    final i = await chip.info();
    expect(i.devId, 0x412);
    expect(i.flashKb, 32);
    expect(i.supported, isTrue);
    expect(i.readProtected, isFalse);
  });

  test('writeImage programs firmware + settings block and never touches pages 30-31', () async {
    final (t, chip) = await connect();
    final eeprom = eepromOf(t);
    // fake firmware 27.5 KB, then the settings block at 0x7700 like FirmwareRepo.image()
    const len = 0x7800;
    final img = Uint8List(len)..fillRange(0, len, 0xFF);
    for (var i = 0; i < 28100; i++) {
      img[i] = (i * 7 + 3) & 0xFF;
    }
    final block = SettingsBlock.build(BikeSettings.defaults());
    img.setRange(0x7700, 0x7800, block);
    final steps = <String>{};
    await chip.writeImage(img, progress: (s, _) => steps.add(s));
    expect(t.flash.sublist(0, len), img);
    expect(eepromOf(t), eeprom, reason: 'emulated EEPROM must be unchanged');
    expect(t.erasedPages.every((p) => p < 30), isTrue);
    expect(t.erasedPages.toSet(), Set.of(List.generate(30, (i) => i)));
    for (final (a, b) in t.programmedRanges) {
      expect(b <= 0x08007800, isTrue, reason: 'programmed 0x${a.toRadixString(16)}..0x${b.toRadixString(16)}');
    }
    expect(steps, containsAll(['Erasing', 'Writing', 'Verifying', 'Done']));
    expect(t.halted, isTrue);
    await chip.resetRun();
    expect(t.halted, isFalse);
  });

  test('patchPage rewrites only the settings block', () async {
    final (t, chip) = await connect();
    final img = Uint8List(0x7800)..fillRange(0, 0x7800, 0xFF);
    for (var i = 0; i < 0x7500; i++) {
      img[i] = i & 0xFF; // code reaching into page 29
    }
    await chip.writeImage(img);
    t.erasedPages.clear();
    final eeprom = eepromOf(t);
    final s = BikeSettings.defaults()..['speed_limit'] = 32;
    final block = SettingsBlock.build(s);
    await chip.patchPage(SettingsBlock.address, block);
    expect(t.erasedPages, [29]);
    expect(t.flash.sublist(0, 0x7700), img.sublist(0, 0x7700), reason: 'code in page 29 kept');
    expect(t.flash.sublist(0x7700, 0x7800), block);
    expect(eepromOf(t), eeprom);
    expect(SettingsBlock.decode(Uint8List.fromList(t.flash.sublist(0x7700, 0x7800)))!['speed_limit'], 32);
  });

  test('writes into the EEPROM pages are refused', () async {
    final (t, chip) = await connect();
    expect(() => Stm32F1.checkWritable(0x08007800, 2), throwsA(isA<StLinkException>()));
    expect(() => Stm32F1.checkWritable(0x08007700, 0x101), throwsA(isA<StLinkException>()));
    expect(() => Stm32F1.checkWritable(0x07FFFFFE, 4), throwsA(isA<StLinkException>()));
    await expectLater(chip.writeImage(Uint8List(0x7801)), throwsA(isA<StLinkException>()));
    await expectLater(chip.erasePage(0x08007800), throwsA(isA<StLinkException>()));
    await expectLater(chip.erasePage(0x08007C00), throwsA(isA<StLinkException>()));
    await expectLater(chip.patchPage(0x08007800, Uint8List(4)), throwsA(isA<StLinkException>()));
    expect(t.erasedPages, isEmpty);
  });

  test('read-out protection removal', () async {
    final (t, chip) = await connect();
    t.rdp = true;
    t.optionBytes.setAll(0, [0x00, 0xFF, 0x00, 0xFF]); // locked
    expect(await chip.readProtected(), isTrue);
    final log = <String>[];
    await chip.removeReadProtection(log: log.add);
    expect(t.optionBytes.sublist(0, 2), [0xA5, 0x5A]);
    expect(t.optionBytes.sublist(2, 16), [0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00]);
    t.powerCycle();
    expect(await chip.readProtected(), isFalse);
    expect(log.last, contains('Power cycle'));
  });

  test('loader is not started with zero half-words', () async {
    final (t, chip) = await connect();
    await chip.resetHalt();
    final before = t.loaderRuns;
    await chip.programPage(0x08000000, Uint8List(0));
    expect(t.loaderRuns, before);
  });

  test('memory reads are split at 1 KB boundaries', () async {
    final (_, chip) = await connect();
    final data = await chip.st.readMem(0x080003F0, 0x420);
    expect(data.length, 0x420);
  });
}

Matcher closeAsDouble(double v) => closeTo(v, 0.01);
