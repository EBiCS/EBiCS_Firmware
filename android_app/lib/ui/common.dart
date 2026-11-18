import 'package:material_ui/material_ui.dart';

import '../firmware/bike_settings.dart';

const okColor = Color(0xFF3FB950);
const badColor = Color(0xFFF85149);
const warnColor = Color(0xFFD29922);

enum Tone { muted, busy, ok, bad, warn }

Color toneColor(BuildContext context, Tone t) => switch (t) {
      Tone.ok => okColor,
      Tone.bad => badColor,
      Tone.warn => warnColor,
      Tone.busy => Theme.of(context).colorScheme.primary,
      Tone.muted => Theme.of(context).colorScheme.onSurfaceVariant,
    };

class StatusLine extends StatelessWidget {
  final String text;
  final Tone tone;
  const StatusLine(this.text, {super.key, this.tone = Tone.muted});

  @override
  Widget build(BuildContext context) {
    if (text.isEmpty) return const SizedBox.shrink();
    final icon = switch (tone) {
      Tone.ok => Icons.check_circle,
      Tone.bad => Icons.error,
      Tone.warn => Icons.warning_amber,
      Tone.busy => Icons.hourglass_top,
      Tone.muted => Icons.info_outline,
    };
    final c = toneColor(context, tone);
    return Padding(
      padding: const EdgeInsets.symmetric(vertical: 8),
      child: Row(crossAxisAlignment: CrossAxisAlignment.start, children: [
        Icon(icon, color: c, size: 22),
        const SizedBox(width: 10),
        Expanded(child: Text(text, style: Theme.of(context).textTheme.titleMedium?.copyWith(color: c))),
      ]),
    );
  }
}

class Bullets extends StatelessWidget {
  final List<String> items;
  const Bullets(this.items, {super.key});
  @override
  Widget build(BuildContext context) => Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          for (final t in items)
            Padding(
              padding: const EdgeInsets.only(left: 6, bottom: 6),
              child: Row(crossAxisAlignment: CrossAxisAlignment.start, children: [
                const Text('•  '),
                Expanded(child: Text(t)),
              ]),
            ),
        ],
      );
}

/// Scrolling monospace log.
class LogBox extends StatefulWidget {
  final List<String> lines;
  final double height;
  const LogBox(this.lines, {super.key, this.height = 160});
  @override
  State<LogBox> createState() => _LogBoxState();
}

class _LogBoxState extends State<LogBox> {
  final _sc = ScrollController();
  @override
  void didUpdateWidget(LogBox old) {
    super.didUpdateWidget(old);
    WidgetsBinding.instance.addPostFrameCallback((_) {
      if (_sc.hasClients) _sc.jumpTo(_sc.position.maxScrollExtent);
    });
  }

  @override
  void dispose() {
    _sc.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    if (widget.lines.isEmpty) return const SizedBox.shrink();
    return Container(
      height: widget.height,
      margin: const EdgeInsets.symmetric(vertical: 8),
      padding: const EdgeInsets.all(8),
      decoration: BoxDecoration(color: const Color(0xFF0D1117), borderRadius: BorderRadius.circular(8)),
      child: ListView(
        controller: _sc,
        children: [
          for (final l in widget.lines)
            Text(l, style: const TextStyle(fontFamily: 'monospace', fontSize: 12, color: Color(0xFFD6DEEB))),
        ],
      ),
    );
  }
}

/// Editor for one settings field. Calls [onChanged] with the new raw value.
class FieldEditor extends StatefulWidget {
  final Field field;
  final BikeSettings settings;
  final ValueChanged<Object?> onChanged;
  const FieldEditor({super.key, required this.field, required this.settings, required this.onChanged});
  @override
  State<FieldEditor> createState() => _FieldEditorState();
}

class _FieldEditorState extends State<FieldEditor> {
  late final TextEditingController _tc = TextEditingController(text: _text());

  String _text() {
    final v = widget.settings[widget.field.key];
    if (v is double && v == v.roundToDouble() && widget.field.kind == FieldKind.decimal) return v.toStringAsFixed(1);
    return '${v ?? ''}';
  }

  @override
  void dispose() {
    _tc.dispose();
    super.dispose();
  }

  void _help() {
    final f = widget.field;
    if (f.help.isEmpty) return;
    showDialog<void>(
        context: context,
        builder: (c) => AlertDialog(
              title: Text(f.label.trim()),
              content: Text(f.help),
              actions: [TextButton(onPressed: () => Navigator.pop(c), child: const Text('OK'))],
            ));
  }

  Widget _dropdown<T>(List<(String, T)> items, T current) {
    final values = items.map((e) => e.$2).toList();
    return DropdownButtonFormField<T>(
      initialValue: values.contains(current) ? current : values.first,
      isExpanded: true,
      decoration: InputDecoration(labelText: widget.field.label.trim(), border: const OutlineInputBorder()),
      items: [for (final (label, v) in items) DropdownMenuItem(value: v, child: Text(label, overflow: TextOverflow.ellipsis))],
      onChanged: (v) => widget.onChanged(v),
    );
  }

  @override
  Widget build(BuildContext context) {
    final f = widget.field;
    final s = widget.settings;
    final Widget editor = switch (f.kind) {
      FieldKind.boolean => SwitchListTile(
          contentPadding: EdgeInsets.zero,
          title: Text(f.label.trim()),
          value: s.b(f.key),
          onChanged: (v) => widget.onChanged(v),
        ),
      FieldKind.display => _dropdown([for (final d in displays) (d.label, d.key)], s.display),
      FieldKind.battery => _dropdown(batteries, s.i('battery')),
      FieldKind.wheel => _dropdown([for (final (n, _) in wheels) (n, n)], s.s('wheel')),
      FieldKind.speedTemp => _dropdown(speedTempInputs, s.i('speed_temp_input')),
      _ => TextField(
          controller: _tc,
          keyboardType: f.kind == FieldKind.text
              ? TextInputType.text
              : TextInputType.numberWithOptions(decimal: f.kind == FieldKind.decimal, signed: true),
          decoration: InputDecoration(
            labelText: f.label.trim(),
            suffixText: f.unit,
            border: const OutlineInputBorder(),
          ),
          onChanged: (t) {
            final txt = t.trim().replaceAll(',', '.');
            if (f.kind == FieldKind.integer) {
              final v = int.tryParse(txt) ?? double.tryParse(txt)?.toInt();
              if (v != null) widget.onChanged(v);
            } else if (f.kind == FieldKind.decimal) {
              final v = double.tryParse(txt);
              if (v != null) widget.onChanged(v);
            } else {
              widget.onChanged(t.trim());
            }
          },
        ),
    };
    return Padding(
      padding: const EdgeInsets.symmetric(vertical: 6),
      child: Row(children: [
        Expanded(child: editor),
        if (f.help.isNotEmpty)
          IconButton(icon: const Icon(Icons.help_outline), tooltip: 'Help', onPressed: _help),
      ]),
    );
  }
}

Future<bool> confirm(BuildContext context, String title, String text, {String yes = 'Continue'}) async {
  final r = await showDialog<bool>(
    context: context,
    builder: (c) => AlertDialog(
      title: Text(title),
      content: Text(text),
      actions: [
        TextButton(onPressed: () => Navigator.pop(c, false), child: const Text('Cancel')),
        FilledButton(onPressed: () => Navigator.pop(c, true), child: Text(yes)),
      ],
    ),
  );
  return r == true;
}

void snack(BuildContext context, String msg) {
  ScaffoldMessenger.of(context).showSnackBar(SnackBar(content: Text(msg)));
}
