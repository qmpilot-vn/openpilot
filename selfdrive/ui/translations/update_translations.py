#!/usr/bin/env python3
import ast
import json
from itertools import chain
import os
from openpilot.common.basedir import BASEDIR
from openpilot.system.ui.lib.multilang import SYSTEM_UI_DIR, UI_DIR, TRANSLATIONS_DIR, multilang
from openpilot.selfdrive.ui.translations.potools import POEntry, extract_strings, generate_pot, merge_po, init_po

LANGUAGES_FILE = os.path.join(str(TRANSLATIONS_DIR), "languages.json")
POT_FILE = os.path.join(str(TRANSLATIONS_DIR), "app.pot")

# Text that reaches the UI from other processes, translated at render time
OFFROAD_ALERTS_FILE = os.path.join("selfdrive", "selfdrived", "alerts_offroad.json")
EVENTS_FILES = [
  os.path.join("selfdrive", "selfdrived", "events.py"),
  os.path.join("sunnypilot", "selfdrive", "selfdrived", "events.py"),
]
UPDATER_STATES = ["checking...", "downloading...", "finalizing update..."]
# runtime variants of alerts whose f-strings don't reduce to a plain "{}" template
EXTRA_ALERT_STRINGS = [
  "{} seconds remaining. Press again to save early.",
  "{} second remaining. Press again to save early.",
  "Driving Personality: Aggressive",
  "Driving Personality: Standard",
  "Driving Personality: Relaxed",
]


def _alert_text(node: ast.expr) -> str | None:
  # f-string values become "{}" placeholders, matched by tr_dynamic
  if isinstance(node, ast.Constant) and isinstance(node.value, str):
    return node.value
  if isinstance(node, ast.JoinedStr):
    return "".join(v.value if isinstance(v, ast.Constant) else "{}" for v in node.values)
  return None


def extract_external_strings() -> list[POEntry]:
  seen: dict[str, POEntry] = {}

  def add(msgid: str, ref: str):
    msgid = msgid.strip()
    if not msgid or not any(c.isalpha() for c in msgid):
      return
    if msgid in seen:
      seen[msgid].source_refs.append(ref)
    else:
      seen[msgid] = POEntry(msgid=msgid, source_refs=[ref])

  with open(os.path.join(BASEDIR, OFFROAD_ALERTS_FILE), encoding="utf-8") as f:
    for key, alert in json.load(f).items():
      if alert.get("text"):
        add(alert["text"], f"{OFFROAD_ALERTS_FILE}:{key}")

  for path in EVENTS_FILES:
    with open(os.path.join(BASEDIR, path), encoding="utf-8") as f:
      tree = ast.parse(f.read(), filename=path)
    for node in ast.walk(tree):
      if not isinstance(node, ast.Call):
        continue
      func = node.func
      name = func.id if isinstance(func, ast.Name) else func.attr if isinstance(func, ast.Attribute) else ""
      if not name.endswith("Alert"):
        continue
      args = list(node.args[:2]) + [kw.value for kw in node.keywords if kw.arg in ("alert_text_1", "alert_text_2")]
      for arg in args:
        if (text := _alert_text(arg)) is not None:
          add(text, f"{path}:{node.lineno}")

  for state in UPDATER_STATES:
    add(state, "system/updated/updated.py")
  for text in EXTRA_ALERT_STRINGS:
    add(text, EVENTS_FILES[0])

  return list(seen.values())


def update_translations():
  files = []
  for root, _, filenames in chain(os.walk(SYSTEM_UI_DIR),
                                  os.walk(os.path.join(str(UI_DIR), "widgets")),
                                  os.walk(os.path.join(str(UI_DIR), "layouts")),
                                  os.walk(os.path.join(str(UI_DIR), "onroad")),
                                  os.walk(os.path.join(str(UI_DIR), "mici")),
                                  os.walk(os.path.join(str(UI_DIR), "sunnypilot"))):
    for filename in filenames:
      if filename.endswith(".py") and "/tests" not in root:
        files.append(os.path.relpath(os.path.join(root, filename), BASEDIR))

  # Extract translatable strings and generate .pot template
  entries = extract_strings(files, BASEDIR)
  known = {e.msgid for e in entries}
  entries += [e for e in extract_external_strings() if e.msgid not in known]
  generate_pot(entries, POT_FILE)

  # Generate/update translation files for each language
  for name in multilang.languages.values():
    po_file = os.path.join(str(TRANSLATIONS_DIR), f"app_{name}.po")
    if os.path.exists(po_file):
      merge_po(po_file, POT_FILE)
    else:
      init_po(POT_FILE, po_file, name)


if __name__ == "__main__":
  update_translations()
