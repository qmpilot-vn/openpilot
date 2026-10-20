import ast
import json
import re
from pathlib import Path

import pytest

from openpilot.common.basedir import BASEDIR


KEYS = ("LaneCentering", "LaneCenterOffset", "LaneCenteringE2EAuthority")


def _read(relpath: str) -> str:
  return (Path(BASEDIR) / relpath).read_text()


def test_params_keys_declare_lane_centering():
  text = _read("common/params_keys.h")
  assert '{"LaneCentering", {PERSISTENT | BACKUP, BOOL, "0"}}' in text
  assert '{"LaneCenterOffset", {PERSISTENT | BACKUP, FLOAT, "0.0"}}' in text
  assert '{"LaneCenteringE2EAuthority", {PERSISTENT | BACKUP, FLOAT, "1.0"}}' in text


def test_controlsd_applies_lane_centering_before_vinfast_bias():
  text = _read("selfdrive/controls/controlsd.py")
  apply_at = text.index("self.lane_centering.update(")
  bias_at = text.index("not self.lane_centering_enabled")
  assert apply_at < bias_at
  assert "LaneCenteringController" in text
  assert "self.lane_centering.reset()" in text


def test_controlsd_ext_reads_lane_centering_params():
  text = _read("sunnypilot/selfdrive/controls/controlsd_ext.py")
  for key in KEYS:
    assert f'"{key}"' in text
  assert "_update_lane_centering_params" in text
  assert "UnknownKeyName" in text


def test_steering_ui_exposes_lane_centering_controls():
  tree = ast.parse(_read("selfdrive/ui/sunnypilot/layouts/settings/steering.py"))
  found = set()
  for keywords in (
    node.keywords
    for node in ast.walk(tree)
    if isinstance(node, ast.Call)
  ):
    values = {kw.arg: ast.literal_eval(kw.value) for kw in keywords if kw.arg == "param" and isinstance(kw.value, ast.Constant)}
    found.update(values.values())
  assert set(KEYS) <= found


def test_sunnylink_settings_include_lane_centering():
  yaml_text = _read("sunnypilot/sunnylink/settings_ui_src/pages/steering.yaml")
  for key in KEYS:
    assert f"key: {key}" in yaml_text

  schema = json.loads(_read("sunnypilot/sunnylink/settings_ui.json"))
  steering = next(panel for panel in schema["panels"] if panel["id"] == "steering")
  section = next(sec for sec in steering["sections"] if sec["id"] == "lane_centering")
  item = section["items"][0]
  assert item["key"] == "LaneCentering"
  sub_keys = {sub["key"] for sub in item["sub_items"]}
  assert sub_keys == {"LaneCenterOffset", "LaneCenteringE2EAuthority"}


def test_shipped_params_so_is_aarch64():
  so = Path(BASEDIR) / "common/params_pyx.so"
  data = so.read_bytes()
  assert data[0:4] == b"\x7fELF"
  machine = int.from_bytes(data[18:20], "little")
  assert machine == 183  # EM_AARCH64
  strings = re.findall(rb"Lane[A-Za-z0-9]+", data)
  decoded = {s.decode() for s in strings}
  assert "LaneTurnDesire" in decoded


def _can_import_params() -> bool:
  try:
    from openpilot.common.params import Params, UnknownKeyName  # noqa: F401
    return True
  except ImportError:
    return False


class _FakeParams:
  def __init__(self, values=None, missing=False, unknown_exc=None):
    self.values = values or {}
    self.missing = missing
    self.unknown_exc = unknown_exc

  def get_bool(self, key):
    if self.missing:
      raise self.unknown_exc(key)
    return bool(self.values.get(key, False))

  def get(self, key, return_default=False):
    if self.missing:
      raise self.unknown_exc(key)
    return self.values.get(key)


@pytest.mark.skipif(not _can_import_params(), reason="params_pyx.so is aarch64; run this on the C4 sim")
def test_lane_centering_param_reader_defaults_and_values():
  from openpilot.common.params import UnknownKeyName
  from openpilot.sunnypilot.selfdrive.controls.controlsd_ext import ControlsExt

  ext = ControlsExt.__new__(ControlsExt)
  ext.params = _FakeParams({
    "LaneCentering": True,
    "LaneCenterOffset": -0.12,
    "LaneCenteringE2EAuthority": 0.4,
  })
  ext._update_lane_centering_params()
  assert ext.lane_centering_enabled is True
  assert ext.lane_center_offset == -0.12
  assert ext.lane_centering_e2e_authority == 0.4

  ext.params = _FakeParams(missing=True, unknown_exc=UnknownKeyName)
  ext._update_lane_centering_params()
  assert ext.lane_centering_enabled is False
  assert ext.lane_center_offset == 0.0
  assert ext.lane_centering_e2e_authority == 1.0


@pytest.mark.skipif(not _can_import_params(), reason="params_pyx.so is aarch64; run this on the C4 sim")
def test_shipped_params_so_still_missing_new_keys_until_rebuild():
  from openpilot.common.params import Params, UnknownKeyName

  params = Params()
  try:
    present = all(params.check_key(key) for key in KEYS)
  except UnknownKeyName:
    present = False
  if present:
    assert params.get("LaneCentering", return_default=True) in (False, True)
    assert params.get("LaneCenterOffset", return_default=True) == 0.0
    assert params.get("LaneCenteringE2EAuthority", return_default=True) == 1.0
  else:
    with pytest.raises(UnknownKeyName):
      params.check_key("LaneCentering") or params.get_bool("LaneCentering")
