import json
from pathlib import Path

import pytest

from openpilot.sunnypilot.modeld_v2 import camera_fl_params as cfp


@pytest.fixture
def fl_store(tmp_path, monkeypatch):
  path = tmp_path / "camera_focal_length.json"
  monkeypatch.setattr(cfp, "_FILE_CANDIDATES", (path,))
  monkeypatch.setattr(cfp, "_store_path", lambda: path)
  return path


def test_defaults_when_missing(fl_store):
  assert not fl_store.exists()
  ecam, fcam = cfp.get_focal_lengths()
  assert ecam == cfp.DEFAULT_ECAM_FL
  assert fcam == cfp.DEFAULT_FCAM_FL
  assert cfp.get_wide_euler_bias_deg() == list(cfp.DEFAULT_WIDE_EULER_BIAS_DEG)
  data = json.loads(fl_store.read_text())
  assert data["ecam"] == 650.0
  assert data["fcam"] == 2700.0
  assert data["wide_euler_bias_deg"] == [0.0, 1.0, 0.0]


def test_roundtrip_file(fl_store):
  cfp.set_focal_lengths(660.0, 2750.0)
  data = json.loads(fl_store.read_text())
  assert data["ecam"] == 660.0
  assert data["fcam"] == 2750.0
  assert data["wide_euler_bias_deg"] == [0.0, 1.0, 0.0]
  assert cfp.get_focal_lengths() == (660.0, 2750.0)


def test_set_one(fl_store):
  cfp.set_focal_lengths(650.0, 2700.0)
  cfp.set_one_focal_length("ecam", 680.0)
  assert cfp.get_focal_lengths() == (680.0, 2700.0)
  cfp.set_one_focal_length("fcam", 2800.0)
  assert cfp.get_focal_lengths() == (680.0, 2800.0)


def test_set_wide_euler_bias(fl_store):
  cfp.set_focal_lengths(650.0, 2700.0)
  cfp.set_wide_euler_bias_deg(0.0, 1.5, -0.5)
  assert cfp.get_wide_euler_bias_deg() == [0.0, 1.5, -0.5]
  data = json.loads(fl_store.read_text())
  assert data["ecam"] == 650.0
  assert data["fcam"] == 2700.0
  assert data["wide_euler_bias_deg"] == [0.0, 1.5, -0.5]


def test_legacy_per_model_bias_dict(fl_store):
  fl_store.write_text(json.dumps({
    "ecam": 650.0,
    "fcam": 2700.0,
    "wide_euler_bias_deg": {
      "PMV2": [0.0, 2.0, 0.0],
      "TCPMV3": [0.0, 1.0, 0.0],
    },
  }))
  # Global list preferred; legacy dict falls back to first usable entry / default key
  bias = cfp.get_wide_euler_bias_deg()
  assert len(bias) == 3
  assert bias == [0.0, 2.0, 0.0] or bias == [0.0, 1.0, 0.0]


def test_legacy_default_key_in_bias_dict(fl_store):
  fl_store.write_text(json.dumps({
    "ecam": 650.0,
    "fcam": 2700.0,
    "wide_euler_bias_deg": {
      "default": [0.0, 1.0, 0.25],
      "PMV2": [0.0, 9.0, 0.0],
    },
  }))
  assert cfp.get_wide_euler_bias_deg() == [0.0, 1.0, 0.25]
