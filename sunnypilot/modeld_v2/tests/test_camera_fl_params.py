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


def test_roundtrip_file(fl_store):
  cfp.set_focal_lengths(660.0, 2750.0)
  data = json.loads(fl_store.read_text())
  assert data == {"ecam": 660.0, "fcam": 2750.0}
  assert cfp.get_focal_lengths() == (660.0, 2750.0)


def test_set_one(fl_store):
  cfp.set_focal_lengths(650.0, 2700.0)
  cfp.set_one_focal_length("ecam", 680.0)
  assert cfp.get_focal_lengths() == (680.0, 2700.0)
  cfp.set_one_focal_length("fcam", 2800.0)
  assert cfp.get_focal_lengths() == (680.0, 2800.0)
