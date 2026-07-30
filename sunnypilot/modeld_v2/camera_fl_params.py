"""
Persistent ecam/fcam focal-length overrides for non-Comma C3XL.

Stored in JSON (not Params) so prebuilt releases work without rebuilding
common/params_pyx.so.
"""
from __future__ import annotations

import json
import os
from pathlib import Path

DEFAULT_ECAM_FL = 650.0
DEFAULT_FCAM_FL = 2700.0

_FILE_CANDIDATES = (
  Path("/data/qmpilot/camera_focal_length.json"),
  Path(os.path.expanduser("~/.qmpilot_camera_focal_length.json")),
  Path("/tmp/qmpilot_camera_focal_length.json"),
)


def _store_path() -> Path:
  for p in _FILE_CANDIDATES:
    try:
      p.parent.mkdir(parents=True, exist_ok=True)
      if p.parent.exists() and os.access(p.parent, os.W_OK):
        return p
    except OSError:
      continue
  return _FILE_CANDIDATES[-1]


def _read_file() -> dict[str, float]:
  path = _store_path()
  if not path.exists():
    return {}
  try:
    data = json.loads(path.read_text())
    out = {}
    if "ecam" in data:
      out["ecam"] = float(data["ecam"])
    if "fcam" in data:
      out["fcam"] = float(data["fcam"])
    return out
  except Exception:
    return {}


def _write_file(ecam: float, fcam: float) -> None:
  path = _store_path()
  path.parent.mkdir(parents=True, exist_ok=True)
  path.write_text(json.dumps({"ecam": float(ecam), "fcam": float(fcam)}, indent=2) + "\n")


def get_focal_lengths() -> tuple[float, float]:
  """Return (ecam_fl, fcam_fl). Defaults 650 / 2700."""
  ecam, fcam = DEFAULT_ECAM_FL, DEFAULT_FCAM_FL
  file_vals = _read_file()
  if "ecam" in file_vals:
    ecam = file_vals["ecam"]
  if "fcam" in file_vals:
    fcam = file_vals["fcam"]
  return float(ecam), float(fcam)


def set_focal_lengths(ecam: float, fcam: float) -> None:
  _write_file(float(ecam), float(fcam))


def set_one_focal_length(which: str, value: float) -> None:
  ecam, fcam = get_focal_lengths()
  if which == "ecam":
    ecam = float(value)
  elif which == "fcam":
    fcam = float(value)
  else:
    raise ValueError(which)
  set_focal_lengths(ecam, fcam)
