"""
Persistent ecam/fcam focal-length + wide-euler bias for non-Comma C3XL.

Stored in JSON (not Params) so prebuilt releases work without rebuilding
common/params_pyx.so.

Device file: /data/qmpilot/camera_focal_length.json
"""
from __future__ import annotations

import json
import os
from pathlib import Path

DEFAULT_ECAM_FL = 650.0
DEFAULT_FCAM_FL = 2700.0
# Device-wide overlay bias on liveCalibration.wideFromDeviceEuler (degrees).
DEFAULT_WIDE_EULER_BIAS_DEG = (0.0, 1.0, 0.0)  # roll, pitch, yaw

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


def _parse_bias(raw) -> list[float] | None:
  """Parse bias as [r,p,y]. Accepts list or legacy per-model dict."""
  if raw is None:
    return None
  if isinstance(raw, (list, tuple)) and len(raw) == 3:
    return [float(raw[0]), float(raw[1]), float(raw[2])]
  if isinstance(raw, dict) and raw:
    # Legacy per-model dict: prefer default/*, else any entry with len 3.
    for key in ("default", "*", "global"):
      if key in raw:
        return _parse_bias(raw[key])
    for val in raw.values():
      parsed = _parse_bias(val)
      if parsed is not None:
        return parsed
  return None


def _read_file() -> dict:
  path = _store_path()
  if not path.exists():
    return {}
  try:
    data = json.loads(path.read_text())
    return data if isinstance(data, dict) else {}
  except Exception:
    return {}


def _write_file(ecam: float, fcam: float, bias_deg: list[float] | tuple[float, float, float]) -> None:
  path = _store_path()
  path.parent.mkdir(parents=True, exist_ok=True)
  payload = {
    "ecam": float(ecam),
    "fcam": float(fcam),
    "wide_euler_bias_deg": [float(bias_deg[0]), float(bias_deg[1]), float(bias_deg[2])],
  }
  path.write_text(json.dumps(payload, indent=2) + "\n")


def get_focal_lengths() -> tuple[float, float]:
  """Return (ecam_fl, fcam_fl). Defaults 650 / 2700. Writes defaults file if missing."""
  path = _store_path()
  data = _read_file()
  ecam = float(data["ecam"]) if "ecam" in data else DEFAULT_ECAM_FL
  fcam = float(data["fcam"]) if "fcam" in data else DEFAULT_FCAM_FL
  bias = _parse_bias(data.get("wide_euler_bias_deg")) or list(DEFAULT_WIDE_EULER_BIAS_DEG)
  if not path.exists():
    _write_file(ecam, fcam, bias)
  return ecam, fcam


def get_wide_euler_bias_deg() -> list[float]:
  """Return [roll, pitch, yaw] degrees. Default [0, 1, 0]."""
  data = _read_file()
  bias = _parse_bias(data.get("wide_euler_bias_deg"))
  if bias is None:
    return list(DEFAULT_WIDE_EULER_BIAS_DEG)
  return bias


def set_focal_lengths(ecam: float, fcam: float) -> None:
  bias = get_wide_euler_bias_deg()
  _write_file(float(ecam), float(fcam), bias)


def set_wide_euler_bias_deg(roll: float, pitch: float, yaw: float) -> None:
  ecam, fcam = get_focal_lengths()
  _write_file(ecam, fcam, [float(roll), float(pitch), float(yaw)])


def set_one_focal_length(which: str, value: float) -> None:
  ecam, fcam = get_focal_lengths()
  if which == "ecam":
    ecam = float(value)
  elif which == "fcam":
    fcam = float(value)
  else:
    raise ValueError(which)
  set_focal_lengths(ecam, fcam)
