"""VF8/VF9 option to take the set speed from the car, the way VF6 does.

The shipped params process does not know a new key until it is rebuilt, and
manager deletes unknown files under /data/params/d. The choice lives in
/data/qmpilot, same as the other VinFast switches. Missing means off, so
qmpilot keeps owning the set speed.
"""

import os
from pathlib import Path

PARAM = "VinFastCarSetSpeed"
STORE_DIRS = (Path("/data/qmpilot"), Path("/persist/qmpilot"))
# VF8 Plus is already on the VF6 safety platform, so its set speed follows the car.
VF8_VF9 = ("VINFAST_VF8", "VINFAST_VF9", "VINFAST_VF8_ECO")


def _fingerprint_name(fingerprint) -> str:
  if isinstance(fingerprint, str):
    return fingerprint
  return getattr(fingerprint, "name", None) or str(fingerprint or "")


def applies_to(fingerprint) -> bool:
  return _fingerprint_name(fingerprint) in VF8_VF9


def _parse_on(raw) -> bool | None:
  if raw is None:
    return None
  if isinstance(raw, bool):
    return raw
  text = str(raw).strip().lower()
  if text in ("1", "true"):
    return True
  if text in ("0", "false"):
    return False
  return None


def _paths(path=None):
  if path is not None:
    return [Path(path)]
  return [directory / PARAM for directory in STORE_DIRS]


def _atomic_write(dest: Path, text: str) -> None:
  try:
    dest.parent.mkdir(parents=True, exist_ok=True)
    tmp = dest.with_name(dest.name + ".tmp")
    tmp.write_text(text, encoding="utf-8")
    os.replace(tmp, dest)
  except OSError:
    pass


def car_set_speed_enabled(path=None) -> bool:
  """Default off. A missing choice keeps qmpilot's own set speed."""
  for dest in _paths(path):
    try:
      parsed = _parse_on(dest.read_text(encoding="utf-8"))
    except OSError:
      continue
    if parsed is not None:
      return parsed
  return False


def set_car_set_speed(on: bool, path=None) -> None:
  text = "1" if on else "0"
  for dest in _paths(path):
    _atomic_write(dest, text)
