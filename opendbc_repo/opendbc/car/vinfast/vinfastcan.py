"""_NATIVE_SHIM — logic is loaded from vinfastcan_impl*.so (same Python/ABI as build)."""
import importlib.machinery
import importlib.util
import sys
from pathlib import Path

_IMPL_STEM = "vinfastcan_impl"
_dir = Path(__file__).resolve().parent
_so = next((p for suf in importlib.machinery.EXTENSION_SUFFIXES if (p := _dir / f"{_IMPL_STEM}{suf}").is_file()), None)
if _so is None:
  raise ImportError(
    "Missing native %s*.so — rebuild with opendbc_repo/scripts/package_vinfast_so.sh using Python %s"
    % (_IMPL_STEM, ".".join(map(str, sys.version_info[:3])))
  )
# Load under the extension's short name so PyInit_<stem> matches, then publish as this module.
_spec = importlib.util.spec_from_file_location(_IMPL_STEM, _so)
assert _spec and _spec.loader
_mod = importlib.util.module_from_spec(_spec)
sys.modules[_IMPL_STEM] = _mod
try:
  _spec.loader.exec_module(_mod)
finally:
  sys.modules.pop(_IMPL_STEM, None)
# Cython tags classes with __module__=_IMPL_STEM; car_helpers needs opendbc.car.<brand>.*
for _attr in dir(_mod):
  _obj = getattr(_mod, _attr)
  if getattr(_obj, "__module__", None) == _IMPL_STEM:
    try:
      _obj.__module__ = __name__
    except (AttributeError, TypeError):
      pass
_mod.__name__ = __name__
_mod.__package__ = __package__
_mod.__file__ = str(_so)
_mod.__loader__ = _spec.loader
_mod.__spec__ = _spec
sys.modules[__name__] = _mod

# Host overlay: angle-based Torq_Fact_Req, scaled down while the driver yield
# blend is up (same as vf-dev-c3xl-mads / vf-dev-c4).
from opendbc.car.vinfast.steer_torque import torque_factor as _torque_factor
from opendbc.car.vinfast.values import CANBUS as _CANBUS


def _apply_checksum(packer, msg_name, bus, checksum_field, values):
  msg = packer.make_can_msg(msg_name, bus, values)
  checksum = _mod.vinfast_checksum(msg[1])
  values[checksum_field] = checksum
  return packer.make_can_msg(msg_name, bus, values)


def scale_torque_factor(factor: float, yield_blend: float) -> float:
  """Move Torq_Fact_Req toward its 0.5 floor as yield_blend goes 0..1."""
  blend = min(1.0, max(0.0, float(yield_blend)))
  return float(factor) + (0.5 - float(factor)) * blend


def create_steering_control(packer, CP, frame, apply_angle, lat_active, yield_blend=0.0):
  alive = frame % 15
  factor = _torque_factor(apply_angle, lat_active, CP.carFingerprint)
  if lat_active:
    factor = scale_torque_factor(factor, yield_blend)
  values = {
    "CHKSM_ADAS_EPS_LATE_CON": 0,
    "ALV_ADAS_EPS_LATE_CON": alive,
    "ADAS_EPS_StrWhe_TOLAct": 0,
    "ADAS_EPS_StrWhe_AOLAct": 1 if lat_active else 0,
    "ADAS_EPS_AOLReq": apply_angle if lat_active else 0.0,
    "ADAS_EPS_Torq_Fact_Req": factor,
    "SECCAN_ADAS_EPS_LATE_CON": 0,
  }
  return _apply_checksum(packer, "ADAS_EPS_LATE_CON", _CANBUS.chassis, "CHKSM_ADAS_EPS_LATE_CON", values)


_mod.scale_torque_factor = scale_torque_factor
_mod.create_steering_control = create_steering_control
