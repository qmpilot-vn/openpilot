"""_NATIVE_SHIM — logic is loaded from values_impl*.so (same Python/ABI as build)."""
import importlib.machinery
import importlib.util
import sys
from pathlib import Path

_IMPL_STEM = "values_impl"
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

# Plaintext steer_press.py + carcontroller yield/reclaim (same as vf-dev-c3xl-mads).
_mod.CarControllerParams.STEER_DRIVER_RELEASE_NM = 0.8
_mod.CarControllerParams.STEER_RELEASE_MIN_COUNT = 50
_mod.CarControllerParams.STEER_SELF_TQ_VF6 = (0.0005, 0.003)
_mod.CarControllerParams.STEER_SELF_TQ_VF8 = (0.0003, 0.012)
_mod.CarControllerParams.STEER_YIELD_START_NM = 1.2
_mod.CarControllerParams.STEER_YIELD_FULL_NM = 2.2
_mod.CarControllerParams.STEER_YIELD_PERSIST_FRAMES = 4
_mod.CarControllerParams.STEER_TAKEOVER_NM = 1.5
_mod.CarControllerParams.STEER_TAKEOVER_NM_CAP = 3.5
_mod.CarControllerParams.STEER_TAKEOVER_IMPULSE_NMS = 0.12
_mod.CarControllerParams.STEER_YIELD_ARM_BLEND = 0.3
_mod.CarControllerParams.STEER_YIELD_ARM_FRAMES = 5
_mod.CarControllerParams.STEER_RECLAIM_FRAMES = 200
_mod.CarControllerParams.STEER_RECLAIM_RATE_MIN = 0.08
# 00000451 / 36ee U-turn: leave follow-hold only when back inside the OEM stop
# and within this gap of the path, then reclaim.
_mod.CarControllerParams.STEER_FOLLOW_GAP_EXIT = 25.0


# VF8 Plus 2025-26: VF8 specs, VF6/VF7 DBC and steer limits, VF6 safety and InfoCAN.
from opendbc.car.lateral import AngleSteeringLimits

_VF8_PLUS = "VINFAST_VF8_PLUS"
if getattr(_mod.CAR, _VF8_PLUS, None) is not None:
  # The packed values module already defines VF8 Plus. Do not rebuild the enum member.
  _plus = getattr(_mod.CAR, _VF8_PLUS)
else:
  _vf8_cfg = _mod.CAR.VINFAST_VF8.config
  _plus_dbc = dict(_mod.CAR.VINFAST_VF6.config.dbc_dict)
  _plus_cfg = _vf8_cfg.override(
    car_docs=[_mod.VinFastCarDocs("VinFast VF8 Plus 2025-26", "All", car_parts=_vf8_cfg.car_docs[0].car_parts)],
    dbc_dict=_plus_dbc,
  )
  _plus_cfg.platform_str = _VF8_PLUS
  _plus_cfg.freeze()
  _plus = str.__new__(_mod.CAR, _VF8_PLUS)
  _plus._name_ = _VF8_PLUS
  _plus._value_ = _VF8_PLUS
  _plus.config = _plus_cfg
  _plus._sort_order_ = len(_mod.CAR._member_names_)
  setattr(_mod.CAR, _VF8_PLUS, _plus)
  _mod.CAR._member_map_[_VF8_PLUS] = _plus
  _mod.CAR._value2member_map_[_VF8_PLUS] = _plus
  _mod.CAR._member_names_.append(_VF8_PLUS)
  _mod.DBC[_plus] = _plus_cfg.dbc_dict

# VF8 Eco 2023-24: VF8 body, DBC and MRR, Eco curb weight (2328 kg). Keep on staging.
_VF8_ECO = "VINFAST_VF8_ECO"
if getattr(_mod.CAR, _VF8_ECO, None) is not None:
  _eco = getattr(_mod.CAR, _VF8_ECO)
else:
  _vf8_for_eco = _mod.CAR.VINFAST_VF8.config
  _eco_cfg = _vf8_for_eco.override(
    car_docs=[_mod.VinFastCarDocs("VinFast VF8 Eco 2023-24", "All", car_parts=_vf8_for_eco.car_docs[0].car_parts)],
    specs=_vf8_for_eco.specs.override(mass=2328.0),
  )
  _eco_cfg.platform_str = _VF8_ECO
  _eco_cfg.freeze()
  _eco = str.__new__(_mod.CAR, _VF8_ECO)
  _eco._name_ = _VF8_ECO
  _eco._value_ = _VF8_ECO
  _eco.config = _eco_cfg
  _eco._sort_order_ = len(_mod.CAR._member_names_)
  setattr(_mod.CAR, _VF8_ECO, _eco)
  _mod.CAR._member_map_[_VF8_ECO] = _eco
  _mod.CAR._value2member_map_[_VF8_ECO] = _eco
  _mod.CAR._member_names_.append(_VF8_ECO)
  _mod.DBC[_eco] = _eco_cfg.dbc_dict


_orig_is_vf6 = _mod.is_vf6_safety_platform

def _is_vf6_safety_platform(candidate) -> bool:
  name = candidate if isinstance(candidate, str) else getattr(candidate, "name", None)
  if name == _VF8_PLUS or candidate is _plus:
    return True
  return _orig_is_vf6(candidate)

_mod.is_vf6_safety_platform = _is_vf6_safety_platform

_orig_params_init = _mod.CarControllerParams.__init__

def _params_init(self, CP):
  _orig_params_init(self, CP)
  name = CP.carFingerprint if isinstance(CP.carFingerprint, str) else getattr(CP.carFingerprint, "name", None)
  if name != _VF8_PLUS:
    return
  steer_max = 180.0
  bp, rate = _mod.VF6_STEER_SPEED_BP_MS, _mod.VF6_STEER_RATE_V_DEG_PER_STEP
  self.STEER_MAX = int(steer_max)
  self.ANGLE_LIMITS = AngleSteeringLimits(steer_max, (bp, rate), (bp, rate))
  self.ANGLE_MAX_LOOKUP = (bp, _mod.VF6_STEER_ANGLE_MAX_V_DEG)

_mod.CarControllerParams.__init__ = _params_init

# Packed radard and radar_interface call mrr_half_width. Eco is the VF8 body.
def _alias_vf8(table):
  if isinstance(table, dict) and "VINFAST_VF8" in table and _VF8_ECO not in table:
    table[_VF8_ECO] = table["VINFAST_VF8"]

_alias_vf8(getattr(_mod, "MRR_WIDTH_M", None))
_alias_vf8(getattr(_mod, "MRR_HALF_WIDTH", None))

_orig_mrr_half = getattr(_mod, "mrr_half_width", None)
if _orig_mrr_half is not None:
  def _mrr_half_width(fingerprint):
    name = fingerprint if isinstance(fingerprint, str) else getattr(fingerprint, "name", None)
    if name == _VF8_ECO:
      return _orig_mrr_half("VINFAST_VF8")
    return _orig_mrr_half(fingerprint)

  _mod.mrr_half_width = _mrr_half_width
