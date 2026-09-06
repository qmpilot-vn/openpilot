"""_NATIVE_SHIM — logic is loaded from carcontroller_impl*.so (same Python/ABI as build)."""
import importlib.machinery
import importlib.util
import sys
from pathlib import Path

_IMPL_STEM = "carcontroller_impl"
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

# Host-side reengage: latch at red, launch only when planner resume + in-lane
# lead is actually receding. Disables the packaged popup kick so InfoCAN dRel
# chatter on the 7 m VF6 edge cannot flip CAN accel -1.0 <-> +0.6.
from opendbc.car.vinfast.reengage import VinFastReengage, is_vf6_reengage_platform

_ImplCC = _mod.CarController
_LongCtrlStarting = None


def _long_ctrl_starting():
  global _LongCtrlStarting
  if _LongCtrlStarting is None:
    from cereal import car
    _LongCtrlStarting = car.CarControl.Actuators.LongControlState.starting
  return _LongCtrlStarting


class _PopupOff:
  def __init__(self, cs):
    object.__setattr__(self, "_cs", cs)

  def __getattr__(self, name):
    if name == "acc_popup_feed":
      return 0
    return getattr(self._cs, name)


class _ActuatorsOverride:
  """Reader-like actuators with override accel. Native impl calls as_builder() on a Reader."""

  def __init__(self, actuators, accel, long_state):
    object.__setattr__(self, "_act", actuators)
    object.__setattr__(self, "_accel", float(accel))
    object.__setattr__(self, "_long_state", long_state)

  def as_builder(self):
    b = self._act.as_builder()
    b.accel = self._accel
    b.longControlState = self._long_state
    return b

  def __getattr__(self, name):
    if name == "accel":
      return self._accel
    if name == "longControlState":
      return self._long_state
    return getattr(self._act, name)


class _CCOverride:
  """Do not pass a capnp Builder into the .so — it then does actuators.as_builder() and crashes."""

  def __init__(self, cc, accel, long_state):
    object.__setattr__(self, "_cc", cc)
    object.__setattr__(self, "_actuators", _ActuatorsOverride(cc.actuators, accel, long_state))

  def as_builder(self):
    return self

  def __getattr__(self, name):
    if name == "actuators":
      return self._actuators
    return getattr(self._cc, name)


class CarController:
  def __init__(self, *args, **kwargs):
    self._impl = _ImplCC(*args, **kwargs)
    cp = args[1] if len(args) > 1 else kwargs.get("CP")
    fp = getattr(cp, "carFingerprint", "") if cp is not None else ""
    self.reengage = VinFastReengage(vf6=is_vf6_reengage_platform(fp))

  def update(self, CC, CC_SP, CS, now_nanos):
    popup = int(getattr(CS, "acc_popup_feed", 0) or 0)
    allow_launch = bool(getattr(getattr(CC, "cruiseControl", None), "resume", False))
    overriding, accel = self.reengage.update(
      popup=popup,
      long_active=bool(getattr(CC, "longActive", False)),
      v_ego=float(getattr(getattr(CS, "out", CS), "vEgo", 0.0)),
      standstill=bool(getattr(getattr(CS, "out", CS), "standstill", False)),
      lead_one=getattr(CC_SP, "leadOne", None),
      lead_two=getattr(CC_SP, "leadTwo", None),
      allow_launch=allow_launch,
    )
    cc = CC
    if overriding and accel is not None:
      cc = _CCOverride(CC, accel, _long_ctrl_starting())
    cs = _PopupOff(CS)
    new_actuators, can_sends = self._impl.update(cc, CC_SP, cs, now_nanos)
    if overriding and accel is not None:
      try:
        new_actuators = new_actuators.as_builder()
        new_actuators.accel = float(accel)
        new_actuators.longControlState = _long_ctrl_starting()
        new_actuators.reengageStatus = True
      except Exception:
        pass
    elif hasattr(new_actuators, "reengageStatus"):
      try:
        new_actuators = new_actuators.as_builder()
        new_actuators.reengageStatus = bool(self.reengage.active)
      except Exception:
        pass
    self.reengage.observe_popup(popup)
    return new_actuators, can_sends

  def __getattr__(self, name):
    return getattr(self._impl, name)


_mod.CarController = CarController
