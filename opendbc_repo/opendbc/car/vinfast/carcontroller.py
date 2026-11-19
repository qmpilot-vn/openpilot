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

# Host Auto Reengage (plaintext). The packaged controller used to latch popup 3
# itself; keep that off and run reengage.py on every VinFast. Settings → Vehicle
# → VinFastReengage turns it off; the car then waits for Resume at every stop.
from pathlib import Path as _Path

from opendbc.car.vinfast.reengage import LEAD_CLASS_CLOSING, LEAD_CLASS_CRITICAL, VinFastReengage

# VF6 / VF7 Plus / VF8 Plus share the FCAM InfoCAN leads and the hybrid ACC path.
VF6_PLATFORM = {"VINFAST_VF6", "VINFAST_VF7_PLUS", "VINFAST_VF8_PLUS"}
CREEP_FINGERPRINTS = {"VINFAST_VF8", "VINFAST_VF9"} | VF6_PLATFORM
# Added to every personality's creep gap. The VF9 stop at 4.6 m (683068420a0d15ba 00000010/37)
# sat too close from the driver's seat; VF8 follows the same gaps. The VF6 InfoCAN range matches
# the bumper-mounted corner radars within 0.1 m at rest (683068420a0d15ba 0000002b, segs 2-11),
# so its gaps are real: the planner rested at 5.4-6.6 m and the driver nudged up to 3.1-4.4 m by
# hand. The VF6 platform creeps from 4.5 m and rests near 3.6 m on aggressive.
CREEP_GAP_OFFSET_M = {"VINFAST_VF8": 1.0, "VINFAST_VF9": 1.0, **{fp: -1.5 for fp in VF6_PLATFORM}}
RESUME_FINGERPRINTS = {"VINFAST_VF8", "VINFAST_VF9"} | VF6_PLATFORM
# The VF6 delivers about half the commanded accel for the first 1.5 s off the line, and the
# 0.6 launch hold sat under a planner asking for 0.7-1.6 (000003fe, 401.7 s / 1631.7 s).
LAUNCH_FOLLOW_FINGERPRINTS = VF6_PLATFORM
# The VF6 InfoCAN lead speed spikes while the range closes, and the C4 vision twin under-reads
# the InfoCAN range by 0.8-1.5 m; the VF8/VF9 MRR creep stays as it is.
RANGE_CONFIRMS_AWAY_FINGERPRINTS = VF6_PLATFORM
# A stopped lead past this already leaves room to go. VF6 creeps to 3.6-5.6 m, so 7 m is past
# every rest point; VF8/VF9 keep 10 m, where 8 m launched inside their creep gap.
LEAVE_FAR_DREL_M = {fp: 7.0 for fp in VF6_PLATFORM}
# The planner's stop-follow (vf_stop_follow) drops the stop behind a lead moving off; the launch takes
# its accel inside the MPC gap and a stop object 0.5 m out and moving counts as leaving.
STOP_FOLLOW_FINGERPRINTS = VF6_PLATFORM | {"VINFAST_VF8", "VINFAST_VF9"}
CREEP_STOP_LATCH_FINGERPRINTS = VF6_PLATFORM


def _reengage_enabled() -> bool:
  """Vehicle-tab toggle. Missing key stays on; get_bool would read that as off."""
  try:
    from openpilot.common.params import Params
    val = Params().get("VinFastReengage", return_default=True)
    if val is None:
      return True
    if isinstance(val, bool):
      return val
    return str(val).strip().lower() in ("1", "true")
  except Exception:
    pass
  path = _Path("/data/params/d/VinFastReengage")
  try:
    return path.read_text(encoding="utf-8").strip().lower() in ("1", "true")
  except OSError:
    return True


_LongCtrlStarting = None
_LongCtrlStopping = None


def _long_ctrl_starting():
  global _LongCtrlStarting
  if _LongCtrlStarting is None:
    from cereal import car
    _LongCtrlStarting = car.CarControl.Actuators.LongControlState.starting
  return _LongCtrlStarting


def _long_ctrl_stopping():
  global _LongCtrlStopping
  if _LongCtrlStopping is None:
    from cereal import car
    _LongCtrlStopping = car.CarControl.Actuators.LongControlState.stopping
  return _LongCtrlStopping


def _long_ctrl_for_accel(accel: float):
  return _long_ctrl_starting() if accel > 0.0 else _long_ctrl_stopping()


class _PopupOff:
  """Drop popup 3 so the core controller cannot run its own auto-go."""

  def __init__(self, cs):
    object.__setattr__(self, "_cs", cs)

  def __getattr__(self, name):
    if name == "acc_popup_feed":
      return 0
    return getattr(self._cs, name)


class _ActuatorsOverride:
  def __init__(self, actuators, accel, long_state):
    object.__setattr__(self, "_act", actuators)
    object.__setattr__(self, "_accel", float(accel))
    object.__setattr__(self, "_long_state", long_state)

  def as_builder(self):
    builder = self._act.as_builder()
    builder.accel = self._accel
    builder.longControlState = self._long_state
    return builder

  def __getattr__(self, name):
    if name == "accel":
      return self._accel
    if name == "longControlState":
      return self._long_state
    return getattr(self._act, name)


class _CCOverride:
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
    self._impl = CarControllerImpl(*args, **kwargs)
    self.reengage = VinFastReengage()
    self._reengage_enabled = _reengage_enabled()
    self._reengage_enabled_check = 0
    fingerprint = str(getattr(self._impl.CP, "carFingerprint", ""))
    self._creep_enabled = fingerprint in CREEP_FINGERPRINTS
    self._creep_gap_offset = CREEP_GAP_OFFSET_M.get(fingerprint, 0.0)
    self._resume_enabled = fingerprint in RESUME_FINGERPRINTS
    self._launch_follow_planner = fingerprint in LAUNCH_FOLLOW_FINGERPRINTS
    self._range_confirms_away = fingerprint in RANGE_CONFIRMS_AWAY_FINGERPRINTS
    self._leave_far_drel = LEAVE_FAR_DREL_M.get(fingerprint)
    self._creep_stop_latches = fingerprint in CREEP_STOP_LATCH_FINGERPRINTS
    self._stop_follow = fingerprint in STOP_FOLLOW_FINGERPRINTS

  def update(self, CC, CC_SP, CS, now_nanos):
    self._reengage_enabled_check += 1
    if self._reengage_enabled_check >= 50:
      self._reengage_enabled_check = 0
      self._reengage_enabled = _reengage_enabled()
    if not self._reengage_enabled:
      return self._impl.update(CC, CC_SP, _PopupOff(CS), now_nanos)

    popup = int(getattr(CS, "acc_popup_feed", 0) or 0)
    allow_launch = bool(getattr(getattr(CC, "cruiseControl", None), "resume", False))
    standstill = bool(getattr(getattr(CS, "out", CS), "standstill", False))
    args = dict(
      popup=popup,
      long_active=bool(getattr(CC, "longActive", False)),
      v_ego=float(getattr(getattr(CS, "out", CS), "vEgo", 0.0)),
      standstill=standstill,
      lead_one=getattr(CC_SP, "leadOne", None),
      lead_two=getattr(CC_SP, "leadTwo", None),
      allow_launch=allow_launch,
      sonar_active=bool(getattr(CS, "sonar_active", False)),
      sonar_front=int(getattr(CS, "sonar_front_inner", 0) or 0),
      sonar_corner=int(getattr(CS, "sonar_front_outer", 0) or 0),
      planner_accel=float(getattr(getattr(CC, "actuators", None), "accel", 0.0)),
      creep_enabled=self._creep_enabled,
      lead_distance_bars=int(getattr(getattr(CC, "hudControl", None), "leadDistanceBars", 2) or 2),
      gap_offset=self._creep_gap_offset,
      resume_enabled=self._resume_enabled,
      launch_follow_planner=self._launch_follow_planner,
      range_confirms_away=self._range_confirms_away,
      leave_far_drel=self._leave_far_drel,
      creep_stop_latches=self._creep_stop_latches,
      stop_follow=self._stop_follow,
    )
    overriding, accel = self.reengage.update(**args)
    active = self.reengage.active
    # Creep brakes on its own TTC. Yielding to a model that wants to stop 3 m short
    # would toggle the override every frame.
    if (overriding and accel is not None and (not standstill) and accel > 0.0 and
        not self.reengage.creeping and
        self.reengage.lead_class in (LEAD_CLASS_CLOSING, LEAD_CLASS_CRITICAL)):
      try:
        model = float(getattr(getattr(CC, "actuators", None), "accel", accel))
        if model < accel:
          # Stepping to the planner's raw accel sent its -2.0 standstill hold at 1.5 km/h
          # (567ee85c1955eff0 000003fe, 1017.7 s / 1592.0 s).
          accel = self.reengage.yield_to_planner(model, args["v_ego"])
      except (TypeError, ValueError, AttributeError):
        pass

    cc = _CCOverride(CC, accel, _long_ctrl_for_accel(accel)) if overriding and accel is not None else CC
    new_actuators, can_sends = self._impl.update(cc, CC_SP, _PopupOff(CS), now_nanos)
    if overriding and accel is not None:
      try:
        new_actuators = new_actuators.as_builder()
        new_actuators.accel = float(accel)
        new_actuators.longControlState = _long_ctrl_for_accel(accel)
        new_actuators.reengageStatus = True
      except Exception:
        pass
    elif hasattr(new_actuators, "reengageStatus"):
      try:
        new_actuators = new_actuators.as_builder()
        new_actuators.reengageStatus = bool(active)
      except Exception:
        pass
    self.reengage.observe_popup(popup)
    return new_actuators, can_sends

  def __getattr__(self, name):
    return getattr(self._impl, name)
