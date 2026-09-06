"""_BYTECODE_SHIM — logic is loaded from __pycache__/radard_impl.*.pyc (same Python minor as build).

VF6/VF7 InfoCAN post-load patches (VF8/VF9 measured radar is unchanged):
* red-light stick-slip: clamp near-stopped leads so ego speed does not ping-pong
* urban following: position-only aLeadK/vLead spikes look like emergency braking
  (seg11 ~21.7s: aLeadK pegged at -3.5 while vision still ~7 m/s)
* hold a stopped ~7 m InfoCAN lead across a one-frame 3.7 m vision swap

VF8/VF9 + PMV2 highway post-load patches:
* radar-only / candidate range and path corridor were too tight above ~100 km/h
* |vRel| and static-lead caps dropped slowing/stopped in-path cars until close

Ported from vf-dev-c3xl:
* is_urban_crossing_miss TTC horizon (74d51be)
* urban MRR vision-speed / range-rate / lead-hold / closer-UNKNOWN (20a104f)
* radar_interface_impl .so with MAX_DX_STD_ANY_FRAC=0.8 (3893948)
"""
import importlib.util
import sys
import time
from pathlib import Path
from typing import Any

from openpilot.selfdrive.controls.lib.infocan_lead import InfoCanLeadHold

_IMPL_STEM = "radard_impl"
_pyc = Path(__file__).resolve().parent / "__pycache__" / f"{_IMPL_STEM}.{sys.implementation.cache_tag}.pyc"
if not _pyc.is_file():
  raise ImportError(
    "Missing bytecode %s — rebuild with package_vinfast_modules_bytecode.sh using Python %s"
    % (_pyc, ".".join(map(str, sys.version_info[:3])))
  )
_spec = importlib.util.spec_from_file_location(__name__, _pyc)
assert _spec and _spec.loader
_mod = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_mod)

# InfoCAN Kalman step was 2.0 m/s per frame — far too large for creep / red-light.
if hasattr(_mod, "POSITION_ONLY_MAX_VLEAD_STEP"):
  _mod.POSITION_ONLY_MAX_VLEAD_STEP = 0.5

# Blend toward vision sooner: the 30 km/h moto event had |vRadar−vVision|≈2.3 m/s,
# under the old 3.0 threshold, so MPC kept the fake radar kinematics.
if hasattr(_mod, "POSITION_ONLY_FUSION_MAX_VABS"):
  _mod.POSITION_ONLY_FUSION_MAX_VABS = 1.5

# Impl interpolates to -3.5 below 40 m. That is what long MPC treated as an
# emergency stop on a rolling moto whose dRel was merely stick-slipping.
if hasattr(_mod, "POSITION_ONLY_ALEADK_V"):
  _mod.POSITION_ONLY_ALEADK_V = (-1.2, -1.0)

# Real pyc knobs (INFO_HOST_* never existed). Widen creep path for VN lane-share.
if hasattr(_mod, "URBAN_CREEP_PATH_MAX_LAT"):
  _mod.URBAN_CREEP_PATH_MAX_LAT = 1.8

# VF8/VF9 + PMV2 alpha-long: vision lead_prob locks late at highway speed, so
# radard falls through to radar-only. MRR already reports to 300 m, but several
# gates then drop the in-path track until it is already close.
_VF_HIGHWAY_VEGO = 100.0 / 3.6
_VF_RADAR_ONLY_MAX_DIST = 220.0
_VF_MAX_LEAD_DIST = 250.0
_VF_RADAR_ONLY_MAX_LAT = 4.5
_VF_STATIC_LEAD_DIST_HIGHWAY = 180.0
_VF_PATH_TOL_BP = [5.0, 30.0, 60.0, 120.0, 220.0]
_VF_PATH_TOL_V = [1.2, 1.5, 2.5, 3.5, 4.5]
_VF_URBAN_PATH_TOL_V = [1.9, 2.0, 2.5, 3.5, 4.5]
if hasattr(_mod, "RADAR_ONLY_MAX_DIST"):
  _mod.RADAR_ONLY_MAX_DIST = _VF_RADAR_ONLY_MAX_DIST
if hasattr(_mod, "MAX_LEAD_DIST"):
  _mod.MAX_LEAD_DIST = _VF_MAX_LEAD_DIST
if hasattr(_mod, "RADAR_ONLY_MAX_LAT"):
  _mod.RADAR_ONLY_MAX_LAT = _VF_RADAR_ONLY_MAX_LAT
if hasattr(_mod, "PATH_TOL_BP"):
  _mod.PATH_TOL_BP = _VF_PATH_TOL_BP
if hasattr(_mod, "PATH_TOL_V"):
  _mod.PATH_TOL_V = _VF_PATH_TOL_V
if hasattr(_mod, "URBAN_PATH_TOL_V"):
  _mod.URBAN_PATH_TOL_V = _VF_URBAN_PATH_TOL_V

_orig_max_vrel = float(getattr(_mod, "MAX_VREL_FILTER", 25.0))
_orig_max_static = float(getattr(_mod, "MAX_STATIC_LEAD_DIST", 50.0))
_orig_select_best = getattr(_mod, "select_best_radar_track", None)
if _orig_select_best is not None:
  def select_best_radar_track(tracks, v_ego, *args, **kwargs):
    # Stopped lead |vRel| ≈ v_ego; +10 m/s covers far-range vRel noise.
    # Oncoming is ~2 v_ego and still rejected.
    _mod.MAX_VREL_FILTER = max(_orig_max_vrel, float(v_ego) + 10.0)
    if float(v_ego) >= _VF_HIGHWAY_VEGO:
      _mod.MAX_STATIC_LEAD_DIST = _VF_STATIC_LEAD_DIST_HIGHWAY
    else:
      _mod.MAX_STATIC_LEAD_DIST = _orig_max_static
    try:
      return _orig_select_best(tracks, v_ego, *args, **kwargs)
    finally:
      _mod.MAX_VREL_FILTER = _orig_max_vrel
      _mod.MAX_STATIC_LEAD_DIST = _orig_max_static

  _mod.select_best_radar_track = select_best_radar_track

# Highway: is_urban_crossing_miss projects only 4 s of yvRel, so a 9 s TTC at 90 m
# looks like a 4 m miss and a dead-centre closer is dropped. Skip when TTC is
# outside the horizon. Same wrap as vf-dev-c3xl 74d51be.
_orig_urban_crossing_miss = getattr(_mod, "is_urban_crossing_miss", None)
_orig_predicted_lat_at_ttc = getattr(_mod, "predicted_lateral_offset_at_ttc", None)
if _orig_urban_crossing_miss is not None and _orig_predicted_lat_at_ttc is not None:
  _VF_CROSSING_MISS_MAX_TTC = float(getattr(_mod, "URBAN_CROSSING_TTC_HORIZON", 4.0))

  def is_urban_crossing_miss(track, v_ego, y_rel_path):
    _lat, ttc = _orig_predicted_lat_at_ttc(track, v_ego, y_rel_path)
    if ttc > _VF_CROSSING_MISS_MAX_TTC:
      return False
    return _orig_urban_crossing_miss(track, v_ego, y_rel_path)

  _mod.is_urban_crossing_miss = is_urban_crossing_miss

# Treat lead as stationary when absolute speed estimate is below this at low ego speed.
_VF_STOPPED_LEAD_V = 1.2  # [m/s]
# Position-only aLeadK floor for published leads. Impl interpolates to -3.5 below 40 m
# (hardcoded left= in _position_only_aleadk_floor); that is what long MPC treated as
# an emergency stop. Keep a real close TTC uncapped.
_VF_INFOCAN_ALEADK_FLOOR = -1.2  # [m/s²]
_VF_INFOCAN_VLEAD_STEP = 0.25    # [m/s] per ~50 ms frame → 5 m/s²
_VF_INFOCAN_TTC_URGENT = 1.8     # [s]
_VF_INFOCAN_URGENT_DREL = 12.0   # [m]
_prev_vleadk: dict[int, float] = {}


def _stabilize_stopped_infocan_lead(lead_dict: dict[str, Any],
                                    track: Any,
                                    v_ego: float) -> dict[str, Any]:
  """Force stationary kinematics for near-stopped InfoCAN leads at low speed."""
  if (not lead_dict.get("status", False)) or (not lead_dict.get("radar", False)):
    return lead_dict
  if getattr(track, "measured", True):
    return lead_dict
  smooth_vego = float(getattr(_mod, "LOW_SPEED_LEAD_SMOOTH_VEGO", 20.0 / 3.6))
  if v_ego >= smooth_vego:
    return lead_dict

  v_lead_k = float(getattr(track, "vLeadK", lead_dict.get("vLeadK", 0.0)))
  v_rel = float(lead_dict.get("vRel", v_lead_k - v_ego))
  # Near-zero absolute lead, or noisy "opening" while ego creeps toward a stopper.
  stopped = abs(v_lead_k) < _VF_STOPPED_LEAD_V
  false_opening = v_ego < 3.0 and v_rel > -0.15 and abs(v_lead_k) < 2.0
  if not (stopped or false_opening):
    return lead_dict

  lead_dict["vLead"] = 0.0
  lead_dict["vLeadK"] = 0.0
  lead_dict["vRel"] = -float(v_ego)
  # Kill accel chatter that makes long MPC oscillate at standstill.
  a = float(lead_dict.get("aLeadK", 0.0))
  lead_dict["aLeadK"] = min(0.0, max(-0.3, a))
  return lead_dict


def _temper_position_only_lead(lead_dict: dict[str, Any],
                               track: Any,
                               v_ego: float) -> dict[str, Any]:
  """Stop InfoCAN range stick-slip from looking like an emergency stop to long MPC."""
  if (not lead_dict.get("status", False)) or (not lead_dict.get("radar", False)):
    return lead_dict
  if getattr(track, "measured", True):
    return lead_dict

  d_rel = float(lead_dict.get("dRel", getattr(track, "dRel", 0.0)))
  v_lead = float(lead_dict.get("vLeadK", getattr(track, "vLeadK", 0.0)))
  closing = max(0.0, float(v_ego) - v_lead)
  ttc = d_rel / max(closing, 0.1)
  urgent = ttc < _VF_INFOCAN_TTC_URGENT and d_rel < _VF_INFOCAN_URGENT_DREL

  if not urgent:
    a = float(lead_dict.get("aLeadK", 0.0))
    lead_dict["aLeadK"] = max(_VF_INFOCAN_ALEADK_FLOOR, a)
    tid = int(getattr(track, "identifier", lead_dict.get("radarTrackId", -1)))
    prev = _prev_vleadk.get(tid)
    if prev is not None:
      v_lead = max(prev - _VF_INFOCAN_VLEAD_STEP, min(prev + _VF_INFOCAN_VLEAD_STEP, v_lead))
    if tid >= 0:
      _prev_vleadk[tid] = v_lead
    lead_dict["vLeadK"] = v_lead
    lead_dict["vLead"] = v_lead
    lead_dict["vRel"] = v_lead - float(v_ego)
  return lead_dict


_orig_stabilize_low_speed = getattr(_mod, "stabilize_low_speed_radar_lead", None)
if _orig_stabilize_low_speed is not None:
  def stabilize_low_speed_radar_lead(lead_dict, track, v_ego):
    lead_dict = _orig_stabilize_low_speed(lead_dict, track, v_ego)
    lead_dict = _stabilize_stopped_infocan_lead(lead_dict, track, v_ego)
    return _temper_position_only_lead(lead_dict, track, v_ego)

  _mod.stabilize_low_speed_radar_lead = stabilize_low_speed_radar_lead

_orig_stabilize_position_only = getattr(_mod, "stabilize_position_only_radar_lead", None)
if _orig_stabilize_position_only is not None:
  def stabilize_position_only_radar_lead(lead_dict, track, v_ego):
    lead_dict = _orig_stabilize_position_only(lead_dict, track, v_ego)
    return _temper_position_only_lead(lead_dict, track, v_ego)

  _mod.stabilize_position_only_radar_lead = stabilize_position_only_radar_lead

# A 3.7 m InfoCAN ghost has TTC/dRel that bypasses the packaged 3.5 m hold.
# Tighten bypass only for unmeasured tracks; VF8/VF9 Doppler is unchanged.
_orig_stability_hold = getattr(_mod, "apply_lead_stability_hold", None)
if _orig_stability_hold is not None:
  def apply_lead_stability_hold(lead_dict, track, v_ego):
    if getattr(track, "measured", True):
      return _orig_stability_hold(lead_dict, track, v_ego)
    old_d = getattr(_mod, "LEAD_STABILITY_MIN_DREL", 3.5)
    old_ttc = getattr(_mod, "LEAD_STABILITY_BYPASS_TTC", 1.0)
    _mod.LEAD_STABILITY_MIN_DREL = 2.5
    _mod.LEAD_STABILITY_BYPASS_TTC = 0.6
    try:
      return _orig_stability_hold(lead_dict, track, v_ego)
    finally:
      _mod.LEAD_STABILITY_MIN_DREL = old_d
      _mod.LEAD_STABILITY_BYPASS_TTC = old_ttc

  _mod.apply_lead_stability_hold = apply_lead_stability_hold

# MRR lead guards from vf-dev-c3xl. Gated at 22 m/s so highway is unchanged.
_VF_GUARD_MAX_VEGO = 22.0
_VF_VISION_TRUST_DREL = 12.0
_VF_VISION_TRUST_PROB = 0.5
_VF_VISION_SLOWER_MIN = 1.0
_VF_RATE_SANITY_ERR = 2.0
_VF_RATE_SANITY_FRAMES = 3
_VF_RATE_MAX_DT = 0.3
_VF_RATE_HIST_TTL = 3.0
_VF_JUMP_OUTWARD_M = 3.5
_VF_JUMP_HOLD_T = 0.6
_VF_JUMP_HOLD_FRAMES = 4
_VF_JUMP_FAR_M = 10.0
_VF_HOLD_MAX_DREL = 40.0
_VF_HOLD_NEAR_DREL = 12.0
_VF_HOLD_MAX_VLEAD = 2.0
_VF_HOLD_MIN_CLOSING = 1.5
_VF_HOLD_MAX_ADVANCE = 2.0
_rate_hist: dict[int, tuple[float, float, int]] = {}
_lead_hold: dict[int, dict[str, Any]] = {}


def _set_lead_speed(lead: dict[str, Any], v_lead: float, v_ego: float) -> None:
  lead["vLeadK"] = v_lead
  lead["vLead"] = v_lead
  lead["vRel"] = v_lead - float(v_ego)


def _vision_lead(lead_msg: Any, v_ego: float, model_v_ego: float) -> tuple[float, float] | None:
  if lead_msg is None:
    return None
  try:
    if float(lead_msg.prob) < _VF_VISION_TRUST_PROB:
      return None
    d_rel = float(lead_msg.x[0]) - float(getattr(_mod, "RADAR_TO_CAMERA", 1.52))
    v_lead = float(v_ego) + (float(lead_msg.v[0]) - float(model_v_ego))
  except (AttributeError, IndexError, TypeError, ValueError):
    return None
  return d_rel, v_lead


def _apply_vision_speed_guard(lead: dict[str, Any], lead_msg: Any,
                              v_ego: float, model_v_ego: float) -> dict[str, Any]:
  vis = _vision_lead(lead_msg, v_ego, model_v_ego)
  if vis is None:
    return lead
  d_vis, v_vis = vis
  if abs(d_vis - float(lead["dRel"])) > _VF_VISION_TRUST_DREL:
    return lead
  if float(lead["vLeadK"]) - v_vis < _VF_VISION_SLOWER_MIN:
    return lead
  _set_lead_speed(lead, v_vis, v_ego)
  return lead


def _apply_range_rate_guard(lead: dict[str, Any], v_ego: float, now: float) -> dict[str, Any]:
  for tid, (seen, _, _) in list(_rate_hist.items()):
    if now - seen > _VF_RATE_HIST_TTL:
      del _rate_hist[tid]
  tid = int(lead.get("radarTrackId", -1))
  d_rel = float(lead["dRel"])
  prev = _rate_hist.get(tid)
  if prev is None:
    _rate_hist[tid] = (now, d_rel, 0)
    return lead
  t_prev, d_prev, bad = prev
  dt = now - t_prev
  if not 0.02 < dt < _VF_RATE_MAX_DT:
    _rate_hist[tid] = (now, d_rel, bad)
    return lead
  closing = (d_prev - d_rel) / dt
  if abs(closing) > float(getattr(_mod, "MAX_VREL_FILTER", 25.0)):
    _rate_hist[tid] = (now, d_rel, bad)
    return lead
  v_rate = float(v_ego) - closing
  bad = bad + 1 if float(lead["vLeadK"]) - v_rate >= _VF_RATE_SANITY_ERR else 0
  _rate_hist[tid] = (now, d_rel, bad)
  if bad >= _VF_RATE_SANITY_FRAMES:
    _set_lead_speed(lead, min(float(lead["vLeadK"]), v_rate), v_ego)
  return lead


def _hold_candidate(lead: dict[str, Any], v_ego: float) -> bool:
  if not lead.get("status", False):
    return False
  d_rel = float(lead.get("dRel", 1e3))
  if d_rel < _VF_HOLD_NEAR_DREL:
    return True
  if d_rel >= _VF_HOLD_MAX_DREL:
    return False
  v_lead = float(lead.get("vLeadK", 1e3))
  return abs(v_lead) < _VF_HOLD_MAX_VLEAD or float(v_ego) - v_lead > _VF_HOLD_MIN_CLOSING


def _apply_lead_hold(lead: dict[str, Any], slot: int, v_ego: float, now: float) -> dict[str, Any]:
  held = _lead_hold.get(slot)
  if held is not None and now - held["t"] > _VF_JUMP_HOLD_T:
    del _lead_hold[slot]
    held = None
  if held is not None:
    prev = held["lead"]
    v_hold = float(prev["vLeadK"])
    advance = min(max(0.0, (float(v_ego) - v_hold) * (now - held["t"])), _VF_HOLD_MAX_ADVANCE)
    d_hold = max(0.5, float(prev["dRel"]) - advance)
    live = bool(lead.get("status", False))
    step = float(lead["dRel"]) - d_hold if live else 0.0
    jumped = live and step > _VF_JUMP_OUTWARD_M
    if jumped and step <= _VF_JUMP_FAR_M:
      held["n"] += 1
    if (not live) or (jumped and held["n"] <= _VF_JUMP_HOLD_FRAMES):
      out = dict(prev)
      out["dRel"] = d_hold
      _set_lead_speed(out, v_hold, v_ego)
      return out
  if _hold_candidate(lead, v_ego):
    _lead_hold[slot] = {"lead": dict(lead), "t": now, "n": 0}
  elif lead.get("status", False):
    _lead_hold.pop(slot, None)
  return lead


_VF_CLOSER_MAX_VEGO = 11.11
_VF_CLOSER_MIN_GAIN = 2.0
_VF_CLOSER_MAX_DREL = 15.0
_VF_CLOSER_MAX_LAT = 1.4
_VF_CLOSER_MIN_CLOSING = 0.5
_VF_CLOSER_MIN_CNT = 3
_VF_CLOSER_MIN_HOLD = 0.5
_VF_CLOSER_HYSTERESIS = 1.0
_VF_CLOSER_HIST_TTL = 2.0
_closer_streak: dict[int, tuple[float, float]] = {}
_closer_pick: dict[int, int] = {}


def _closer_track_rejected(track: Any, v_ego: float, pyo: float) -> bool:
  for name, call in (("is_oncoming_obstacle", (track, v_ego, pyo)),
                     ("is_crossing_traffic", (track, v_ego, pyo)),
                     ("is_ground_stationary_roadside", (track, v_ego, pyo)),
                     ("is_lateral_flyby", (track, pyo))):
    fn = getattr(_mod, name, None)
    if fn is None:
      continue
    try:
      if fn(*call):
        return True
    except (AttributeError, TypeError, ValueError):
      continue
  return False


def _apply_closer_in_path_lead(lead: dict[str, Any], bound: dict[str, Any],
                               v_ego: float, now: float) -> dict[str, Any]:
  tracks = bound.get("tracks")
  if not tracks or not lead.get("status", False) or v_ego > _VF_CLOSER_MAX_VEGO:
    return lead
  for tid, (seen, _) in list(_closer_streak.items()):
    if now - seen > _VF_CLOSER_HIST_TTL:
      del _closer_streak[tid]
  preceding = int(getattr(_mod, "ORIENT_PRECEEDING", 12))
  ok_lanes = (int(getattr(_mod, "LANE_UNKNOWN", 0)), int(getattr(_mod, "LANE_HOST", 3)))
  path_x, path_y = bound.get("path_x"), bound.get("path_y")
  path_ok = bool(bound.get("path_valid")) and path_x is not None and path_y is not None
  offset_fn = getattr(_mod, "get_path_lateral_offset", None)
  cutoff = float(lead["dRel"]) - _VF_CLOSER_MIN_GAIN
  best = None
  for track in tracks.values():
    try:
      d_rel = float(track.dRel)
      if d_rel >= min(cutoff, _VF_CLOSER_MAX_DREL) or float(track.vRel) > -_VF_CLOSER_MIN_CLOSING:
        continue
      if not getattr(track, "measured", False) or int(track.cnt) < _VF_CLOSER_MIN_CNT:
        continue
      if int(track.motionOrientation) != preceding or int(track.laneAssignment) not in ok_lanes:
        continue
      pyo = 0.0
      if path_ok and offset_fn is not None:
        pyo = float(offset_fn(d_rel, path_x, path_y, bound.get("path_y_std"), v_ego=v_ego)[0])
      if abs(float(track.yRel) - pyo) >= _VF_CLOSER_MAX_LAT:
        continue
      if _closer_track_rejected(track, v_ego, pyo):
        continue
      tid = int(getattr(track, "identifier", -1))
    except (AttributeError, IndexError, TypeError, ValueError):
      continue
    seen, streak = _closer_streak.get(tid, (now, 0.0))
    streak = streak + (now - seen) if 0.0 < now - seen < 0.3 else 0.0
    _closer_streak[tid] = (now, streak)
    if streak < _VF_CLOSER_MIN_HOLD:
      continue
    margin = 0.0 if tid == _closer_pick.get(0) else _VF_CLOSER_HYSTERESIS
    if best is None or d_rel + margin < best[0]:
      best = (d_rel + margin, track, tid)
  if best is None:
    _closer_pick.pop(0, None)
    return lead
  _, track, tid = best
  _closer_pick[0] = tid
  swapped = dict(track.get_RadarState(0.0, v_ego))
  swapped["fcw"] = False
  return swapped


_infocan_hold = InfoCanLeadHold()
_orig_get_lead = getattr(_mod, "get_lead", None)
if _orig_get_lead is not None:
  _GET_LEAD_ARGS = _orig_get_lead.__code__.co_varnames[:_orig_get_lead.__code__.co_argcount]

  def get_lead(*args, **kwargs):
    lead = _orig_get_lead(*args, **kwargs)
    bound = dict(zip(_GET_LEAD_ARGS, args, strict=False))
    bound.update(kwargs)
    v_ego = float(bound.get("v_ego", 0.0))
    if v_ego <= _VF_GUARD_MAX_VEGO:
      now = time.monotonic()
      slot = 0 if bound.get("low_speed_override", True) else 1
      if slot == 0:
        lead = _apply_closer_in_path_lead(lead, bound, v_ego, now)
      if lead.get("status", False) and lead.get("radar", False):
        lead = dict(lead)
        lead = _apply_vision_speed_guard(lead, bound.get("lead_msg"), v_ego,
                                         float(bound.get("model_v_ego", v_ego)))
        lead = _apply_range_rate_guard(lead, v_ego, now)
      lead = _apply_lead_hold(lead, slot, v_ego, now)
    tracks = bound.get("tracks")
    if tracks is not None:
      return _infocan_hold.update(lead, tracks, v_ego)
    return lead

  _mod.get_lead = get_lead

sys.modules[__name__] = _mod
