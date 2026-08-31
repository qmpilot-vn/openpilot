"""_BYTECODE_SHIM — logic is loaded from __pycache__/radard_impl.*.pyc (same Python minor as build).

VF6/VF7 InfoCAN post-load patches:
* red-light stick-slip: clamp near-stopped leads so ego speed does not ping-pong
* urban following: position-only aLeadK/vLead spikes look like emergency braking
  (seg11 ~21.7s: aLeadK pegged at -3.5 while vision still ~7 m/s)

VF8/VF9 + PMV2 highway post-load patches:
* radar-only / candidate range and path corridor were too tight above ~100 km/h
* |vRel| and static-lead caps dropped slowing/stopped in-path cars until close
"""
import importlib.util
import sys
from pathlib import Path
from typing import Any

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

# VF7 InfoCAN: offset motorbikes in the host lane at red lights (no-op if absent).
if hasattr(_mod, "INFO_HOST_CREEP_MAX_LAT"):
  _mod.INFO_HOST_CREEP_MAX_LAT = 2.0
if hasattr(_mod, "INFO_HOST_MIN_TRACK_CNT"):
  _mod.INFO_HOST_MIN_TRACK_CNT = 2

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

sys.modules[__name__] = _mod
