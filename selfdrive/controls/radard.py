#!/usr/bin/env python3
"""VinFast radar–vision fusion daemon (radard_new).

Takes ``liveTracks`` (raw radar points from ``radar_interface_new``) and
``modelV2`` (vision model) and produces ``radarState`` with up to two fused
leads for the longitudinal planner.

Design philosophy
-----------------
* **Radar-trusted**: radar data that passed RadarInterface filters is reliable.
  Fusion prefers radar+vision, but trusts radar-only when vision is weak.
* **Speed-dependent safety**: implausibly close leads are rejected based on ego
  speed (``min_lead_distance``).
* **Lead hysteresis**: prevents lead-switch jitter when two tracks score
  similarly by requiring a margin before swapping.
"""
import math
import numpy as np
from collections import deque
from typing import Any

import capnp
from cereal import messaging, log, car
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL, Priority, config_realtime_process
from openpilot.common.swaglog import cloudlog
from openpilot.common.simple_kalman import KF1D


# ── Kalman / physics constants ────────────────────────────────────────────────
_LEAD_ACCEL_TAU = 1.5          # acceleration decay: 50 % at 1 s
SPEED, ACCEL = 0, 1            # KF state indices
V_EGO_STATIONARY = 4.0         # [m/s] below this → no stationary-object flag
RADAR_TO_CENTER = 2.7          # (deprecated)
RADAR_TO_CAMERA = 1.52         # radar → camera mesh frame offset [m]

# ── Fusion thresholds (module-level for easy tuning) ──────────────────────────
# Radar-priority: fuse eagerly, trust radar-only at long range
VISION_MATCH_THRESHOLD = 0.20   # min vision prob for radar+vision fusion (lower = fuse more)
VISION_ONLY_THRESHOLD  = 0.55  # min vision prob for vision-only lead (higher = radar-only fires more)
DIST_MATCH_FACTOR      = 0.45  # vision-radar distance tolerance factor (wider at range)
DIST_MATCH_MIN         = 7.0   # [m]  minimum distance tolerance
VEL_MATCH_REL          = 2.0   # [m/s] relative velocity tolerance (tight)
VEL_MATCH_ABS          = 15.0  # [m/s] absolute velocity tolerance (loose)
LAT_MATCH_FACTOR       = 3.0   # vision yStd multiplier for lateral tolerance
LAT_MATCH_MIN          = 2.5   # [m]  minimum lateral tolerance
RADAR_ONLY_MAX_DIST    = 100.0 # [m]  max distance for radar-only lead (was 30, now long-range)
RADAR_ONLY_MAX_LAT     = 2.1   # [m]  max |yRel| for radar-only lead (wider for bikes/motos)
STATIC_VEL_THRESHOLD   = 0.3   # [m/s] vision speed → static
STATIC_RADAR_VEL_MIN   = 5.0   # [m/s] radar speed above this triggers override
LEAD_HYSTERESIS_MARGIN = 0.15  # risk-score margin to switch leads (prevents jitter)
MAX_LEAD_DIST          = 120.0 # [m]  reject tracks beyond this
MAX_STATIC_LEAD_DIST   = 50.0  # [m]  reject static tracks beyond this
MAX_VREL_FILTER        = 25.0  # [m/s] reject high-speed tracks beyond 20 m (was 15)

# ── Path-relative lateral filtering ────────────────────────────────────────────
# Use modelV2.position to project radar tracks onto the driving path.
# On curves, a parked car on the roadside may have small straight-line yRel
# but large path-relative offset — this catches that.
PATH_MIN_VEGO       = 0.0    # [m/s] keep path gating active at low speed
PATH_MIN_RANGE      = 5.0    # [m]   model path unreliable at very close range
PATH_FALLBACK_YSTD  = 2.0    # [m]   if model yStd exceeds this, widen tolerance
# Distance-dependent tolerance: tight near (path accurate), wide far (path uncertain)
PATH_TOL_BP         = [5.0, 30.0, 60.0, 120.0]   # [m] distance breakpoints
PATH_TOL_V          = [1.2, 1.5,  2.5,  3.5]      # [m] lateral tolerance at each BP
CLOSE_RANGE_CUTIN_TOL = 2.0  # [m] widened lane corridor for close, fast-approaching cut-ins

# ── Speed-dependent minimum lead distance ─────────────────────────────────────
# At very low speed radar is noisy (multipath, bumper reflections, ground
# clutter).  Require a larger minimum distance to reject these.
LOW_SPEED_CUTOFF = 3       # [m/s] ≈ 5 km/h — below this, tighten filters
HOST_PRECEDING_PRIORITY_VEGO = 35.0 / 3.6  # [m/s] prioritize in-lane preceding tracks above 35 km/h
LOW_SPEED_MIN_TRACK_CNT = 6  # frames (~0.5 s) — new tracks ignored at low speed
CREEP_CLOSEST_LEAD_VEGO = 8.0 / 3.6  # [m/s] force-closest lead selection below 8 km/h
LOW_SPEED_PATH_MAX_LAT = 0.9         # [m] stricter path gate below 8 km/h
CREEP_LEAD_MAX_DREL = 18.0           # [m] focus on nearby lead while creeping
CREEP_LEAD_MIN_TRACK_CNT = 3         # frames
CREEP_LEAD_MAX_YVREL = 1.0           # [m/s] reject strong lateral movers
CREEP_LEAD_MIN_VLEADK = -0.3         # [m/s] reject opposite-direction objects

# FCW guardrails to avoid early warnings in dense low-speed traffic
FCW_MIN_VEGO = 4.0         # [m/s] ~14.4 km/h
FCW_MIN_CLOSING = 1.5      # [m/s] require meaningful closing speed
FCW_MAX_TTC = 2.2          # [s] warn only for near-term collision risk
FCW_MIN_MODEL_PROB = 0.95  # keep FCW conservative

# Short lead-stability hold: newly appeared radar leads are tempered for the
# first few frames to avoid one-frame spikes causing hard decel commands.
LEAD_STABILITY_HOLD_FRAMES = 10
LEAD_STABILITY_MIN_DREL = 3.5   # [m] don't hold very-close obstacles
LEAD_STABILITY_BYPASS_TTC = 1.0 # [s] bypass hold only for truly imminent collision risk
LEAD_HOLD_MIN_VREL = -2.5       # [m/s] clamp very negative closing speed
LEAD_HOLD_MIN_ALEADK = -0.8     # [m/s²] clamp strong negative accel estimate
LEAD_HOLD_SIDE_YREL = 0.7       # [m] stricter clamp for laterally offset new leads
LEAD_HOLD_SIDE_MIN_VREL = -1.2  # [m/s] side-object clamp to avoid hard brake snap
LEAD_HOLD_SIDE_MIN_ALEADK = -0.3  # [m/s²] side-object accel clamp

# Near-field stability gate: close radar leads must persist for a few frames
# before being accepted, unless collision is truly imminent.
NEAR_FIELD_STABLE_DREL = 4.0    # [m]
NEAR_FIELD_MIN_TRACK_CNT = 3    # frames
NEAR_FIELD_BYPASS_TTC = 0.9     # [s]

# Global radar-lead stability gate: require persistence before a radar track can
# become an active lead, except for imminent collision cases.
RADAR_LEAD_MIN_TRACK_CNT = 3
RADAR_LEAD_MIN_TRACK_CNT_LOW_SPEED = 4
RADAR_LEAD_BYPASS_TTC = 1.0
RADAR_LEAD_BYPASS_DREL = 8.0

# Low-speed lead smoothing to suppress noisy radar relative velocity spikes.
LOW_SPEED_LEAD_SMOOTH_VEGO = 20.0 / 3.6  # [m/s]
LOW_SPEED_LEAD_MIN_VREL = -2.5           # [m/s] softer cap to avoid hard brake snaps below 20 km/h
LOW_SPEED_LEAD_MIN_ALEADK = -0.8         # [m/s²] softer decel estimate floor at low speed
LOW_SPEED_LEAD_MAX_ALEADK = 1.5          # [m/s²]

# Side-pass / fly-by rejection using predicted miss distance.
SIDE_PASS_MIN_DREL = 2.0
SIDE_PASS_MAX_DREL = 20.0
SIDE_PASS_MIN_LAT = 0.6
SIDE_PASS_MIN_YVREL = 0.5
SIDE_PASS_MAX_TTC = 2.5
SIDE_PASS_MISS_LAT = 0.9

# User-requested hard reject: at >10 km/h, do not follow slow pedestrian-like
# preceding side leads that can momentarily collapse into near-path radar ghosts.
HARD_PED_REJECT_VEGO = 10.0 / 3.6   # [m/s]
HARD_PED_REJECT_MAX_VLEAD = 3.0      # [m/s] ~11 km/h (ped-like speed)
HARD_PED_REJECT_MAX_LAT = 1.2        # [m]
SLOW_VRU_REJECT_MAX_DREL = 22.0      # [m] nearby slow VRU pass-by zone
SLOW_VRU_REJECT_MAX_VLEAD = 3.2      # [m/s] slow VRU-like absolute speed
SLOW_VRU_REJECT_MAX_LAT = 1.3        # [m] around planned path center
SLOW_VRU_REJECT_MIN_YVREL = 0.15     # [m/s] must show lateral motion

# Ego-overtaking pass-by rejection: when ego closes fast on a same-direction
# side object whose yRel was recently well outside the lane, the current
# small yRel is likely a radar measurement artifact (beam reflects off
# nearest edge at close range), not a genuine lane entry.
OVERTAKE_PASS_MIN_VREL = -2.5      # [m/s] minimum closing speed to consider
OVERTAKE_PASS_MAX_DREL = 25.0      # [m] within passing range
OVERTAKE_PASS_HIST_LAT = 1.0       # [m] track must have been outside this recently
OVERTAKE_PASS_CUR_LAT = 0.4        # [m] current path-relative offset still nonzero
OVERTAKE_PASS_YREL_DECAY = 0.97    # per-frame decay for max |yRel| tracker

# Lateral fly-by (artifact) rejection:
# signature = historically side-offset track that suddenly collapses near path
# center while ego is closing. This targets "yRel suddenly drops low" ghosts.
LATERAL_FLYBY_MIN_DREL = 6.0
LATERAL_FLYBY_MAX_DREL = 70.0
LATERAL_FLYBY_MIN_CLOSING = 3.5         # [m/s]
LATERAL_FLYBY_MIN_TRACK_CNT = 6         # need more history to detect collapse robustly
LATERAL_FLYBY_HIST_MIN_LAT = 1.2        # [m] track was recently side-offset
LATERAL_FLYBY_STRONG_HIST_LAT = 1.8     # [m] strong side-evidence even if DBC noisy
LATERAL_FLYBY_CUR_MAX_LAT = 0.35        # [m] currently appears near path center
LATERAL_FLYBY_COLLAPSE_RATIO_MAX = 0.35 # current lat / recent max lat
LATERAL_FLYBY_MIN_CENTERING_YV = 0.35   # [m/s] keep likely true cut-ins
LATERAL_FLYBY_CUTIN_PROTECT_DREL = 22.0 # [m]
LATERAL_FLYBY_MAX_YVREL_FOR_REJECT = 0.8  # [m/s] avoid suppressing active merge dynamics

# ── DBC motion classification enums (mirrored from radar_interface) ───────────
# Motion_Status
MSTATUS_INVALID       = 0
MSTATUS_UNKNOWN       = 1
MSTATUS_MOVING        = 2
MSTATUS_STATIONARY    = 3
MSTATUS_STOPPED       = 4
MSTATUS_MOVING_SLOWLY = 5

# Motion_Orientation
ORIENT_INVALID        = 0
ORIENT_DRIFTING_RIGHT = 1
ORIENT_CROSSING_RIGHT = 3
ORIENT_OC_DRIFT_RIGHT = 5
ORIENT_ONCOMING       = 6
ORIENT_OC_DRIFT_LEFT  = 7
ORIENT_CROSSING_LEFT  = 9
ORIENT_DRIFTING_LEFT  = 11
ORIENT_PRECEEDING     = 12
ORIENT_UNKNOWN_VAL    = 13

CROSSING_ORIENTATIONS = {ORIENT_CROSSING_LEFT, ORIENT_CROSSING_RIGHT}
ONCOMING_ORIENTATIONS = {ORIENT_ONCOMING, ORIENT_OC_DRIFT_LEFT, ORIENT_OC_DRIFT_RIGHT}
SAME_DIR_ORIENTATIONS = {ORIENT_PRECEEDING, ORIENT_DRIFTING_LEFT, ORIENT_DRIFTING_RIGHT}
DRIFTING_ORIENTATIONS = {ORIENT_DRIFTING_LEFT, ORIENT_DRIFTING_RIGHT}

# Lane_Assignment
LANE_UNKNOWN     = 0
LANE_LEFT_LEFT   = 1
LANE_LEFT        = 2
LANE_HOST        = 3
LANE_RIGHT       = 4
LANE_RIGHT_RIGHT = 5
ADJACENT_LANES   = {LANE_LEFT_LEFT, LANE_LEFT, LANE_RIGHT, LANE_RIGHT_RIGHT}

def min_lead_distance(v_ego: float) -> float:
  """Minimum acceptable dRel given ego speed.

  At higher speeds a lead at 1–3 m is physically implausible
  (TTC < 0.3 s → already in contact).
  At very low speed (< 5 km/h) radar produces ground clutter / multipath
  noise, but valid leads in dense traffic can still be very close.
  """
  if v_ego < 1.5:
    return 1.2
  if v_ego < LOW_SPEED_CUTOFF:
    return 2.5
  return max(2.0, v_ego * 0.3)


def get_path_lateral_offset(d_rel: float, path_x: np.ndarray, path_y: np.ndarray,
                            path_y_std: np.ndarray | None = None) -> tuple[float, float]:
  """Compute the driving-path lateral offset at longitudinal distance ``d_rel``.

  ``modelV2.position`` is time-indexed: ``.x`` = predicted longitudinal position,
  ``.y`` = predicted lateral position (ISO: positive = left).
  Radar ``yRel`` uses openpilot convention (positive = right), so we negate
  the interpolated path_y.

  Returns:
    (path_y_at_d, tolerance)
      path_y_at_d: path lateral offset in openpilot frame (positive = right)
      tolerance:   distance-dependent + uncertainty-scaled lateral tolerance [m]
  """
  if len(path_x) < 2 or d_rel < PATH_MIN_RANGE or d_rel > float(path_x[-1]):
    return 0.0, float(np.interp(d_rel, PATH_TOL_BP, PATH_TOL_V))

  path_y_at_d = -float(np.interp(d_rel, path_x, path_y))

  base_tol = float(np.interp(d_rel, PATH_TOL_BP, PATH_TOL_V))

  if path_y_std is not None and len(path_y_std) == len(path_x):
    y_std = float(np.interp(d_rel, path_x, path_y_std))
    if y_std > PATH_FALLBACK_YSTD:
      base_tol = max(base_tol, y_std * 1.5)

  return path_y_at_d, base_tol


def is_track_in_planning_path(track: "Track",
                              path_x: np.ndarray | None = None,
                              path_y: np.ndarray | None = None,
                              path_y_std: np.ndarray | None = None,
                              path_valid: bool = False) -> bool:
  """True when a radar track lies inside the model driving-path corridor.

  If path is invalid/unavailable, fall back to straight-line lateral gating.
  """
  if path_valid and path_x is not None and path_y is not None:
    path_y_at_d, path_tol = get_path_lateral_offset(track.dRel, path_x, path_y, path_y_std)
    lat_err = abs(track.yRel - path_y_at_d)
    tol = min(RADAR_ONLY_MAX_LAT, path_tol)
    # Intersection/cut-in hazard allowance: if object is close and longitudinally
    # approaching, widen corridor a bit so we don't miss true collision targets.
    if track.dRel < 15.0 and track.vRel < -1.0:
      tol = max(tol, CLOSE_RANGE_CUTIN_TOL)
    return lat_err < tol
  fallback_tol = RADAR_ONLY_MAX_LAT
  if track.dRel < 15.0 and track.vRel < -1.0:
    fallback_tol = max(fallback_tol, CLOSE_RANGE_CUTIN_TOL)
  return abs(track.yRel) < fallback_tol


def should_trigger_fcw(track: "Track", v_ego: float, model_prob: float) -> bool:
  """Conservative FCW gate to reduce early low-speed warnings."""
  if model_prob < FCW_MIN_MODEL_PROB:
    return False
  if v_ego < FCW_MIN_VEGO:
    return False
  closing_speed = -float(track.vRel)
  if closing_speed < FCW_MIN_CLOSING:
    return False
  ttc = float(track.dRel) / max(closing_speed, 0.1)
  return ttc < FCW_MAX_TTC


def apply_lead_stability_hold(lead_dict: dict[str, Any],
                              track: "Track",
                              v_ego: float) -> dict[str, Any]:
  """Temper new radar leads briefly to avoid harsh one-frame braking."""
  if (not lead_dict.get('status', False)) or (not lead_dict.get('radar', False)):
    return lead_dict
  if track.cnt >= LEAD_STABILITY_HOLD_FRAMES:
    return lead_dict

  d_rel = float(lead_dict.get('dRel', 0.0))
  v_rel = float(lead_dict.get('vRel', 0.0))
  closing_speed = -v_rel
  ttc = d_rel / max(closing_speed, 0.1)

  # Do not suppress urgent braking for truly imminent hazards.
  if d_rel <= LEAD_STABILITY_MIN_DREL or ttc <= LEAD_STABILITY_BYPASS_TTC:
    return lead_dict

  # New side-offset leads are more likely crossing/adjacent objects; temper
  # initial decel harder unless TTC is truly imminent (handled above).
  v_rel_floor = LEAD_HOLD_SIDE_MIN_VREL if abs(track.yRel) > LEAD_HOLD_SIDE_YREL else LEAD_HOLD_MIN_VREL
  if v_rel < v_rel_floor:
    v_rel = v_rel_floor
    lead_dict['vRel'] = v_rel
    lead_dict['vLead'] = v_ego + v_rel
    lead_dict['vLeadK'] = v_ego + v_rel

  a_lead_k = float(lead_dict.get('aLeadK', 0.0))
  a_floor = LEAD_HOLD_SIDE_MIN_ALEADK if abs(track.yRel) > LEAD_HOLD_SIDE_YREL else LEAD_HOLD_MIN_ALEADK
  if a_lead_k < a_floor:
    lead_dict['aLeadK'] = a_floor

  return lead_dict


def stabilize_low_speed_radar_lead(lead_dict: dict[str, Any],
                                   track: "Track",
                                   v_ego: float) -> dict[str, Any]:
  """Stabilize low-speed radar lead kinematics using Kalman speed estimate."""
  if (not lead_dict.get('status', False)) or (not lead_dict.get('radar', False)):
    return lead_dict
  if v_ego >= LOW_SPEED_LEAD_SMOOTH_VEGO:
    return lead_dict

  # Prefer smooth Kalman absolute speed for low-speed following.
  vrel_smooth = float(track.vLeadK - v_ego)
  vrel_raw = float(lead_dict.get('vRel', vrel_smooth))
  alpha = 0.8  # heavy weight to smooth estimate at low speed
  vrel_out = alpha * vrel_smooth + (1.0 - alpha) * vrel_raw
  vrel_floor = LOW_SPEED_LEAD_MIN_VREL
  # Extra protection against sensitive side/crossing objects at low speed:
  # keep longitudinal closing estimate gentle unless geometry is clearly in-lane.
  if track.dRel < 15.0 and abs(track.yRel) > 0.5:
    vrel_floor = max(vrel_floor, -1.5)
  if track.motionOrientation in CROSSING_ORIENTATIONS:
    vrel_floor = max(vrel_floor, -1.2)
  vrel_out = max(vrel_floor, vrel_out)

  lead_dict['vRel'] = vrel_out
  lead_dict['vLead'] = v_ego + vrel_out
  lead_dict['vLeadK'] = float(track.vLeadK)

  a_out = float(lead_dict.get('aLeadK', 0.0))
  a_floor = LOW_SPEED_LEAD_MIN_ALEADK
  if track.dRel < 15.0 and abs(track.yRel) > 0.5:
    a_floor = max(a_floor, -0.5)
  if track.motionOrientation in CROSSING_ORIENTATIONS:
    a_floor = max(a_floor, -0.4)
  lead_dict['aLeadK'] = min(LOW_SPEED_LEAD_MAX_ALEADK, max(a_floor, a_out))
  return lead_dict


def stabilize_position_only_radar_lead(lead_dict: dict[str, Any],
                                       track: "Track",
                                       v_ego: float) -> dict[str, Any]:
  """Use Kalman absolute speed when radar only reports distance (VF6 InfoCAN)."""
  if (not lead_dict.get('status', False)) or (not lead_dict.get('radar', False)):
    return lead_dict
  if track.measured or track.cnt < 3:
    return lead_dict

  vrel_k = float(track.vLeadK - v_ego)
  vrel_raw = float(lead_dict.get('vRel', vrel_k))
  alpha = 0.8
  vrel_out = alpha * vrel_k + (1.0 - alpha) * vrel_raw
  lead_dict['vRel'] = vrel_out
  lead_dict['vLead'] = v_ego + vrel_out
  lead_dict['vLeadK'] = float(track.vLeadK)
  return lead_dict


class KalmanParams:
  def __init__(self, dt: float):
    # Lead Kalman Filter params, calculating K from A, C, Q, R requires the control library.
    # hardcoding a lookup table to compute K for values of radar_ts between 0.01s and 0.2s
    assert dt > .01 and dt < .2, "Radar time step must be between .01s and 0.2s"
    self.A = [[1.0, dt], [0.0, 1.0]]
    self.C = [1.0, 0.0]
    #Q = np.matrix([[10., 0.0], [0.0, 100.]])
    #R = 1e3
    #K = np.matrix([[ 0.05705578], [ 0.03073241]])
    dts = [i * 0.01 for i in range(1, 21)]
    K0 = [0.12287673, 0.14556536, 0.16522756, 0.18281627, 0.1988689,  0.21372394,
          0.22761098, 0.24069424, 0.253096,   0.26491023, 0.27621103, 0.28705801,
          0.29750003, 0.30757767, 0.31732515, 0.32677158, 0.33594201, 0.34485814,
          0.35353899, 0.36200124]
    K1 = [0.29666309, 0.29330885, 0.29042818, 0.28787125, 0.28555364, 0.28342219,
          0.28144091, 0.27958406, 0.27783249, 0.27617149, 0.27458948, 0.27307714,
          0.27162685, 0.27023228, 0.26888809, 0.26758976, 0.26633338, 0.26511557,
          0.26393339, 0.26278425]
    self.K = [[np.interp(dt, dts, K0)], [np.interp(dt, dts, K1)]]


class Track:
  """Kalman-filtered radar track.

  Wraps a 1-D Kalman filter on ``vLead`` (absolute lead speed) and exposes
  smoothed ``vLeadK`` / ``aLeadK``.  Also tracks ``yRel`` history to estimate
  lateral velocity (``yvRel``) for crossing-traffic detection.
  """

  def __init__(self, identifier: int, v_lead: float, kalman_params: KalmanParams):
    self.identifier = identifier
    self.cnt = 0
    self.aLeadTau = FirstOrderFilter(_LEAD_ACCEL_TAU, 0.45, DT_MDL)
    self.K_A = kalman_params.A
    self.K_C = kalman_params.C
    self.K_K = kalman_params.K
    self.kf = KF1D([[v_lead], [0.0]], self.K_A, self.K_C, self.K_K)

    # lateral velocity estimation from yRel history
    self._prev_yRel: float | None = None
    self.yvRel: float = 0.0       # estimated lateral velocity [m/s]

    # Recent maximum |yRel| with slow decay — detects radar yRel collapse
    # artifacts when ego overtakes a side object at close range.
    self.max_recent_abs_yRel: float = 0.0

    # DBC motion classification from radar hardware (0 = invalid/unavailable)
    self.motionStatus: int = MSTATUS_INVALID
    self.motionOrientation: int = ORIENT_INVALID
    self.laneAssignment: int = LANE_UNKNOWN

  def update(self, d_rel: float, y_rel: float, v_rel: float, v_lead: float,
             measured: float, yv_rel_meas: float | None = None,
             motion_status: int = 0, motion_orientation: int = 0,
             lane_assignment: int = 0):
    self.dRel = d_rel
    self.yRel = y_rel
    self.vRel = v_rel

    if abs(y_rel) > self.max_recent_abs_yRel:
      self.max_recent_abs_yRel = abs(y_rel)
    else:
      self.max_recent_abs_yRel = max(abs(y_rel),
                                     self.max_recent_abs_yRel * OVERTAKE_PASS_YREL_DECAY)
    self.vLead = v_lead
    self.measured = measured

    self.motionStatus = motion_status
    self.motionOrientation = motion_orientation
    self.laneAssignment = lane_assignment

    # Estimate lateral velocity: blend radar-reported yvRel (if available)
    # with ΔyRel/DT_MDL to improve crossing/cut-in discrimination.
    if self._prev_yRel is not None:
      raw_yvRel = (y_rel - self._prev_yRel) / DT_MDL
      if yv_rel_meas is not None and math.isfinite(yv_rel_meas):
        raw_yvRel = 0.6 * float(yv_rel_meas) + 0.4 * raw_yvRel
      # Faster response for new tracks (first 5 frames): 0.6 new / 0.4 old
      # After that, smoother: 0.4 new / 0.6 old
      # This lets us detect crossing motorbikes within 2-3 frames (~0.1s)
      alpha = 0.6 if self.cnt < 5 else 0.4
      self.yvRel = alpha * raw_yvRel + (1.0 - alpha) * self.yvRel
    elif yv_rel_meas is not None and math.isfinite(yv_rel_meas):
      self.yvRel = float(yv_rel_meas)
    self._prev_yRel = y_rel

    # Kalman update on absolute lead speed
    if self.cnt > 0:
      self.kf.update(self.vLead)

    self.vLeadK = float(self.kf.x[SPEED][0])
    self.aLeadK = float(self.kf.x[ACCEL][0])

    # Guard nan from Kalman (shouldn't happen, but defensive)
    if not math.isfinite(self.vLeadK):
      self.vLeadK = v_lead
    if not math.isfinite(self.aLeadK):
      self.aLeadK = 0.0

    if abs(self.aLeadK) < 0.5:
      self.aLeadTau.x = _LEAD_ACCEL_TAU
    else:
      self.aLeadTau.update(0.0)

    self.cnt += 1

  def get_RadarState(self, model_prob: float = 0.0, v_ego: float = 0.0) -> dict[str, Any]:
    radar_fcw_allowed = v_ego >= (30.0 / 3.6)
    return {
      "dRel": float(self.dRel),
      "yRel": float(self.yRel),
      "vRel": float(self.vRel),
      "vLead": float(self.vLead),
      "vLeadK": float(self.vLeadK),
      "aLeadK": float(self.aLeadK),
      "aLeadTau": float(self.aLeadTau.x),
      "status": True,
      # Radar leads: suppress FCW below 30 km/h, allow normal FCW above.
      "fcw": radar_fcw_allowed and self.is_potential_fcw(model_prob),
      "modelProb": model_prob,
      "radar": True,
      "radarTrackId": self.identifier,
    }

  def potential_low_speed_lead(self, v_ego: float) -> bool:
    """True if track is a plausible stopped-car lead at low ego speed.
    Excludes objects with significant lateral velocity (crossing pedestrians/bikes).
    Excludes objects clearly receding (overtaking motorbike from behind).
    Excludes ground-stationary offset objects (parked cars on roadside).
    At very low speed (< 5 km/h), requires more persistence and tighter lateral."""
    if self.vRel > 1.0:
      return False
    # Ground-stationary + offset → parked, not a lead
    if self.vLeadK < 1.0 and v_ego > 1.5 and abs(self.yRel) > 0.8:
      return False
    if v_ego < LOW_SPEED_CUTOFF:
      return (abs(self.yRel) < 1.0 and
              abs(self.yvRel) < 0.5 and
              self.cnt >= LOW_SPEED_MIN_TRACK_CNT and
              (1.2 < self.dRel < 20))
    return (abs(self.yRel) < 1.5 and
            abs(self.yvRel) < 0.8 and
            (v_ego < V_EGO_STATIONARY) and
            (2.0 < self.dRel < 25))

  def is_potential_fcw(self, model_prob: float) -> bool:
    return model_prob > .9

  def __repr__(self):
    return (f"Track(id={self.identifier} x={self.dRel:.1f} y={self.yRel:+.1f} "
            f"v={self.vRel:+.1f} yvRel={self.yvRel:+.1f})")


def calculate_collision_risk(track: Track, v_ego: float,
                             path_y_offset: float = 0.0) -> float:
  """Calculate collision risk score for a track — relaxed to avoid hard braking
  on crossing pedestrians, motorbikes, and cut-ins.

  Higher score = higher priority for lead selection.
  Heavily favours objects that are:
    - Dead-centre on the driving path (small path-relative lateral offset)
    - Staying in our lane (small |yvRel|)
    - Genuinely ahead and closing slowly (high TTC)
  Objects that are lateral, crossing, or only briefly in our path score low.

  ``path_y_offset`` is the driving-path lateral position in openpilot frame
  at this track's dRel (positive = right).  On a straight road this is ~0.
  """
  lat_from_path = abs(track.yRel - path_y_offset)

  # ── Lateral position ──────────────────────────────────────────────────────
  # Tight lane: only objects within ~1.5m of path centre score well
  # Beyond 1.5m the score drops sharply — they're likely in another lane
  lateral_score = max(0, 1.0 - (lat_from_path / 2.5) ** 2)  # quadratic falloff, 0 at 2.5m

  # ── Distance ──────────────────────────────────────────────────────────────
  # Gentle curve: don't over-prioritise close objects (avoids snapping to a
  # pedestrian that briefly appears at 5m)
  distance_score = max(0, 1.0 - track.dRel / 150.0)  # 0 at 150m — gentle: don't penalise far radar leads

  # ── Closing speed ─────────────────────────────────────────────────────────
  closing_speed = -track.vRel   # positive = approaching
  is_static = abs(track.vRel) < 0.5

  if is_static and v_ego > 0.5:
    closing_speed = v_ego
    closing_score = min(1.0, closing_speed / 30.0)  # softer: normalize to 30 m/s (was 20)
  elif closing_speed > 0:
    closing_score = min(1.0, closing_speed / 30.0)
  else:
    closing_score = 0.0   # receding → not a threat

  # ── Time to collision (TTC) ───────────────────────────────────────────────
  # Only score high when TTC is very short (< 4s).  Longer TTC → negligible.
  if closing_speed > 0.5:
    ttc = track.dRel / closing_speed
    ttc_score = max(0, 1.0 - ttc / 4.0)  # 0 at 4s (was 10s — much more relaxed)
  else:
    ttc_score = 0.0

  # ── Lateral-velocity penalty ──────────────────────────────────────────────
  # Objects moving sideways quickly will clear our path — penalise heavily.
  # Threshold raised to 0.8 m/s: bicycle wobble (~0.3–0.5 m/s) should NOT
  # reduce score.  Only penalise when lateral motion is clearly dominant.
  lateral_vel_penalty = 1.0
  if abs(track.yvRel) > 0.8:
    # At 1.5 m/s lateral → 0.65×, at 2.5 m/s → 0.30×, at 3.5+ m/s → ~0
    lateral_vel_penalty = max(0.02, 1.0 - (abs(track.yvRel) / 3.0) ** 1.5)

  # ── Combined score ────────────────────────────────────────────────────────
  # Lateral position dominates (50%) so off-centre objects rarely win
  risk_score = (
    0.50 * lateral_score +     # 50% — must be in our lane
    0.20 * distance_score +    # 20% — closer is riskier (gentle)
    0.15 * closing_score +     # 15% — approaching matters
    0.15 * ttc_score           # 15% — imminent collision matters
  ) * lateral_vel_penalty

  # Ground-stationary penalty: parked cars (vLeadK near 0 while ego moves)
  # that are offset from path are roadside objects, not in-lane obstacles.
  # Steeper falloff (1.2m instead of 1.5m) to strongly penalise side-road parking.
  if track.vLeadK < 1.0 and v_ego > 2.0 and lat_from_path > 0.5:
    park_penalty = max(0.02, 1.0 - (lat_from_path / 1.2) ** 2)
    risk_score *= park_penalty

  # Opposite-direction objects:
  # - At low speed (< 20 km/h), keep them eligible (milder penalty) so the
  #   car can behave cautiously in dense urban traffic.
  # - At higher speed, heavily penalise to avoid phantom lead snaps.
  if track.vLeadK < -1.0:
    is_low_speed = v_ego < 5.6
    if is_low_speed:
      oncoming_penalty = 0.8 if lat_from_path < 0.6 else 0.5
    else:
      oncoming_penalty = 0.05 if lat_from_path > 0.4 else 0.2
    risk_score *= oncoming_penalty

  # ── DBC-based score adjustments ──────────────────────────────────────────
  mo = track.motionOrientation
  la = track.laneAssignment
  has_valid_orient = mo not in (ORIENT_INVALID, ORIENT_UNKNOWN_VAL)
  has_valid_lane = la != LANE_UNKNOWN

  # Boost score for PRECEDING + HOST lane (hardware confirms real in-lane lead)
  if has_valid_orient and mo == ORIENT_PRECEEDING and has_valid_lane and la == LANE_HOST:
    # At higher ego speed, prioritize confirmed in-lane preceding targets more.
    if v_ego > HOST_PRECEDING_PRIORITY_VEGO:
      risk_score = max(risk_score, 0.72)
    else:
      risk_score = max(risk_score, 0.6)

  # Penalise crossing / oncoming / drifting objects that passed is_crossing_traffic
  # but still shouldn't score high in lead selection.
  # Crossing objects near our path (< 1.5m) are real hazards (truck/car at
  # intersection) — only penalise when clearly off to the side.
  if has_valid_orient and mo in CROSSING_ORIENTATIONS:
    if lat_from_path > 1.5:
      risk_score *= 0.3
    elif lat_from_path > 0.8:
      risk_score *= 0.7
  if has_valid_orient and mo in ONCOMING_ORIENTATIONS:
    risk_score *= 0.2
  if has_valid_orient and mo in DRIFTING_ORIENTATIONS and lat_from_path > 0.5:
    risk_score *= 0.4

  # Adjacent lane objects should score lower than host lane
  if has_valid_lane and la in ADJACENT_LANES:
    risk_score *= 0.3

  return risk_score


def is_crossing_traffic(track: Track, v_ego: float,
                        path_y_offset: float = 0.0) -> bool:
  """Detect crossing / overtaking traffic that should NOT trigger braking,
  while keeping genuine in-lane objects AND cut-ins as leads.

  Key distinction (uses sign of yRel_path × yvRel):
    - **Cut-in**:   yRel_path × yvRel < 0 → object moving TOWARD path (merging)
    - **Leaving**:  yRel_path × yvRel > 0 → object moving AWAY from path
    - **Crossing**: high |yvRel|, passing through our lane

  ``path_y_offset`` shifts the "centre" from straight-ahead to the model's
  predicted driving path.  On curves this prevents roadside objects from
  appearing in-lane.

  Returns True → track should be excluded from leads.
  """
  is_low_speed = v_ego < 5.6  # ~20 km/h — urban passing zone

  # Path-relative lateral offset (0 = dead on driving path)
  yRel_path = track.yRel - path_y_offset

  # ── DBC hardware classification — high-confidence early decisions ──────
  # The radar hardware's own tracker has Doppler + multi-frame context to
  # classify motion far more reliably than single-frame kinematics.
  # Use these as early returns when the signal is valid and unambiguous.
  mo = track.motionOrientation
  la = track.laneAssignment
  ms = track.motionStatus
  has_valid_orient = mo not in (ORIENT_INVALID, ORIENT_UNKNOWN_VAL)
  has_valid_lane = la != LANE_UNKNOWN
  slow_vru_like = (
    track.vLeadK < SLOW_VRU_REJECT_MAX_VLEAD and
    ms in (MSTATUS_MOVING_SLOWLY, MSTATUS_MOVING) and
    abs(track.yvRel) > SLOW_VRU_REJECT_MIN_YVREL
  )

  # DBC says CROSSING — only reject when clearly going to miss our path.
  # A car/truck crossing at an intersection near our lane is a real collision
  # hazard that MUST be kept as a lead.  Only reject when far enough lateral
  # (> 2.0m — outside lane width) AND TTC is long (not imminent).
  if has_valid_orient and mo in CROSSING_ORIENTATIONS:
    moving_toward_path = (yRel_path * track.yvRel) < 0.0 and abs(track.yvRel) > 0.2
    # Slow VRU-like crossing near our path is commonly a side pass artifact.
    if (slow_vru_like and 0.35 < abs(yRel_path) < SLOW_VRU_REJECT_MAX_LAT and
            track.dRel < SLOW_VRU_REJECT_MAX_DREL):
      return True
    closing_speed = max(-track.vRel, v_ego * 0.5)
    ttc = track.dRel / max(closing_speed, 0.1)
    if abs(yRel_path) > 2.0 and ttc > 3.0 and not moving_toward_path:
      return True

  # DBC says ONCOMING + laterally offset → definite oncoming traffic
  if has_valid_orient and mo in ONCOMING_ORIENTATIONS and abs(yRel_path) > 0.5:
    if not (is_low_speed and track.dRel < 12.0 and abs(yRel_path) < 1.0):
      return True

  # DBC says DRIFTING (same-dir, moving laterally) in adjacent lane → passing
  if has_valid_orient and mo in DRIFTING_ORIENTATIONS:
    if has_valid_lane and la in ADJACENT_LANES:
      return True
    if abs(yRel_path) > 1.1 and track.vRel > -0.8:
      return True

  # DBC says adjacent lane + same direction + laterally offset → not our lead
  if has_valid_lane and la in ADJACENT_LANES and has_valid_orient and mo in SAME_DIR_ORIENTATIONS:
    if abs(yRel_path) > 0.8 and track.vRel > -1.0:
      return True

  # DBC says PRECEDING + HOST lane → strong confidence it IS a real lead.
  # Skip the rest of the crossing heuristics to avoid false rejection.
  if has_valid_orient and mo == ORIENT_PRECEEDING and has_valid_lane and la == LANE_HOST:
    host_preceding_keep_lat = 1.5 if v_ego <= HOST_PRECEDING_PRIORITY_VEGO else 1.8
    # Exception: slow VRU-like "preceding" near path center should still be
    # eligible for rejection by downstream crossing/pass-by gates.
    if abs(yRel_path) < host_preceding_keep_lat and not slow_vru_like:
      return False

  # ── Kinematic-based detection (fallback when DBC is INVALID/UNKNOWN) ───

  # Oncoming if absolute lead speed is opposite direction (preferred) or
  # relative speed is far more negative than ego speed.
  is_oncoming = (track.vLeadK < -1.0) or (track.vRel < -(v_ego + 2.0))
  # Also trust DBC oncoming classification even if kinematics don't confirm yet
  if has_valid_orient and mo in ONCOMING_ORIENTATIONS:
    is_oncoming = True

  # ── Detect cut-in vs. passing ──────────────────────────────────────────
  # yRel_path × yvRel < 0 means the object is moving toward the driving path.
  # A bike cutting in from the right: yRel_path > 0, yvRel < 0 → product < 0.
  # Require some minimum lateral motion to be sure (not just noise).
  is_cutting_in = (yRel_path * track.yvRel < 0) and abs(track.yvRel) > 0.3

  # Same-direction overtaking reject (general):
  # A laterally offset object that is faster than ego is typically passing by
  # in an adjacent corridor and should not become a braking lead.
  # Keep cut-ins excluded from this filter so genuine merges still pass.
  speed_adv = track.vLeadK - v_ego
  pass_lat_thresh = 0.5 if v_ego < 5.0 else 0.7
  if (not is_cutting_in and not is_oncoming and track.dRel < 50.0 and
      abs(yRel_path) > pass_lat_thresh and speed_adv > 0.8 and track.vRel > -1.0):
    return True

  # A track we're genuinely catching up to is likely a real in-lane object.
  approach_thresh = -2.0 if is_low_speed else -1.0
  is_approaching = track.vRel < approach_thresh

  # Oncoming / opposite-direction objects:
  # - Low speed: keep as lead candidates when close and near path centre.
  # - Higher speed: filter aggressively (except dead-centre + very close).
  if is_oncoming:
    if is_low_speed and track.dRel < 15.0 and abs(yRel_path) < 1.2:
      return False
    if abs(yRel_path) > 0.4 or track.dRel > 12.0:
      return True

  # Close cut-in hazard: do not classify too early as crossing.
  # Keep this narrow to avoid promoting adjacent-lane crossing traffic.
  if (track.dRel < 12.0 and track.vRel < -0.5 and abs(yRel_path) < 1.2
      and (is_cutting_in or abs(track.yvRel) < 1.2 or (is_oncoming and is_low_speed))):
    return False

  # ── Criterion 1: lateral velocity + offset → crossing ──────────────────
  # SKIP for cut-ins: a bike at yRel=1.5 moving toward centre is merging,
  # not crossing.
  if not is_cutting_in:
    yvRel_thresh = 2.0 if is_approaching else (1.2 if is_low_speed else 1.5)
    yRel_thresh  = 1.2 if is_low_speed else 1.5
    if abs(track.yvRel) > yvRel_thresh and abs(yRel_path) > yRel_thresh:
      return True

  # ── Criterion 2: very high lateral velocity → sweeping across ──────────
  # Even a cut-in at > 3 m/s lateral is too fast to be a merge — it's
  # blasting through.  But raise threshold for cut-ins slightly.
  sweep_thresh = 4.0 if is_cutting_in else 3.0
  if abs(track.yvRel) > sweep_thresh:
    return True

  # ── Criterion 3: far lateral → not in our lane ─────────────────────────
  # Cut-ins start from the side, so give them more room.
  far_lat = (3.5 if is_cutting_in else 2.5) if is_low_speed else (4.0 if is_cutting_in else 3.0)
  if abs(yRel_path) > far_lat:
    return True

  # ── Criterion 4: lateral velocity dominates closing speed ──────────────
  # SKIP for cut-ins — their lateral speed IS expected to be high.
  if not is_approaching and not is_cutting_in:
    closing_speed = max(0.1, -track.vRel)
    ratio = 1.5 if is_low_speed else 2.0
    if abs(track.yvRel) > 0.8 and abs(track.yvRel) > closing_speed * ratio and abs(yRel_path) > 0.5:
      return True

  # ── Criterion 5: offset AND moving AWAY from centre → leaving ──────────
  # By definition, cut-ins have yRel × yvRel < 0 (toward centre), so this
  # criterion naturally doesn't fire for them.
  leaving_lat = 1.5 if is_low_speed else 2.0
  if abs(yRel_path) > leaving_lat and yRel_path * track.yvRel > 0:
    return True

  # ── Criterion 6: "stationary but offset" → likely passing motorbike ────
  # SKIP for cut-ins: a bike merging in may briefly show low vRel.
  if not is_cutting_in:
    static_vRel = 2.0 if is_low_speed else 1.0
    static_yRel = 1.5 if is_low_speed else 2.0
    if abs(track.vRel) < static_vRel and abs(yRel_path) > static_yRel and v_ego > 1.0:
      return True

  # ── Criterion 7: new track + offset → wait for velocity to settle ──────
  # For cut-ins, still require a few frames but use a shorter count.
  min_cnt = (5 if is_cutting_in else 8) if is_low_speed else (3 if is_cutting_in else 5)
  new_yRel = (1.8 if is_cutting_in else 1.2) if is_low_speed else (2.5 if is_cutting_in else 1.8)
  if track.cnt < min_cnt and abs(yRel_path) > new_yRel and not is_approaching:
    return True

  # ── Criterion 8: new "stationary" track while moving ───────────────────
  # SKIP for cut-ins: merging bike may briefly show low vRel.
  if not is_cutting_in:
    new_vRel = 2.0 if is_low_speed else 0.5
    min_cnt8 = 8 if is_low_speed else 5
    if track.cnt < min_cnt8 and abs(track.vRel) < new_vRel and v_ego > 1.0 and not is_approaching:
      return True

  # ── Criterion 9: same-direction overtaking / adjacent-lane traffic ────
  # A vehicle (typically motorbike) in a neighbouring lane or passing on
  # the side.  These are NOT braking targets.
  #
  # Two tiers:
  #   (a) Clearly offset (|yRel| > 1.2 m) — almost certainly adjacent
  #       lane; filter even if closing moderately (vRel > -3.0).
  #   (b) Slightly offset (|yRel| > 0.7 m) — could be squeezing past in
  #       our lane; require they aren't closing fast (vRel > -1.5).
  #
  # Skip for cut-ins (object converging toward lane centre).
  if not is_cutting_in:
    if abs(yRel_path) > 1.2 and track.vRel > -3.0:
      return True
    overtake_yRel = 0.7 if is_low_speed else 0.8
    overtake_vRel = -0.5 if is_low_speed else -1.5
    if abs(yRel_path) > overtake_yRel and track.vRel > overtake_vRel:
      return True

  # ── Criterion 10: ground-stationary roadside object (parked car) ──────
  # Objects with near-zero absolute speed (Kalman vLeadK) that are
  # laterally offset are parked cars, road furniture, guardrails on the
  # side of the road.  The lateral planner handles avoidance — radar
  # should not lock onto them as leads to avoid phantom braking.
  # Uses vLeadK (Kalman-filtered absolute speed) which is far more
  # reliable than vRel for detecting ground-stationary objects.
  if not is_cutting_in and v_ego > 2.0:
    if track.vLeadK < 1.0 and abs(yRel_path) > 1.0:
      return True

  # ── Criterion 11: ego overtaking slower same-direction side object ────
  # When ego closes fast on a same-direction object that was recently well
  # outside the driving path, radar yRel can collapse at close range (the
  # beam reflects off the nearest edge of the object, compressing the
  # apparent lateral offset).  This mimics a sudden lane entry but is a
  # measurement artifact.  max_recent_abs_yRel confirms the track was
  # recently far from path centre — genuine in-lane obstacles have
  # consistently small yRel history and will NOT trigger this criterion.
  if not is_oncoming and track.vLeadK > 0 and track.vRel < OVERTAKE_PASS_MIN_VREL:
    if (track.dRel < OVERTAKE_PASS_MAX_DREL and
        track.max_recent_abs_yRel > OVERTAKE_PASS_HIST_LAT and
        abs(yRel_path) > OVERTAKE_PASS_CUR_LAT):
      return True

  return False


def is_lateral_flyby(track: Track, path_y_offset: float = 0.0) -> bool:
  """Reject side-track lateral-collapse artifacts while preserving cut-ins."""
  y_rel_path = track.yRel - path_y_offset
  closing_speed = -track.vRel
  if (track.cnt < LATERAL_FLYBY_MIN_TRACK_CNT or
      not (LATERAL_FLYBY_MIN_DREL < track.dRel < LATERAL_FLYBY_MAX_DREL)):
    return False
  if closing_speed < LATERAL_FLYBY_MIN_CLOSING:
    return False

  abs_lat = abs(y_rel_path)
  hist_lat = max(abs(track.yRel), float(track.max_recent_abs_yRel))
  if hist_lat < LATERAL_FLYBY_HIST_MIN_LAT:
    return False
  if abs_lat > LATERAL_FLYBY_CUR_MAX_LAT:
    return False
  if abs_lat / max(hist_lat, 1e-3) > LATERAL_FLYBY_COLLAPSE_RATIO_MAX:
    return False

  moving_toward_center = (
    abs(track.yvRel) >= LATERAL_FLYBY_MIN_CENTERING_YV and
    (y_rel_path * track.yvRel) < 0.0
  )
  if moving_toward_center and track.dRel < LATERAL_FLYBY_CUTIN_PROTECT_DREL:
    return False
  if abs(track.yvRel) > LATERAL_FLYBY_MAX_YVREL_FOR_REJECT:
    return False

  mo = track.motionOrientation
  la = track.laneAssignment
  side_evidence = (
    (la in ADJACENT_LANES and mo in SAME_DIR_ORIENTATIONS) or
    (mo in DRIFTING_ORIENTATIONS) or
    (hist_lat > LATERAL_FLYBY_STRONG_HIST_LAT and la in ADJACENT_LANES)
  )
  if side_evidence:
    return True

  return False


def select_best_radar_track(tracks: dict[int, Track], v_ego: float,
                            exclude_track_id: int = -1,
                            current_lead_id: int = -1,
                            path_x: np.ndarray | None = None,
                            path_y: np.ndarray | None = None,
                            path_y_std: np.ndarray | None = None,
                            path_valid: bool = False) -> Track | None:
  """Select the best radar track for lead following (L2++ style).

  Applies lateral / distance / velocity gates, scores candidates by
  collision risk, and enforces **hysteresis**: the current lead keeps its
  position unless a challenger scores ``LEAD_HYSTERESIS_MARGIN`` higher.

  When ``path_valid`` is True, lateral gates are computed relative to the
  model's predicted driving path instead of the straight-line radar axis.

  Returns:
    Best track or None.
  """
  if not tracks:
    return None

  min_d = min_lead_distance(v_ego)
  candidates: list[Track] = []

  for tid, track in tracks.items():
    if tid == exclude_track_id:
      continue
    if track.dRel < min_d:
      continue
    if not is_track_in_planning_path(track, path_x, path_y, path_y_std, path_valid):
      continue

    # Compute path-relative lateral offset for this track
    if path_valid and path_x is not None and path_y is not None:
      pyo, _ptol = get_path_lateral_offset(track.dRel, path_x, path_y, path_y_std)
    else:
      pyo = 0.0

    if is_crossing_traffic(track, v_ego, path_y_offset=pyo):
      continue
    if is_lateral_flyby(track, path_y_offset=pyo):
      continue

    # DBC lane-based rejection: adjacent-lane objects that are NOT closing
    # fast should not be lead candidates (confirmed by radar hardware).
    if (track.laneAssignment in ADJACENT_LANES and
        track.laneAssignment != LANE_UNKNOWN and
        track.motionOrientation in SAME_DIR_ORIENTATIONS and
        track.vRel > -1.0 and
        abs(track.yRel) > 1.0):
      continue

    # At very low speed (< 5 km/h), radar is noisy — only trust tracks
    # that have been consistently present for several frames.
    if v_ego < LOW_SPEED_CUTOFF and track.cnt < LOW_SPEED_MIN_TRACK_CNT:
      continue

    is_static = abs(track.vRel) < 0.5

    # Ground-truth stationary: object speed ≈ 0 in world frame while ego
    # is moving.  These are parked cars, signs, guardrails, etc.
    # vLeadK from Kalman gives absolute object speed — much more reliable
    # than vRel for detecting parked objects (vRel ≈ -v_ego, not near 0).
    ground_stationary = track.vLeadK < 1.0 and v_ego > 3.0

    # Lateral selection is handled by planning-path gating above:
    # is_track_in_planning_path(...). Avoid additional distance-coupled lateral
    # gates here.
    if track.dRel > MAX_LEAD_DIST:
      continue

    # Reject ground-stationary objects that are far away — roadside furniture
    if ground_stationary and track.dRel > MAX_STATIC_LEAD_DIST:
      continue

    if is_static:
      if v_ego < 0.5 or track.dRel < 2.0 or track.dRel > MAX_STATIC_LEAD_DIST:
        continue

    if abs(track.vRel) > MAX_VREL_FILTER and track.dRel > 20.0:
      continue

    candidates.append(track)

  if not candidates:
    return None

  def _score(t):
    if path_valid and path_x is not None and path_y is not None:
      pyo_t, _ = get_path_lateral_offset(t.dRel, path_x, path_y, path_y_std)
    else:
      pyo_t = 0.0
    return calculate_collision_risk(t, v_ego, path_y_offset=pyo_t)

  scored = [(_score(t), t) for t in candidates]
  scored.sort(key=lambda x: x[0], reverse=True)

  best_score, best_track = scored[0]

  # ── Hysteresis: keep current lead unless challenger is clearly better ─────
  if current_lead_id >= 0 and best_track.identifier != current_lead_id:
    for score, track in scored:
      if track.identifier == current_lead_id:
        if best_score - score < LEAD_HYSTERESIS_MARGIN:
          cloudlog.debug("radard: hysteresis kept lead %d (Δscore=%.3f)",
                         current_lead_id, best_score - score)
          return track
        break

  return best_track


def laplacian_pdf(x: float, mu: float, b: float):
  b = max(b, 1e-4)
  return math.exp(-abs(x-mu)/b)


def match_vision_to_track(v_ego: float, lead: capnp._DynamicStructReader,
                          tracks: dict[int, Track],
                          path_x: np.ndarray | None = None,
                          path_y: np.ndarray | None = None,
                          path_y_std: np.ndarray | None = None,
                          path_valid: bool = False) -> Track | None:
  """Find the radar track that best matches a vision lead using Laplacian PDFs.

  Returns the best-scoring track if it passes distance / velocity / lateral
  sanity gates, else None.
  """
  if not tracks:
    return None

  offset_vision_dist = lead.x[0] - RADAR_TO_CAMERA

  min_d = min_lead_distance(v_ego)
  # Filter out tracks that are too close, crossing, or too young at low speed
  min_cnt = LOW_SPEED_MIN_TRACK_CNT if v_ego < LOW_SPEED_CUTOFF else 0

  def _crossing(t):
    if path_valid and path_x is not None and path_y is not None:
      pyo, _ = get_path_lateral_offset(t.dRel, path_x, path_y, path_y_std)
    else:
      pyo = 0.0
    # Keep vision matching less edgy: crossing filter only here.
    return is_crossing_traffic(t, v_ego, path_y_offset=pyo)

  tracks = {k: v for k, v in tracks.items()
            if v.dRel >= min_d and not _crossing(v)
            and is_track_in_planning_path(v, path_x, path_y, path_y_std, path_valid)
            and v.cnt >= min_cnt}
  if not tracks:
    return None

  def prob(c):
    return (laplacian_pdf(c.dRel, offset_vision_dist, lead.xStd[0]) *
            laplacian_pdf(c.yRel, -lead.y[0], lead.yStd[0]) *
            laplacian_pdf(c.vRel + v_ego, lead.v[0], lead.vStd[0]))

  track = max(tracks.values(), key=prob)

  # Sanity gates — reject physically implausible matches
  dist_sane = abs(track.dRel - offset_vision_dist) < max(offset_vision_dist * DIST_MATCH_FACTOR, DIST_MATCH_MIN)
  vel_sane  = (abs(track.vRel + v_ego - lead.v[0]) < VEL_MATCH_REL) or \
              (abs(track.vRel + v_ego - lead.v[0]) < VEL_MATCH_ABS)
  lat_sane  = abs(track.yRel - (-lead.y[0])) < max(lead.yStd[0] * LAT_MATCH_FACTOR, LAT_MATCH_MIN)

  if dist_sane and vel_sane and lat_sane:
    return track
  return None


def get_RadarState_from_vision(lead_msg: capnp._DynamicStructReader, v_ego: float, model_v_ego: float):
  lead_v_rel_pred = lead_msg.v[0] - model_v_ego
  dRel = float(lead_msg.x[0] - RADAR_TO_CAMERA)

  # Speed-dependent minimum distance: reject implausibly close vision leads
  if dRel < min_lead_distance(v_ego):
    return {'status': False}

  return {
    "dRel": dRel,
    "yRel": float(-lead_msg.y[0]),
    "vRel": float(lead_v_rel_pred),
    "vLead": float(v_ego + lead_v_rel_pred),
    "vLeadK": float(v_ego + lead_v_rel_pred),
    "aLeadK": float(lead_msg.a[0]),
    "aLeadTau": 0.3,
    "fcw": False,
    "modelProb": float(lead_msg.prob),
    "status": True,
    "radar": False,
    "radarTrackId": -1,
  }


def get_lead(v_ego: float, ready: bool, tracks: dict[int, Track],
             lead_msg: capnp._DynamicStructReader, model_v_ego: float,
             low_speed_override: bool = True, prioritize_vision: bool = False,
             exclude_track_id: int = -1,
             current_lead_id: int = -1,
             path_x: np.ndarray | None = None,
             path_y: np.ndarray | None = None,
             path_y_std: np.ndarray | None = None,
             path_valid: bool = False) -> dict[str, Any]:
  """Determine the best lead vehicle from radar tracks + vision.

  Priority:
    1. Fused radar+vision (highest quality)
    2. Vision-only (no radar match, vision confident)
    3. Radar-only (vision weak, radar filters already passed — gated by path)
    4. Low-speed override (trust radar for close objects at low ego speed)

  ``current_lead_id`` enables hysteresis: the current lead is preferred unless
  a challenger scores significantly higher.
  """
  # ── Priority 1: radar + vision fusion ──────────────────────────────────────
  track: Track | None = None
  if tracks and ready and lead_msg.prob > VISION_MATCH_THRESHOLD:
    track = match_vision_to_track(v_ego, lead_msg, tracks,
                                  path_x=path_x, path_y=path_y,
                                  path_y_std=path_y_std, path_valid=path_valid)

  lead_dict: dict[str, Any] = {'status': False}

  if track is not None:
    lead_dict = track.get_RadarState(lead_msg.prob, v_ego)
    lead_dict['fcw'] = should_trigger_fcw(track, v_ego, lead_msg.prob)

    # Static velocity override (only when vision says static AND radar is way off)
    # Guard: skip override when Kalman-filtered lead speed confirms the object
    # is genuinely moving — vision often misreports slow motorbikes as static.
    if (abs(lead_msg.v[0]) < STATIC_VEL_THRESHOLD
        and abs(track.vRel + v_ego) > STATIC_RADAR_VEL_MIN
        and track.vLeadK < 2.0):
      vision_vRel = lead_msg.v[0] - v_ego
      lead_dict['vRel'] = vision_vRel
      lead_dict['vLead'] = v_ego + vision_vRel
      lead_dict['vLeadK'] = v_ego + vision_vRel
      cloudlog.debug("radard: static override track %d  v_radar=%.1f → v_vision=%.1f",
                     track.identifier, track.vRel + v_ego, lead_msg.v[0])

  # ── Priority 2: vision-only ────────────────────────────────────────────────
  elif track is None and ready and lead_msg.prob > VISION_ONLY_THRESHOLD:
    lead_dict = get_RadarState_from_vision(lead_msg, v_ego, model_v_ego)

  # ── Priority 3: radar-only (gated by driving path) ──────────────────────────
  elif tracks and ready and lead_msg.prob <= VISION_ONLY_THRESHOLD:
    best = select_best_radar_track(tracks, v_ego, exclude_track_id,
                                   current_lead_id=current_lead_id,
                                   path_x=path_x, path_y=path_y,
                                   path_y_std=path_y_std, path_valid=path_valid)
    if best is not None and best.dRel < RADAR_ONLY_MAX_DIST:
      if is_track_in_planning_path(best, path_x, path_y, path_y_std, path_valid):
        lead_dict = best.get_RadarState(0.0, v_ego)
        lead_dict['fcw'] = False

  # Precompute minimum plausible lead distance for this ego speed.
  min_d = min_lead_distance(v_ego)

  # ── Priority 4: low-speed override (radar priority lowered) ────────────────
  # Only override vision when there's no vision lead at all, or radar is much closer (0.5x)
  if low_speed_override:
    def _low_speed_path_valid(c: Track) -> bool:
      if v_ego >= CREEP_CLOSEST_LEAD_VEGO:
        return True
      if not (path_valid and path_x is not None and path_y is not None):
        return True
      pyo, _ = get_path_lateral_offset(c.dRel, path_x, path_y, path_y_std)
      return abs(c.yRel - pyo) <= LOW_SPEED_PATH_MAX_LAT

    def _creep_closest_candidate(c: Track) -> bool:
      # Creep mode should lock onto the closest same-direction object (e.g. a
      # motorbike stopping/rolling forward), not lateral crossers.
      if c.dRel < min_d or c.dRel > CREEP_LEAD_MAX_DREL:
        return False
      if c.cnt < CREEP_LEAD_MIN_TRACK_CNT:
        return False
      if c.vLeadK < CREEP_LEAD_MIN_VLEADK:
        return False
      if abs(c.yvRel) > CREEP_LEAD_MAX_YVREL:
        return False
      if c.motionOrientation in ONCOMING_ORIENTATIONS:
        return False
      if c.motionOrientation in CROSSING_ORIENTATIONS:
        return False
      if c.laneAssignment in ADJACENT_LANES:
        return False
      return (is_track_in_planning_path(c, path_x, path_y, path_y_std, path_valid) and
              _low_speed_path_valid(c))

    low_speed_tracks = [c for c in tracks.values()
                        if c.potential_low_speed_lead(v_ego)
                        and is_track_in_planning_path(c, path_x, path_y, path_y_std, path_valid)
                        and _low_speed_path_valid(c)]
    if v_ego < CREEP_CLOSEST_LEAD_VEGO:
      creep_tracks = [c for c in tracks.values() if _creep_closest_candidate(c)]
      if creep_tracks:
        # Prefer DBC "preceding" targets first, then shortest distance.
        closest = min(creep_tracks, key=lambda c: (0 if c.motionOrientation == ORIENT_PRECEEDING else 1, c.dRel))
        lead_dict = closest.get_RadarState(0.0, v_ego)
        lead_dict['fcw'] = False
      elif low_speed_tracks:
        closest = min(low_speed_tracks, key=lambda c: c.dRel)
        lead_dict = closest.get_RadarState(0.0, v_ego)
        lead_dict['fcw'] = False
    elif low_speed_tracks:
      closest = min(low_speed_tracks, key=lambda c: c.dRel)
      # Creep mode behavior: always follow the closest valid radar lead.
      # This makes re-engage crawling deterministic in stop-and-go traffic.
      vision_ok = lead_dict['status'] and lead_dict.get('modelProb', 0.0) > VISION_ONLY_THRESHOLD
      if (not lead_dict['status']) or \
         (not vision_ok and closest.dRel < 12.0) or \
         (closest.dRel < lead_dict.get('dRel', float('inf')) * 0.5):
        lead_dict = closest.get_RadarState(0.0, v_ego)
        lead_dict['fcw'] = False

  # ── Final safety gate ──────────────────────────────────────────────────────
  if lead_dict['status'] and lead_dict.get('dRel', 0) < min_d:
    lead_dict = {'status': False}

  # Hard geometry sanity gate: never accept very close leads that are clearly
  # lateral to our path (classic phantom braking case, e.g. dRel~1.5m, yRel~2.5m).
  if lead_dict.get('status', False):
    d_rel = float(lead_dict.get('dRel', 0.0))
    y_rel = float(lead_dict.get('yRel', 0.0))
    if path_valid and path_x is not None and path_y is not None:
      pyo, _ = get_path_lateral_offset(d_rel, path_x, path_y, path_y_std)
      lat_from_path = abs(y_rel - pyo)
    else:
      lat_from_path = abs(y_rel)
    if d_rel < 4.0 and lat_from_path > 1.2:
      lead_dict = {'status': False}

  # Low-speed strict path gate: in creeping traffic, only brake for objects
  # very close to the planned path centre. Prevents side peds/bikes from
  # becoming lead when radar labels them as "in-lane" but UI path says otherwise.
  if lead_dict.get('status', False) and v_ego < CREEP_CLOSEST_LEAD_VEGO:
    d_rel = float(lead_dict.get('dRel', 0.0))
    y_rel = float(lead_dict.get('yRel', 0.0))
    if path_valid and path_x is not None and path_y is not None:
      pyo, _ = get_path_lateral_offset(d_rel, path_x, path_y, path_y_std)
      lat_from_path = abs(y_rel - pyo)
      if lat_from_path > LOW_SPEED_PATH_MAX_LAT:
        lead_dict = {'status': False}
    else:
      # If model path is temporarily unavailable, keep the same strict behavior
      # using straight-line lateral as a fallback to avoid side-object braking.
      if abs(y_rel) > LOW_SPEED_PATH_MAX_LAT:
        lead_dict = {'status': False}

  track_obj = None
  if lead_dict.get('status', False) and lead_dict.get('radar', False):
    tid = int(lead_dict.get('radarTrackId', -1))
    track_obj = tracks.get(tid)

  # Global stability requirement for radar leads.
  if lead_dict.get('status', False) and track_obj is not None:
    d_rel = float(lead_dict.get('dRel', 0.0))
    v_rel = float(lead_dict.get('vRel', 0.0))
    closing_speed = -v_rel
    ttc = d_rel / max(closing_speed, 0.1)
    min_cnt_required = (RADAR_LEAD_MIN_TRACK_CNT_LOW_SPEED
                        if v_ego < CREEP_CLOSEST_LEAD_VEGO
                        else RADAR_LEAD_MIN_TRACK_CNT)
    if (track_obj.cnt < min_cnt_required and
            not (d_rel < RADAR_LEAD_BYPASS_DREL and ttc <= RADAR_LEAD_BYPASS_TTC)):
      lead_dict = {'status': False}
      track_obj = None

  # Side-track false-closing reject: remove lateral fly-bys that can survive
  # early selection during brief DBC classification noise.
  if lead_dict.get('status', False) and track_obj is not None:
    d_rel = float(lead_dict.get('dRel', 0.0))
    y_rel = float(lead_dict.get('yRel', 0.0))
    if path_valid and path_x is not None and path_y is not None:
      pyo, _ = get_path_lateral_offset(d_rel, path_x, path_y, path_y_std)
    else:
      pyo = 0.0
    if is_lateral_flyby(track_obj, path_y_offset=pyo):
      lead_dict = {'status': False}
      track_obj = None

  # Slow side-mover reject (ped/cyclist-like): if the selected radar lead is
  # moving slowly in world frame, laterally offset from the planning path, and
  # not strongly closing longitudinally, treat it as passing/crossing traffic.
  if lead_dict.get('status', False) and track_obj is not None:
    d_rel = float(lead_dict.get('dRel', 0.0))
    y_rel = float(lead_dict.get('yRel', 0.0))
    v_rel = float(lead_dict.get('vRel', 0.0))
    if path_valid and path_x is not None and path_y is not None:
      pyo, _ = get_path_lateral_offset(d_rel, path_x, path_y, path_y_std)
      lat_from_path = abs(y_rel - pyo)
    else:
      lat_from_path = abs(y_rel)
    host_preceding = (track_obj.motionOrientation == ORIENT_PRECEEDING and
                      track_obj.laneAssignment == LANE_HOST)
    keep_host_preceding_fast = host_preceding and v_ego > HOST_PRECEDING_PRIORITY_VEGO
    if (d_rel < 25.0 and lat_from_path > 0.8 and track_obj.vLeadK < 2.5 and
            v_rel > -2.0 and abs(track_obj.yvRel) > 0.2 and
            not keep_host_preceding_fast):
      lead_dict = {'status': False}
      track_obj = None

  # Hard-coded reject (requested): if ego is above 10 km/h, object is slower
  # than ego (vRel < 0), pedestrian-like slow absolute speed, and within 1.5 m
  # of path centre, reject to avoid phantom brake on passing pedestrians.
  if lead_dict.get('status', False) and track_obj is not None and v_ego > HARD_PED_REJECT_VEGO:
    d_rel = float(lead_dict.get('dRel', 0.0))
    y_rel = float(lead_dict.get('yRel', 0.0))
    v_rel = float(lead_dict.get('vRel', 0.0))
    if path_valid and path_x is not None and path_y is not None:
      pyo, _ = get_path_lateral_offset(d_rel, path_x, path_y, path_y_std)
      lat_from_path = abs(y_rel - pyo)
    else:
      lat_from_path = abs(y_rel)
    is_ped_like = track_obj.vLeadK < HARD_PED_REJECT_MAX_VLEAD and abs(track_obj.yvRel) > 0.2
    host_preceding = (track_obj.motionOrientation == ORIENT_PRECEEDING and
                      track_obj.laneAssignment == LANE_HOST)
    keep_host_preceding_fast = host_preceding and v_ego > HOST_PRECEDING_PRIORITY_VEGO
    if is_ped_like and v_rel < -0.1 and lat_from_path < HARD_PED_REJECT_MAX_LAT and not keep_host_preceding_fast:
      lead_dict = {'status': False}
      track_obj = None

  # Slow VRU reject (DBC-assisted): reject nearby slow moving/crossing objects
  # that are around path center but exhibit lateral motion typical of side pass.
  if lead_dict.get('status', False) and track_obj is not None:
    d_rel = float(lead_dict.get('dRel', 0.0))
    y_rel = float(lead_dict.get('yRel', 0.0))
    if path_valid and path_x is not None and path_y is not None:
      pyo, _ = get_path_lateral_offset(d_rel, path_x, path_y, path_y_std)
      lat_from_path = abs(y_rel - pyo)
    else:
      lat_from_path = abs(y_rel)
    ms = track_obj.motionStatus
    mo = track_obj.motionOrientation
    slow_vru_like = (
      track_obj.vLeadK < SLOW_VRU_REJECT_MAX_VLEAD and
      ms in (MSTATUS_MOVING_SLOWLY, MSTATUS_MOVING) and
      abs(track_obj.yvRel) > SLOW_VRU_REJECT_MIN_YVREL
    )
    vru_orient = mo in (ORIENT_PRECEEDING, ORIENT_CROSSING_LEFT, ORIENT_CROSSING_RIGHT)
    host_preceding = (mo == ORIENT_PRECEEDING and track_obj.laneAssignment == LANE_HOST)
    if (slow_vru_like and vru_orient and not host_preceding and d_rel < SLOW_VRU_REJECT_MAX_DREL and
            0.35 < lat_from_path < SLOW_VRU_REJECT_MAX_LAT):
      # Keep truly urgent objects; otherwise treat as pass-by/crossing VRU.
      v_rel = float(lead_dict.get('vRel', 0.0))
      closing_speed = -v_rel
      ttc = d_rel / max(closing_speed, 0.1)
      if ttc > 1.1:
        lead_dict = {'status': False}
        track_obj = None

  # Predicted miss-distance reject (handles "ped/bike flying at ego" artifact):
  # if projected lateral offset at closest longitudinal approach is still large,
  # this object will pass by, not collide.
  if lead_dict.get('status', False) and track_obj is not None:
    d_rel = float(lead_dict.get('dRel', 0.0))
    y_rel = float(lead_dict.get('yRel', 0.0))
    v_rel = float(lead_dict.get('vRel', 0.0))
    closing_speed = -v_rel
    if path_valid and path_x is not None and path_y is not None:
      pyo, _ = get_path_lateral_offset(d_rel, path_x, path_y, path_y_std)
      lat_from_path = y_rel - pyo
    else:
      lat_from_path = y_rel
    if (SIDE_PASS_MIN_DREL < d_rel < SIDE_PASS_MAX_DREL and
            abs(lat_from_path) > SIDE_PASS_MIN_LAT and
            abs(track_obj.yvRel) > SIDE_PASS_MIN_YVREL and
            closing_speed > 0.1):
      ttc = min(d_rel / closing_speed, SIDE_PASS_MAX_TTC)
      lat_at_ttc = lat_from_path + track_obj.yvRel * ttc
      if abs(lat_at_ttc) > SIDE_PASS_MISS_LAT:
        lead_dict = {'status': False}
        track_obj = None

  # Near-field stability rule: require a few consecutive frames before
  # accepting close radar leads, unless TTC is critically short.
  if lead_dict.get('status', False) and track_obj is not None:
    d_rel = float(lead_dict.get('dRel', 0.0))
    v_rel = float(lead_dict.get('vRel', 0.0))
    closing_speed = -v_rel
    ttc = d_rel / max(closing_speed, 0.1)
    if (d_rel < NEAR_FIELD_STABLE_DREL and
            track_obj.cnt < NEAR_FIELD_MIN_TRACK_CNT and
            ttc > NEAR_FIELD_BYPASS_TTC):
      lead_dict = {'status': False}
      track_obj = None

  # Apply short hold to new radar leads so initial noisy spikes don't cause
  # harsh decel before the track stabilizes.
  if lead_dict.get('status', False) and lead_dict.get('radar', False):
    if track_obj is not None:
      lead_dict = apply_lead_stability_hold(lead_dict, track_obj, v_ego)
      lead_dict = stabilize_low_speed_radar_lead(lead_dict, track_obj, v_ego)
      lead_dict = stabilize_position_only_radar_lead(lead_dict, track_obj, v_ego)

  return lead_dict


class RadarD:
  """Radar–vision fusion engine.

  Maintains Kalman-filtered ``Track`` objects, selects up to two leads
  (``leadOne``, ``leadTwo``) and publishes ``radarState``.
  """

  def __init__(self, delay: float = 0.0, prioritize_vision: bool = False):
    self.current_time = 0.0

    self.tracks: dict[int, Track] = {}
    self.kalman_params = KalmanParams(DT_MDL)

    self.v_ego = 0.0
    self.v_ego_hist = deque([0.0], maxlen=max(1, int(round(delay / DT_MDL)) + 1))
    self.last_v_ego_frame = -1

    self.radar_state: capnp._DynamicStructBuilder | None = None
    self.radar_state_valid = False

    self.ready = False
    self.prioritize_vision = prioritize_vision

    # Model driving-path arrays (updated each frame from modelV2.position)
    self.path_x = np.array([0.0, 200.0])
    self.path_y = np.array([0.0, 0.0])
    self.path_y_std: np.ndarray | None = None
    self.path_valid = False

    # Track IDs of the last selected leads (for hysteresis)
    self._lead_one_id: int = -1
    self._lead_two_id: int = -1

  def update(self, sm: messaging.SubMaster, rr):
    """Process new radar + vision data and build radarState."""
    self.ready = sm.seen['modelV2']
    self.current_time = 1e-9 * max(sm.logMonoTime.values())

    if sm.recv_frame['carState'] != self.last_v_ego_frame:
      self.v_ego = sm['carState'].vEgo
      self.v_ego_hist.append(self.v_ego)
      self.last_v_ego_frame = sm.recv_frame['carState']

    min_d = min_lead_distance(self.v_ego)
    ar_pts = {pt.trackId: [pt.dRel, pt.yRel, pt.vRel, pt.measured, pt.yvRel,
                           getattr(pt, 'motionStatus', 0),
                           getattr(pt, 'motionOrientation', 0),
                           getattr(pt, 'laneAssignment', 0)]
              for pt in rr.points if pt.dRel >= min_d}

    # Remove stale tracks
    for tid in list(self.tracks.keys()):
      if tid not in ar_pts:
        self.tracks.pop(tid, None)

    # Update / create tracks
    v_ego_delayed = self.v_ego_hist[0] if self.v_ego_hist else self.v_ego
    for tid, rpt in ar_pts.items():
      v_lead = rpt[2] + v_ego_delayed
      if tid not in self.tracks:
        self.tracks[tid] = Track(tid, v_lead, self.kalman_params)
      self.tracks[tid].update(rpt[0], rpt[1], rpt[2], v_lead, rpt[3], rpt[4],
                              motion_status=rpt[5], motion_orientation=rpt[6],
                              lane_assignment=rpt[7])

    # Build radarState
    self.radar_state_valid = sm.all_checks()
    self.radar_state = log.RadarState.new_message()
    self.radar_state.mdMonoTime = sm.logMonoTime['modelV2']
    self.radar_state.radarErrors = rr.errors
    self.radar_state.carStateMonoTime = sm.logMonoTime['carState']

    model_v_ego = (sm['modelV2'].velocity.x[0]
                   if len(sm['modelV2'].velocity.x) else self.v_ego)

    # Extract driving-path geometry for path-relative lateral filtering.
    # position.x/.y are time-indexed predictions of ego future position;
    # .x is monotonically increasing (car moves forward).
    model_pos = sm['modelV2'].position
    if len(model_pos.x) >= 2 and len(model_pos.y) >= 2:
      px = np.array(model_pos.x)
      py = np.array(model_pos.y)
      # Ensure monotonic x (should always be true, but defensive)
      if np.all(np.diff(px) > 0):
        self.path_x = px
        self.path_y = py
        self.path_y_std = np.array(model_pos.yStd) if len(model_pos.yStd) == len(px) else None
        self.path_valid = self.v_ego >= PATH_MIN_VEGO
      else:
        self.path_valid = False
    else:
      self.path_valid = False

    leads_v3 = sm['modelV2'].leadsV3
    _pk = dict(path_x=self.path_x, path_y=self.path_y,
               path_y_std=self.path_y_std, path_valid=self.path_valid)

    if len(leads_v3) > 1:
      # ── Lead One ──────────────────────────────────────────────────
      lead_one = get_lead(self.v_ego, self.ready, self.tracks, leads_v3[0],
                          model_v_ego, low_speed_override=True,
                          prioritize_vision=self.prioritize_vision,
                          exclude_track_id=-1,
                          current_lead_id=self._lead_one_id, **_pk)
      self.radar_state.leadOne = lead_one
      lead_one_tid = (self.radar_state.leadOne.radarTrackId
                      if self.radar_state.leadOne.status else -1)
      self._lead_one_id = lead_one_tid

      # ── Lead Two (exclude LeadOne's track) ────────────────────────
      lead_two = get_lead(self.v_ego, self.ready, self.tracks, leads_v3[1],
                          model_v_ego, low_speed_override=False,
                          prioritize_vision=self.prioritize_vision,
                          exclude_track_id=lead_one_tid,
                          current_lead_id=self._lead_two_id, **_pk)
      self.radar_state.leadTwo = lead_two
      self._lead_two_id = (self.radar_state.leadTwo.radarTrackId
                           if self.radar_state.leadTwo.status else -1)
    elif len(leads_v3) == 1:
      # Only one vision lead available — still try to fuse
      lead_one = get_lead(self.v_ego, self.ready, self.tracks, leads_v3[0],
                          model_v_ego, low_speed_override=True,
                          prioritize_vision=self.prioritize_vision,
                          current_lead_id=self._lead_one_id, **_pk)
      self.radar_state.leadOne = lead_one
      self._lead_one_id = (self.radar_state.leadOne.radarTrackId
                           if self.radar_state.leadOne.status else -1)
      self._lead_two_id = -1
    else:
      # No vision leads — radar-only fallback (still gated by path)
      cloudlog.debug("radard: no vision leads (leadsV3 empty)")
      best = select_best_radar_track(self.tracks, self.v_ego,
                                     current_lead_id=self._lead_one_id, **_pk)
      if best is not None and best.dRel < RADAR_ONLY_MAX_DIST:
        if is_track_in_planning_path(best, self.path_x, self.path_y, self.path_y_std, self.path_valid):
          self.radar_state.leadOne = best.get_RadarState(0.0, self.v_ego)
          self._lead_one_id = best.identifier
        else:
          self._lead_one_id = -1
      else:
        self._lead_one_id = -1
      self._lead_two_id = -1

  def publish(self, pm: messaging.PubMaster):
    assert self.radar_state is not None

    radar_msg = messaging.new_message("radarState")
    radar_msg.valid = self.radar_state_valid
    radar_msg.radarState = self.radar_state
    pm.send("radarState", radar_msg)


# fuses camera and radar data for best lead detection
def main() -> None:
  config_realtime_process(5, Priority.CTRL_LOW)

  # wait for stats about the car to come in from controls
  cloudlog.info("radard is waiting for CarParams")
  CP = messaging.log_from_bytes(Params().get("CarParams", block=True), car.CarParams)
  cloudlog.info("radard got CarParams")

  # *** setup messaging
  sm = messaging.SubMaster(['modelV2', 'carState', 'liveTracks'], poll='modelV2')
  pm = messaging.PubMaster(['radarState'])

  # Trust radar more for VinFast: DBC now has Rel_Vx on all 20 objects,
  # DBC is correct, radar data quality is reliable.
  # prioritize_vision=False allows radar to override vision at low speeds
  prioritize_vision = False
  RD = RadarD(CP.radarDelay, prioritize_vision=prioritize_vision)

  while 1:
    sm.update()

    RD.update(sm, sm['liveTracks'])
    RD.publish(pm)


if __name__ == "__main__":
  main()
