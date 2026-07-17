"""Width-aware radar path nudge for VF8/VF9.

Converts adjacent-lane radar clearance into a small lateral path bias
(``y_nudge`` meters), then to a capped curvature offset ``Δκ``.

Design (comfort trim, not a planner):
  * obstacle on the right → nudge left (negative Δκ in openpilot)
  * obstacle on the left  → nudge right (positive Δκ)
  * both sides tight      → cancel
  * soft gap deadzone so threats do not bang-bang at the cap
"""
from __future__ import annotations

import math
from typing import Any, Iterable

import numpy as np

# Ego body half-width (excl. mirrors)
W_HALF = {
  "VINFAST_VF8": 0.967,
  "VINFAST_VF9": 1.000,
}

# Assumed object half-width: mid between car (~0.9) and moto (~0.35) so gap
# saturates less often on roadside clutter / thin tracks.
OBJ_HALF = 0.55

# Soft clearance band [m]: ignore above deadzone; full threat at soft_full.
GAP_DEADZONE = 0.75
GAP_SOFT_FULL = 0.20

# Desired path shift budget [m] and curvature cap.
# ~10 cm bias: at urban lookahead (~22 m) maps near κ_cap without constant sat.
Y_NUDGE_MAX = 0.10
KAPPA_CAP = 3.5e-4

# Track geometry gates
MIN_DREL = 4.0
MAX_DREL = 45.0
MIN_LAT = 0.90
MAX_LAT = 3.20

# Speed band [m/s] / [kph]
MIN_VEGO = 2.0
MAX_VEGO_KPH = 70.0
# Speed-dependent nudge gain: +50% below 30 km/h, half strength above.
SPEED_GAIN_VEGO_KPH = 30.0
LOW_SPEED_NUDGE_SCALE = 1.50
HIGH_SPEED_NUDGE_SCALE = 0.50

# Stationary roadside: only very close, and heavily down-weighted.
STATIONARY_MAX_DREL = 12.0
STATIONARY_WEIGHT = 0.35
MOVING_MIN_VLEAD = 1.0  # [m/s] absolute

# Cut-in toward path → leave to longitudinal, do not nudge away.
CUTIN_MAX_LAT = 1.20
CUTIN_MIN_YVREL = 0.40
CUTIN_MAX_DREL = 25.0

# Model-curvature fade (same spirit as VF static bias fade).
CURV_FADE_SCALE = 0.002

# Slow LPF so left/right flicker does not weave.
LPF_ALPHA = 0.08

# DBC orientation / lane (0 = invalid/unknown — common on some builds)
ORIENT_ONCOMING = 6
ORIENT_OC_DRIFT_RIGHT = 5
ORIENT_OC_DRIFT_LEFT = 7
ONCOMING_ORIENTATIONS = {ORIENT_ONCOMING, ORIENT_OC_DRIFT_LEFT, ORIENT_OC_DRIFT_RIGHT}
LANE_HOST = 3
MSTATUS_STATIONARY = 3
MSTATUS_STOPPED = 4


def is_vf_radar_nudge_car(fingerprint: str) -> bool:
  return fingerprint in W_HALF


def path_y_at_d(d_rel: float, path_x: np.ndarray | None, path_y: np.ndarray | None) -> float:
  """Path lateral at ``d_rel`` in openpilot frame (+right).

  ``modelV2.position.y`` is ISO (+left), so negate like radard.
  """
  if path_x is None or path_y is None or len(path_x) < 2:
    return 0.0
  if d_rel < float(path_x[0]) or d_rel > float(path_x[-1]):
    return 0.0
  return -float(np.interp(d_rel, path_x, path_y))


def lookahead_m(v_ego: float) -> float:
  """Speed-dependent lookahead for y→κ mapping."""
  return float(np.interp(v_ego, [2.0, 8.0, 20.0], [15.0, 22.0, 40.0]))


def soft_gap_threat(gap: float) -> float:
  """Smooth threat in [0, 1] from clearance gap (meters)."""
  if gap >= GAP_DEADZONE:
    return 0.0
  if gap <= GAP_SOFT_FULL:
    return 1.0
  x = (GAP_DEADZONE - gap) / (GAP_DEADZONE - GAP_SOFT_FULL)
  x = float(np.clip(x, 0.0, 1.0))
  # smoothstep: less bang-bang than a linear ramp
  return x * x * (3.0 - 2.0 * x)


def _track_fields(pt: Any) -> tuple[float, float, float, float, bool, int, int, int]:
  d_rel = float(getattr(pt, "dRel", 0.0))
  y_rel = float(getattr(pt, "yRel", 0.0))
  v_rel = float(getattr(pt, "vRel", 0.0))
  yv_rel = float(getattr(pt, "yvRel", 0.0))
  if not math.isfinite(yv_rel):
    yv_rel = 0.0
  measured = bool(getattr(pt, "measured", True))
  mo = int(getattr(pt, "motionOrientation", 0) or 0)
  la = int(getattr(pt, "laneAssignment", 0) or 0)
  ms = int(getattr(pt, "motionStatus", 0) or 0)
  return d_rel, y_rel, v_rel, yv_rel, measured, mo, la, ms


def side_track_threat(pt: Any, v_ego: float, w_half: float,
                      path_x: np.ndarray | None = None,
                      path_y: np.ndarray | None = None) -> tuple[float, float] | None:
  """Return ``(y_path, threat_weight)`` or None if not a side-nudge candidate.

  ``y_path`` is path-relative lateral (+right). ``threat_weight`` already
  includes soft gap threat × moving/stationary weight.
  """
  d_rel, y_rel, v_rel, yv_rel, measured, mo, la, ms = _track_fields(pt)
  if not math.isfinite(d_rel) or not math.isfinite(y_rel) or not math.isfinite(v_rel):
    return None
  if not (MIN_DREL < d_rel < MAX_DREL):
    return None

  path_off = path_y_at_d(d_rel, path_x, path_y)
  y_path = y_rel - path_off
  abs_y = abs(y_path)
  if not (MIN_LAT < abs_y < MAX_LAT):
    return None

  # Opposite corridor / oncoming — never lateral-nudge from these.
  v_lead = v_ego + v_rel
  if v_lead < -1.0 or v_rel < -(v_ego + 2.0):
    return None
  if mo in ONCOMING_ORIENTATIONS:
    return None

  # Host-lane preceding near path center is a lead, not a side nudge.
  if la == LANE_HOST and abs_y < 1.0:
    return None

  # Cut-in toward path: let longitudinal/radard own it.
  moving_toward = (y_path * yv_rel) < 0.0 and abs(yv_rel) > CUTIN_MIN_YVREL
  if moving_toward and abs_y < CUTIN_MAX_LAT and d_rel < CUTIN_MAX_DREL:
    return None

  gap = abs_y - w_half - OBJ_HALF
  threat = soft_gap_threat(gap)
  if threat <= 1e-3:
    return None

  stationary = ms in (MSTATUS_STATIONARY, MSTATUS_STOPPED) or abs(v_lead) < MOVING_MIN_VLEAD
  if stationary:
    if d_rel > STATIONARY_MAX_DREL:
      return None
    weight = STATIONARY_WEIGHT
  else:
    weight = 1.0
    # Unmeasured / estimate-only tracks are less trustworthy for path bias.
    if not measured:
      weight *= 0.7

  return y_path, threat * weight


def compute_y_nudge(points: Iterable[Any], v_ego: float, fingerprint: str,
                    path_x: np.ndarray | None = None,
                    path_y: np.ndarray | None = None) -> float:
  """Signed path bias in meters (+right). Both-side threats cancel."""
  w_half = W_HALF.get(fingerprint)
  if w_half is None:
    return 0.0

  left_threat = 0.0   # obstacle on left  → want +y (right)
  right_threat = 0.0  # obstacle on right → want -y (left)

  for pt in points:
    res = side_track_threat(pt, v_ego, w_half, path_x, path_y)
    if res is None:
      continue
    y_path, tw = res
    if y_path > 0.0:
      right_threat = max(right_threat, tw)
    else:
      left_threat = max(left_threat, tw)

  return float(np.clip((left_threat - right_threat) * Y_NUDGE_MAX, -Y_NUDGE_MAX, Y_NUDGE_MAX))


def y_nudge_to_kappa(y_nudge: float, v_ego: float, model_curvature: float = 0.0) -> float:
  """Map path bias to curvature offset, faded on curves, capped."""
  L = max(lookahead_m(v_ego), 8.0)
  dk = 2.0 * y_nudge / (L * L)
  fade = math.exp(-abs(float(model_curvature)) / CURV_FADE_SCALE)
  dk *= fade
  return float(np.clip(dk, -KAPPA_CAP, KAPPA_CAP))


class RadarPathNudge:
  """Stateful filtered radar path nudge for controlsd."""

  def __init__(self, fingerprint: str):
    self.fingerprint = fingerprint
    self.enabled = is_vf_radar_nudge_car(fingerprint)
    self.y_nudge_filt = 0.0
    self.kappa_filt = 0.0

  def reset(self) -> None:
    self.y_nudge_filt = 0.0
    self.kappa_filt = 0.0

  def update(self, points: Iterable[Any] | None, v_ego: float,
             model_curvature: float = 0.0,
             path_x: np.ndarray | None = None,
             path_y: np.ndarray | None = None,
             lat_active: bool = True,
             lane_changing: bool = False) -> float:
    """Return filtered Δκ to add to desired curvature."""
    if not self.enabled:
      return 0.0

    target_kappa = 0.0
    if lat_active and not lane_changing and points is not None:
      v_ego = max(float(v_ego), 0.0)
      v_kph = v_ego * 3.6
      if MIN_VEGO <= v_ego and v_kph <= MAX_VEGO_KPH:
        y_raw = compute_y_nudge(points, v_ego, self.fingerprint, path_x, path_y)
        self.y_nudge_filt = (1.0 - LPF_ALPHA) * self.y_nudge_filt + LPF_ALPHA * y_raw
        target_kappa = y_nudge_to_kappa(self.y_nudge_filt, v_ego, model_curvature)
        if v_kph < SPEED_GAIN_VEGO_KPH:
          target_kappa *= LOW_SPEED_NUDGE_SCALE
        else:
          target_kappa *= HIGH_SPEED_NUDGE_SCALE
      else:
        self.y_nudge_filt *= (1.0 - LPF_ALPHA)
    else:
      self.y_nudge_filt *= (1.0 - LPF_ALPHA)

    self.kappa_filt = (1.0 - LPF_ALPHA) * self.kappa_filt + LPF_ALPHA * target_kappa
    # Hold tiny residuals at zero to avoid chronic micro-bias.
    if abs(self.kappa_filt) < 1e-6:
      self.kappa_filt = 0.0
    return float(self.kappa_filt)
