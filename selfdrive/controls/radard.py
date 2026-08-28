"""_BYTECODE_SHIM — logic is loaded from __pycache__/radard_impl.*.pyc (same Python minor as build).

VF6/VF7 InfoCAN post-load patches: position-only distance stick-slip at red lights
makes vLead/aLeadK chatter; clamp near-stopped leads so ego speed does not ping-pong.
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

# VF7 InfoCAN: offset motorbikes in the host lane at red lights (no-op if absent).
if hasattr(_mod, "INFO_HOST_CREEP_MAX_LAT"):
  _mod.INFO_HOST_CREEP_MAX_LAT = 2.0
if hasattr(_mod, "INFO_HOST_MIN_TRACK_CNT"):
  _mod.INFO_HOST_MIN_TRACK_CNT = 2

# Treat lead as stationary when absolute speed estimate is below this at low ego speed.
_VF_STOPPED_LEAD_V = 1.2  # [m/s]


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


_orig_stabilize_low_speed = getattr(_mod, "stabilize_low_speed_radar_lead", None)
if _orig_stabilize_low_speed is not None:
  def stabilize_low_speed_radar_lead(lead_dict, track, v_ego):
    lead_dict = _orig_stabilize_low_speed(lead_dict, track, v_ego)
    return _stabilize_stopped_infocan_lead(lead_dict, track, v_ego)

  _mod.stabilize_low_speed_radar_lead = stabilize_low_speed_radar_lead

sys.modules[__name__] = _mod
