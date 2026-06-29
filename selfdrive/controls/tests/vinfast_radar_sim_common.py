"""Shared helpers for VinFast radar simulation / route replay tests."""
from __future__ import annotations

from typing import Any

# Rough longitudinal estimate from fused lead (not full MPC — for phantom-brake screening).
HARSH_BRAKE_ACCEL = -2.3          # [m/s²] flag threshold
MAX_LONG_ACCEL = -3.5             # [m/s²] planner-style floor
COMFORT_DECEL_GAIN = 0.45         # vRel → accel when closing
TTC_URGENT_S = 2.0
TTC_COMFORT_S = 5.0


def estimate_long_accel(lead: dict[str, Any], v_ego: float) -> float:
  """Estimate commanded longitudinal accel from lead kinematics.

  Uses lead ``vRel``, ``dRel``, ``aLeadK`` — same inputs longitudinal MPC uses
  at a high level. Returns 0 when no lead.
  """
  if not lead.get("status"):
    return 0.0

  d_rel = float(lead.get("dRel", 0.0))
  v_rel = float(lead.get("vRel", 0.0))
  a_lead_k = float(lead.get("aLeadK", 0.0))

  if d_rel < 1.5:
    return MAX_LONG_ACCEL

  closing = max(0.0, -v_rel)
  if closing < 0.3:
    return min(0.5, max(-0.5, a_lead_k * 0.2))

  ttc = d_rel / max(closing, 0.1)
  if ttc > TTC_COMFORT_S:
    return min(0.0, v_rel * 0.15)

  # Closing-speed tracking + lead decel feedforward
  a = v_rel * COMFORT_DECEL_GAIN + a_lead_k * 0.35
  if ttc < TTC_URGENT_S:
    a = min(a, -closing * 0.35)
  return max(MAX_LONG_ACCEL, min(0.5, a))


def is_harsh_brake(accel: float) -> bool:
  return accel < HARSH_BRAKE_ACCEL


def lead_source_label(lead: dict[str, Any]) -> str:
  if not lead.get("status"):
    return "none"
  return "radar" if lead.get("radar") else "vision"


def model_lead_info(lead_msg: Any) -> tuple[float, float, float]:
  """Return (prob, vision_x_m, vision_y_iso_m) from modelV2 leadsV3[0]."""
  if lead_msg is None or len(lead_msg.leadsV3) == 0:
    return 0.0, 200.0, 0.0
  m = lead_msg.leadsV3[0]
  prob = float(m.prob)
  x = float(m.x[0]) if len(m.x) else 200.0
  y = float(m.y[0]) if len(m.y) else 0.0
  return prob, x, y
