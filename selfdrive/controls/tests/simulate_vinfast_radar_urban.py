#!/usr/bin/env python3
"""Simulate VinFast urban/highway radar scenarios through radard.get_lead().

Runs synthetic frame sequences (no rlog) for:
  1. Highway pass-by yRel-collapse (T13 signature)
  2. Vision-only adjacent-lane phantom lead
  3. Urban crossing projected miss
  4. Host-lane stopped lead (must keep)

Usage:
  PYENV_VERSION=3.11.4 PYTHONPATH=. python3 \\
    selfdrive/controls/tests/simulate_vinfast_radar_urban.py
"""
from __future__ import annotations

import logging
import sys
import types
from dataclasses import dataclass

import numpy as np

# ── import stubs (same as test_vinfast_radard_urban.py) ─────────────────────
def _stub_modules():
  rt = types.ModuleType("openpilot.common.realtime")
  rt.DT_MDL = 0.05
  rt.DT_CTRL = 0.01

  class Priority:
    CTRL_LOW = 51
    CTRL_HIGH = 53

  rt.Priority = Priority
  rt.config_realtime_process = lambda *a, **k: None
  sys.modules["openpilot.common.realtime"] = rt

  sl = types.ModuleType("openpilot.common.swaglog")
  sl.cloudlog = logging.getLogger("cloudlog")
  sys.modules["openpilot.common.swaglog"] = sl

  try:
    import openpilot.common.params  # noqa: F401
  except ImportError as e:
    if "params_pyx" not in str(e):
      raise
    ps = types.ModuleType("openpilot.common.params")

    class Params:
      def get(self, *a, **k):
        return None

    ps.Params = Params
    ps.ParamKeyFlag = int
    ps.ParamKeyType = int
    ps.UnknownKeyName = KeyError
    sys.modules["openpilot.common.params"] = ps

  try:
    import cereal.messaging  # noqa: F401
  except ImportError as e:
    if "ipc_pyx" not in str(e):
      raise
    import cereal

    ms = types.ModuleType("cereal.messaging")
    ms.SubMaster = ms.PubMaster = object
    ms.new_message = lambda *a, **k: types.SimpleNamespace()
    sys.modules["cereal.messaging"] = ms
    cereal.messaging = ms


_stub_modules()

from openpilot.selfdrive.controls import radard  # noqa: E402
from openpilot.selfdrive.controls.tests.vinfast_radar_sim_common import (  # noqa: E402
  estimate_long_accel,
  is_harsh_brake,
  lead_source_label,
  model_lead_info,
)


@dataclass
class SimFrame:
  t: float
  d_rel: float
  y_rel: float
  v_rel: float
  yv_rel: float = 0.0
  motion_status: int = radard.MSTATUS_MOVING
  motion_orientation: int = radard.ORIENT_INVALID
  lane_assignment: int = radard.LANE_UNKNOWN
  vision_x: float | None = None
  vision_y: float | None = None
  vision_v: float | None = None
  vision_prob: float = 0.0
  recorded_accel: float = 0.0  # optional: simulate logged planner output


class VisionLead:
  def __init__(self, x: float, y: float, v: float, prob: float):
    self.x = [x]
    self.y = [y]
    self.v = [v]
    self.a = [0.0]
    self.xStd = [2.0]
    self.yStd = [0.5]
    self.vStd = [1.0]
    self.prob = prob


def straight_path(max_x: float = 120.0) -> tuple[np.ndarray, np.ndarray]:
  px = np.array([0.0, max_x])
  py = np.array([0.0, 0.0])
  return px, py


def run_track_sequence(
    name: str,
    frames: list[SimFrame],
    *,
    v_ego: float,
    track_id: int = 13,
    expect_lead_at_end: bool,
) -> list[dict]:
  """Feed frames into a Track + get_lead; print timeline."""
  kp = radard.KalmanParams(radard.DT_MDL)
  tracks: dict[int, radard.Track] = {}
  path_x, path_y = straight_path()
  rows: list[dict] = []

  print(f"\n{'=' * 72}")
  print(f"SCENARIO: {name}")
  print(f"  v_ego={v_ego:.1f} m/s  expect_lead_at_end={expect_lead_at_end}")
  print(f"{'=' * 72}")
  print(f"{'t':>5} {'d':>5} {'yRel':>6} {'vRel':>6} {'peak|y|':>7} "
        f"{'flyby':>5} {'lead':>4} {'src':>6} {'prob':>5} {'vy':>5} "
        f"{'a_sim':>6} {'a_rec':>6} {'brake':>5}")
  print("-" * 72)

  max_harsh_frames = 0
  harsh_with_flyby_lead = 0

  for fr in frames:
    if track_id not in tracks:
      tracks[track_id] = radard.Track(track_id, v_ego + fr.v_rel, kp)
    tr = tracks[track_id]
    tr.update(
      fr.d_rel, fr.y_rel, fr.v_rel, v_ego + fr.v_rel, True, fr.yv_rel,
      motion_status=fr.motion_status,
      motion_orientation=fr.motion_orientation,
      lane_assignment=fr.lane_assignment,
    )

    flyby = radard.is_lateral_flyby(tr, path_y_offset=0.0)
    crossing = radard.is_crossing_traffic(tr, v_ego, path_y_offset=0.0)

    if fr.vision_x is not None:
      lead_msg = VisionLead(
        fr.vision_x + radard.RADAR_TO_CAMERA,
        fr.vision_y,
        fr.vision_v if fr.vision_v is not None else v_ego + fr.v_rel,
        fr.vision_prob,
      )
    else:
      lead_msg = VisionLead(200.0, 0.0, v_ego, 0.1)

    lead = radard.get_lead(
      v_ego, True, tracks, lead_msg, v_ego,
      path_x=path_x, path_y=path_y, path_valid=True,
    )

    a_sim = estimate_long_accel(lead, v_ego)
    a_rec = fr.recorded_accel
    if is_harsh_brake(a_sim):
      max_harsh_frames += 1
    if is_harsh_brake(a_sim) and flyby and lead.get("status"):
      harsh_with_flyby_lead += 1

    src = lead_source_label(lead)
    vy = fr.vision_y if fr.vision_y is not None else 0.0
    row = {
      "t": fr.t, "d": fr.d_rel, "y": fr.y_rel, "vRel": fr.v_rel,
      "peak": tr.peak_abs_yRel, "flyby": flyby, "crossing": crossing,
      "lead": lead.get("status", False), "src": src,
      "prob": fr.vision_prob, "vision_y": vy,
      "a_sim": a_sim, "a_rec": a_rec,
      "harsh": is_harsh_brake(a_sim),
      "lead_d": lead.get("dRel", 0), "lead_y": lead.get("yRel", 0),
      "lead_tid": lead.get("radarTrackId", -1),
    }
    rows.append(row)

    brake_flag = "YES" if is_harsh_brake(a_sim) else "no"
    print(f"{fr.t:5.1f} {fr.d_rel:5.1f} {fr.y_rel:+6.2f} {fr.v_rel:+6.2f} "
          f"{tr.peak_abs_yRel:7.2f} {str(flyby):>5} {str(row['lead']):>4} "
          f"{src:>6} {fr.vision_prob:5.2f} {vy:+5.2f} "
          f"{a_sim:+6.2f} {a_rec:+6.2f} {brake_flag:>5}")

  final = rows[-1]
  ok = final["lead"] == expect_lead_at_end and harsh_with_flyby_lead == 0
  tag = "PASS" if ok else "FAIL"
  print(f"\n  => {tag}: final lead={final['lead']} (expected {expect_lead_at_end})")
  print(f"     harsh_accel_frames={max_harsh_frames}  harsh_with_flyby_lead={harsh_with_flyby_lead}")
  if not ok:
    print(f"     leadOne: d={final['lead_d']:.1f} y={final['lead_y']:+.2f} "
          f"tid={final['lead_tid']} src={final['src']}")
  return rows


def scenario_t13_passby() -> None:
  """Highway adjacent vehicle — yRel collapses; must NOT become lead."""
  v_ego = 20.0
  frames: list[SimFrame] = []
  t = 0.0
  # Side offset phase
  for d, y, v, yv in [
    (37.0, -1.84, -2.25, 0.0),
    (35.0, -1.50, -2.40, 0.15),
    (33.0, -1.20, -2.60, 0.30),
    (31.0, -1.00, -2.80, 0.40),
  ]:
    for _ in range(5):
      frames.append(SimFrame(t, d, y, v, yv_rel=yv,
                              vision_x=d, vision_y=-y, vision_v=v_ego + v,
                              vision_prob=0.65))
      t += 0.05
  # Collapse phase
  for d, y, v, yv in [
    (30.0, -0.47, -2.95, 0.31),
    (28.5, -0.35, -3.05, 0.55),
    (27.5, -0.21, -3.05, 0.74),
  ]:
    for _ in range(4):
      frames.append(SimFrame(t, d, y, v, yv_rel=yv,
                              vision_x=d, vision_y=-y, vision_v=v_ego + v,
                              vision_prob=0.70))
      t += 0.05

  run_track_sequence("T13 highway pass-by yRel-collapse", frames,
                     v_ego=v_ego, expect_lead_at_end=False)


def scenario_vision_adjacent() -> None:
  """Vision-only lead in adjacent lane — must NOT become lead."""
  v_ego = 20.0
  frames = [
    SimFrame(0.0, 200.0, 0.0, 0.0, vision_x=45.0, vision_y=2.5,
             vision_v=13.0, vision_prob=0.75),
  ]
  run_track_sequence("Vision-only adjacent lane", frames,
                     v_ego=v_ego, track_id=99, expect_lead_at_end=False)


def scenario_host_stopped() -> None:
  """Stopped car in host lane — must remain lead."""
  v_ego = 8.0
  frames: list[SimFrame] = []
  t = 0.0
  for _ in range(12):
    frames.append(SimFrame(
      t, 18.0, 0.15, -8.0,
      motion_status=radard.MSTATUS_STOPPED,
      motion_orientation=radard.ORIENT_PRECEEDING,
      lane_assignment=radard.LANE_HOST,
      vision_x=18.0, vision_y=-0.15, vision_v=0.0, vision_prob=0.80,
    ))
    t += 0.05
  run_track_sequence("Host-lane stopped lead", frames,
                     v_ego=v_ego, track_id=1, expect_lead_at_end=True)


def scenario_crossing_miss() -> None:
  """Crossing bike projected to miss path — must NOT become lead."""
  v_ego = 8.0
  frames: list[SimFrame] = []
  t = 0.0
  for _ in range(10):
    frames.append(SimFrame(
      t, 14.0, 0.4, -5.0, yv_rel=1.0,
      motion_orientation=radard.ORIENT_CROSSING_RIGHT,
      lane_assignment=radard.LANE_HOST,
      vision_prob=0.1,
    ))
    t += 0.05
  run_track_sequence("Urban crossing projected miss", frames,
                     v_ego=v_ego, track_id=5, expect_lead_at_end=False)


def main() -> int:
  print("VinFast radar urban simulation (synthetic frames → radard.get_lead)")
  scenario_t13_passby()
  scenario_vision_adjacent()
  scenario_crossing_miss()
  scenario_host_stopped()
  print(f"\n{'=' * 72}")
  print("Done. PASS = final frame matches expected lead selection.")
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
