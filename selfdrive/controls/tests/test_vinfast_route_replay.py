#!/usr/bin/env python3
"""Automated VinFast route replay test — download rlog + re-run radard logic.

Downloads route logs from comma API (public routes, no auth) and replays
``liveTracks`` + ``modelV2`` through current ``radard.get_lead()``.

Usage:
  PYENV_VERSION=3.11.4 PYTHONPATH=. python3 \\
    selfdrive/controls/tests/test_vinfast_route_replay.py \\
    5928b56aca881917/0000011b--989a00c832

  # segment range (default: all segments in route)
  ... 5928b56aca881917/0000011b--989a00c832 --segments 0

Exit code 0 = PASS, 1 = FAIL (fly-by became lead or download error).
"""
from __future__ import annotations

import argparse
import json
import logging
import os
import subprocess
import sys
import time
import types
import urllib.parse
from dataclasses import dataclass, field

import numpy as np

API_HOST = os.getenv("API_HOST", "https://api.commadotai.com")
CACHE_ROOT = os.path.join(
  os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(__file__)))),
  "data", "vinfast_routes",
)


def _stub_imports():
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


_stub_imports()

from openpilot.selfdrive.controls import radard  # noqa: E402
from openpilot.selfdrive.controls.tests.vinfast_radar_sim_common import (  # noqa: E402
  estimate_long_accel,
  is_harsh_brake,
  lead_source_label,
  model_lead_info,
)
from openpilot.tools.lib.logreader import LogReader  # noqa: E402


@dataclass
class RouteResult:
  route_id: str
  segment: int
  frames: int = 0
  flyby_frames: int = 0
  violations: int = 0
  collapse_frames: int = 0
  max_decel: float = 0.0
  max_decel_sim: float = 0.0
  harsh_accel_recorded: int = 0
  harsh_accel_sim: int = 0
  harsh_flyby_sim: int = 0
  vision_only_lead_frames: int = 0
  violation_events: list[dict] = field(default_factory=list)
  accel_events: list[dict] = field(default_factory=list)

  @property
  def passed(self) -> bool:
    return self.violations == 0 and self.harsh_flyby_sim == 0


def normalize_route_id(route: str) -> str:
  route = route.strip().replace("/", "|")
  if "|" not in route:
    raise ValueError(f"invalid route id: {route}")
  return route


def cache_dir(route_id: str) -> str:
  safe = route_id.replace("|", "_")
  return os.path.join(CACHE_ROOT, safe)


def _fetch_route_files(route_id: str) -> dict:
  out_dir = cache_dir(route_id)
  cache_file = os.path.join(out_dir, "_files.json")
  if os.path.isfile(cache_file):
    age = time.time() - os.path.getmtime(cache_file)
    if age < 3600:
      with open(cache_file) as f:
        return json.load(f)

  canonical = urllib.parse.quote(route_id, safe="")
  url = f"{API_HOST}/v1/route/{canonical}/files"
  for attempt in range(5):
    resp = subprocess.check_output(["curl", "-s", url], text=True).strip()
    if resp:
      files = json.loads(resp)
      os.makedirs(out_dir, exist_ok=True)
      with open(cache_file, "w") as f:
        json.dump(files, f)
      return files
    time.sleep(1.0 + attempt)
  raise RuntimeError(f"empty API response for route {route_id}")


def download_segment(route_id: str, seg: int, force: bool = False) -> str:
  out_dir = cache_dir(route_id)
  os.makedirs(out_dir, exist_ok=True)
  out_path = os.path.join(out_dir, f"seg{seg}_rlog.zst")
  if os.path.isfile(out_path) and not force and os.path.getsize(out_path) > 1000:
    return out_path

  files = _fetch_route_files(route_id)
  logs = files.get("logs", [])
  if seg >= len(logs):
    raise IndexError(f"segment {seg} not in route (has {len(logs)} segments)")

  log_url = logs[seg]
  print(f"Downloading seg {seg} → {out_path}")
  for attempt in range(5):
    subprocess.check_call(["curl", "-sL", log_url, "-o", out_path])
    if os.path.getsize(out_path) > 1000:
      return out_path
    time.sleep(1.0 + attempt)
  raise RuntimeError(f"download failed for seg {seg}")


def replay_segment(rlog_path: str, route_id: str, seg: int,
                   min_vego_kph: float = 10.0) -> RouteResult:
  lr = LogReader(rlog_path)
  t0: int | None = None
  v_ego = 0.0
  path_x = np.array([0.0, 200.0])
  path_y = np.array([0.0, 0.0])
  path_y_std: np.ndarray | None = None
  path_valid = False
  last_model = None
  last_accel_recorded = 0.0

  kp = radard.KalmanParams(radard.DT_MDL)
  tracks: dict[int, radard.Track] = {}
  result = RouteResult(route_id=route_id, segment=seg)

  for msg in lr:
    if t0 is None:
      t0 = msg.logMonoTime
    t_s = (msg.logMonoTime - t0) / 1e9
    which = msg.which()

    if which == "carState":
      v_ego = msg.carState.vEgo

    elif which == "carControl":
      last_accel_recorded = float(msg.carControl.actuators.accel)
      if last_accel_recorded < result.max_decel:
        result.max_decel = last_accel_recorded
      if is_harsh_brake(last_accel_recorded):
        result.harsh_accel_recorded += 1

    elif which == "modelV2":
      last_model = msg.modelV2
      pos = last_model.position
      if len(pos.x) >= 2 and len(pos.y) >= 2:
        px = np.array(pos.x)
        py = np.array(pos.y)
        if np.all(np.diff(px) > 0):
          path_x = px
          path_y = py
          path_y_std = np.array(pos.yStd) if len(pos.yStd) == len(px) else None
          path_valid = v_ego >= radard.PATH_MIN_VEGO

    elif which == "liveTracks":
      if v_ego * 3.6 < min_vego_kph or last_model is None:
        continue

      result.frames += 1
      rr = msg.liveTracks
      active = {int(pt.trackId) for pt in rr.points}
      for tid in list(tracks.keys()):
        if tid not in active:
          tracks.pop(tid, None)

      for pt in rr.points:
        tid = int(pt.trackId)
        if tid not in tracks:
          tracks[tid] = radard.Track(tid, pt.vRel + v_ego, kp)
        tr = tracks[tid]
        tr.update(
          pt.dRel, pt.yRel, pt.vRel, pt.vRel + v_ego, pt.measured,
          getattr(pt, "yvRel", None),
          motion_status=getattr(pt, "motionStatus", 0),
          motion_orientation=getattr(pt, "motionOrientation", 0),
          lane_assignment=getattr(pt, "laneAssignment", 0),
        )

      class LeadProxy:
        def __init__(self, m):
          self.leadsV3 = m.leadsV3

      lead_msg = LeadProxy(last_model)
      if len(lead_msg.leadsV3) == 0:
        continue

      model_prob, vision_x, vision_y = model_lead_info(last_model)
      model_v_ego = last_model.velocity.x[0] if len(last_model.velocity.x) else v_ego
      lead = radard.get_lead(
        v_ego, True, tracks, lead_msg.leadsV3[0], model_v_ego,
        path_x=path_x, path_y=path_y, path_y_std=path_y_std, path_valid=path_valid,
      )

      a_sim = estimate_long_accel(lead, v_ego)
      if a_sim < result.max_decel_sim:
        result.max_decel_sim = a_sim
      if is_harsh_brake(a_sim):
        result.harsh_accel_sim += 1

      src = lead_source_label(lead)
      if lead.get("status") and not lead.get("radar"):
        result.vision_only_lead_frames += 1

      any_flyby_lead = False
      for tid, tr in tracks.items():
        pyo = 0.0
        if path_valid:
          pyo, _ = radard.get_path_lateral_offset(tr.dRel, path_x, path_y, path_y_std)
        flyby = radard.is_lateral_flyby(tr, path_y_offset=pyo)
        hist = max(tr.peak_abs_yRel, tr.max_recent_abs_yRel)
        collapse = (
          hist >= radard.LATERAL_FLYBY_HIST_MIN_LAT
          and abs(tr.yRel - pyo) <= radard.LATERAL_FLYBY_CUR_MAX_LAT
          and abs(tr.yRel - pyo) / max(hist, 1e-3) <= radard.LATERAL_FLYBY_COLLAPSE_RATIO_MAX
        )
        if collapse:
          result.collapse_frames += 1
        if flyby:
          result.flyby_frames += 1

        is_lead_tid = (
          lead.get("status") and lead.get("radar") and
          int(lead.get("radarTrackId", -1)) == tid
        )
        if flyby and is_lead_tid:
          result.violations += 1
          any_flyby_lead = True
          if len(result.violation_events) < 20:
            result.violation_events.append({
              "t_s": round(t_s, 2),
              "track_id": tid,
              "d_rel": round(tr.dRel, 1),
              "y_rel": round(tr.yRel, 2),
              "peak_y": round(tr.peak_abs_yRel, 2),
              "v_ego_kph": round(v_ego * 3.6, 0),
              "model_prob": round(model_prob, 2),
              "vision_y": round(vision_y, 2),
              "a_sim": round(a_sim, 2),
              "a_recorded": round(last_accel_recorded, 2),
            })

      if is_harsh_brake(a_sim) and any_flyby_lead:
        result.harsh_flyby_sim += 1
        if len(result.accel_events) < 15:
          result.accel_events.append({
            "t_s": round(t_s, 2),
            "a_sim": round(a_sim, 2),
            "a_recorded": round(last_accel_recorded, 2),
            "src": src,
            "model_prob": round(model_prob, 2),
            "vision_y": round(vision_y, 2),
            "lead_d": round(float(lead.get("dRel", 0)), 1),
            "flyby_lead": any_flyby_lead,
          })

  return result


def fetch_segment_count(route_id: str) -> int:
  return len(_fetch_route_files(route_id).get("logs", []))


def main() -> int:
  parser = argparse.ArgumentParser(description="Automated VinFast route radar replay test")
  parser.add_argument("route", help="Route id e.g. dongle_id/log_id")
  parser.add_argument("--segments", default="0", help="Segment index or range e.g. 0, 0-3, all")
  parser.add_argument("--min-vego-kph", type=float, default=10.0)
  parser.add_argument("--force-download", action="store_true")
  parser.add_argument("--json", action="store_true")
  args = parser.parse_args()

  route_id = normalize_route_id(args.route)
  n_segs = fetch_segment_count(route_id)

  if args.segments == "all":
    seg_list = list(range(n_segs))
  elif "-" in args.segments:
    a, b = args.segments.split("-", 1)
    seg_list = list(range(int(a), int(b) + 1))
  elif "," in args.segments:
    seg_list = [int(x) for x in args.segments.split(",")]
  else:
    seg_list = [int(args.segments)]

  results: list[RouteResult] = []
  for seg in seg_list:
    if seg >= n_segs:
      print(f"WARN: skip seg {seg} (route has {n_segs} segments)")
      continue
    rlog = download_segment(route_id, seg, force=args.force_download)
    print(f"Replaying {route_id} seg {seg} ({os.path.getsize(rlog) / 1e6:.1f} MB)")
    res = replay_segment(rlog, route_id, seg, min_vego_kph=args.min_vego_kph)
    results.append(res)
    status = "PASS" if res.passed else "FAIL"
    print(f"  {status}: frames={res.frames} flyby={res.flyby_frames} "
          f"collapse={res.collapse_frames} violations={res.violations} "
          f"harsh_sim={res.harsh_accel_sim} harsh_rec={res.harsh_accel_recorded} "
          f"harsh_flyby={res.harsh_flyby_sim} vision_only={res.vision_only_lead_frames} "
          f"max_decel_sim={res.max_decel_sim:.2f} max_decel_rec={res.max_decel:.2f} m/s²")
    for ev in res.violation_events:
      print(f"    violation t={ev['t_s']}s T{ev['track_id']} d={ev['d_rel']} y={ev['y_rel']} "
            f"prob={ev.get('model_prob')} vy={ev.get('vision_y')} a_sim={ev.get('a_sim')}")
    for ev in res.accel_events:
      print(f"    harsh_accel t={ev['t_s']}s a_sim={ev['a_sim']} a_rec={ev['a_recorded']} "
            f"src={ev['src']} flyby_lead={ev['flyby_lead']}")

  if args.json:
    print(json.dumps([{
      "route": r.route_id, "segment": r.segment, "passed": r.passed,
      "frames": r.frames, "flyby_frames": r.flyby_frames,
      "collapse_frames": r.collapse_frames, "violations": r.violations,
      "max_decel": r.max_decel, "max_decel_sim": r.max_decel_sim,
      "harsh_accel_sim": r.harsh_accel_sim,
      "harsh_accel_recorded": r.harsh_accel_recorded,
      "harsh_flyby_sim": r.harsh_flyby_sim,
      "vision_only_lead_frames": r.vision_only_lead_frames,
      "violation_events": r.violation_events,
      "accel_events": r.accel_events,
    } for r in results], indent=2))

  all_pass = all(r.passed for r in results)
  print(f"\n{'=' * 60}")
  print(f"ROUTE {route_id}: {'PASS' if all_pass else 'FAIL'} ({len(results)} segment(s))")
  return 0 if all_pass else 1


if __name__ == "__main__":
  raise SystemExit(main())
