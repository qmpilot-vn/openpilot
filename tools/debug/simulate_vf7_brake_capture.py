#!/usr/bin/env python3
"""Simulate replay capture through info_radar filter + simplified radard Kalman.

Usage:
  PYTHONPATH=/path/to/qmpilot python3 tools/debug/simulate_vf7_brake_capture.py [capture.csv]
"""
from __future__ import annotations

import argparse
import csv
import os
import sys

_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, os.path.join(_root, "opendbc_repo"))
sys.path = [p for p in sys.path if "/opt/cereal" not in p]

from opendbc.car.vinfast.info_radar_interface import (
  INFO_KIN_ALPHA,
  INFO_KIN_BETA,
  INFO_KIN_GAMMA,
  INFO_SUSTAINED_MAX_AREL,
  INFO_VREL_DEADBAND,
  KIN_AREL_RAMP_MAX,
  KIN_AREL_RAMP_UPDATES,
  KIN_VREL_RAMP_MAX,
  KIN_WARMUP_KIN_UPDATES,
  MAX_AREL,
  MAX_VREL,
  _DistanceRateFilter,
)

ACCEL_MIN = -3.5
PREWARM_FRAMES = 80
DT_S = 0.1
DT_MDL = 0.05
POSITION_ONLY_MATURE_CNT = 8
POSITION_ONLY_HIGHWAY_VEGO = 25.0
POSITION_ONLY_MIN_VREL = -2.0
POSITION_ONLY_MIN_ALEADK = -0.5
LOW_SPEED_LEAD_SMOOTH_VEGO = 20.0 / 3.6
LOW_SPEED_LEAD_MIN_VREL = -2.5
LOW_SPEED_LEAD_MIN_ALEADK = -0.8
LOW_SPEED_LEAD_MAX_ALEADK = 1.5


class StockDistanceRateFilter:
  __slots__ = ("d", "v", "a", "t_nanos", "initialized")

  def __init__(self, v_init: float = 0.0):
    self.d = None
    self.v = v_init
    self.a = 0.0
    self.t_nanos = 0
    self.initialized = False

  def update(self, d_meas: float, t_nanos: int) -> tuple[float, float]:
    if not self.initialized or self.d is None or t_nanos <= self.t_nanos:
      self.d = d_meas
      self.t_nanos = t_nanos
      self.initialized = True
      return 0.0, 0.0
    dt = (t_nanos - self.t_nanos) / 1e9
    if dt <= 0.001 or dt > 1.0:
      self.d = d_meas
      self.t_nanos = t_nanos
      return self.v, self.a
    d_pred = self.d + self.v * dt + 0.5 * self.a * dt * dt
    residual = d_meas - d_pred
    self.d = d_pred + INFO_KIN_ALPHA * residual
    self.v = self.v + self.a * dt + (INFO_KIN_BETA / dt) * residual
    self.a = self.a + (2.0 * INFO_KIN_GAMMA / (dt * dt)) * residual
    if abs(self.v) < INFO_VREL_DEADBAND:
      self.v = 0.0
    else:
      self.v = max(-MAX_VREL, min(MAX_VREL, self.v))
    self.a = max(-MAX_AREL, min(MAX_AREL, self.a))
    self.t_nanos = t_nanos
    return self.v, self.a


class SimpleLeadKF:
  """1D Kalman on vLead (speed, accel) — matches radard Track core."""

  def __init__(self, v_lead: float):
    self.x0 = v_lead
    self.x1 = 0.0
    self.cnt = 0

  def update(self, v_lead_meas: float) -> tuple[float, float]:
    if self.cnt > 0:
      # light smoothing toward measurement (approximates KF step)
      alpha_v, alpha_a = 0.25, 0.15
      dv = v_lead_meas - self.x0
      self.x0 += alpha_v * dv
      self.x1 += alpha_a * dv / DT_MDL
    else:
      self.x0 = v_lead_meas
      self.x1 = 0.0
    self.cnt += 1
    return self.x0, self.x1


def stabilize_low_speed(lead: dict, track_cnt: int, v_ego: float, measured: bool) -> dict:
  if measured or v_ego >= LOW_SPEED_LEAD_SMOOTH_VEGO:
    return lead
  out = dict(lead)
  vrel = float(out["vRel"])
  if vrel < LOW_SPEED_LEAD_MIN_VREL:
    vrel = LOW_SPEED_LEAD_MIN_VREL
  a_lead_k = float(out["aLeadK"])
  a_lead_k = min(LOW_SPEED_LEAD_MAX_ALEADK, max(LOW_SPEED_LEAD_MIN_ALEADK, a_lead_k))
  out["vRel"] = vrel
  out["vLead"] = v_ego + vrel
  out["aLeadK"] = a_lead_k
  return out


POSITION_ONLY_ALEADK_DREL_BP = (40.0, 80.0)
POSITION_ONLY_ALEADK_V = (-3.5, -2.0)


def _position_only_aleadk_floor(d_rel: float) -> float:
  import numpy as np
  return float(np.interp(d_rel, POSITION_ONLY_ALEADK_DREL_BP, POSITION_ONLY_ALEADK_V, left=-3.5, right=-1.0))


def stabilize_position_only(lead: dict, track_cnt: int, v_ego: float, measured: bool) -> dict:
  if measured:
    return lead
  if v_ego < LOW_SPEED_LEAD_SMOOTH_VEGO:
    return lead
  out = dict(lead)
  vrel_k = out["vLeadK"] - v_ego
  vrel_raw = out["vRel"]
  alpha = 0.8 if track_cnt >= 3 else 1.0
  vrel_out = alpha * vrel_k + (1.0 - alpha) * vrel_raw
  if track_cnt < POSITION_ONLY_MATURE_CNT:
    vrel_out = max(POSITION_ONLY_MIN_VREL, vrel_out)
  a_lead_k = out["aLeadK"]
  if track_cnt < POSITION_ONLY_MATURE_CNT:
    a_lead_k = max(POSITION_ONLY_MIN_ALEADK, a_lead_k)
  else:
    floor = _position_only_aleadk_floor(float(out.get("dRel", 0)))
    if a_lead_k < floor:
      a_lead_k = floor
  out["vRel"] = vrel_out
  out["vLead"] = v_ego + vrel_out
  out["aLeadK"] = a_lead_k
  return out


def load_distances(csv_path: str) -> list[float]:
  out: list[float] = []
  with open(csv_path, newline="") as f:
    for row in csv.DictReader(f):
      d = row.get("lead_dRel") or row.get("lt_dRel")
      if d:
        out.append(float(d))
  return out


def prewarm_filter(filt, vrel: float, d_start: float, frames: int) -> int:
  """Prewarm: smooth closing, then hold (stick-slip setup). Returns last timestamp."""
  t = 1_000_000_000
  d = d_start
  close_frames = int(frames * 0.7)
  hold_frames = frames - close_frames
  for _ in range(close_frames):
    t += int(DT_S * 1e9)
    filt.update(d, t)
    d = max(1.5, d + vrel * DT_S)
  for _ in range(hold_frames):
    t += int(DT_S * 1e9)
    filt.update(d, t)
  return t


def run_chain(distances: list[float], use_fix: bool, v_ego_kph: float,
              prewarm_vrel: float | None = None) -> list[dict]:
  v_ego = v_ego_kph / 3.6
  vrel0 = prewarm_vrel if prewarm_vrel is not None else 0.0
  filt = (_DistanceRateFilter() if use_fix else StockDistanceRateFilter(v_init=vrel0))
  t = prewarm_filter(filt, vrel0, distances[0], PREWARM_FRAMES)
  kf = SimpleLeadKF(v_ego + vrel0)
  rows: list[dict] = []
  for i, d_meas in enumerate(distances):
    t += int(DT_S * 1e9)
    vrel, arel = filt.update(d_meas, t)
    v_lead = v_ego + vrel
    v_lead_k, a_lead_k = kf.update(v_lead)

    lead = {"dRel": d_meas, "vRel": vrel, "vLead": v_lead, "vLeadK": v_lead_k, "aLeadK": a_lead_k}
    if use_fix:
      lead = stabilize_low_speed(lead, kf.cnt, v_ego, measured=False)
      lead = stabilize_position_only(lead, kf.cnt, v_ego, measured=False)

    a_lead_k = lead["aLeadK"]
    # MPC proxy: very negative aLeadK at highway → commands toward floor
    if a_lead_k < -3.0 and d_meas < 120.0:
      mpc_proxy = ACCEL_MIN
    elif a_lead_k < -1.5:
      mpc_proxy = -2.0
    else:
      mpc_proxy = max(-0.5, a_lead_k * 0.3)

    rows.append({
      "i": i, "dRel": d_meas, "lt_vRel": vrel, "lt_aRel": arel,
      "aLeadK": a_lead_k, "mpc_proxy": mpc_proxy, "cnt": kf.cnt,
    })
  return rows


def summarize(label: str, rows: list[dict]) -> None:
  print(f"\n=== {label} ===")
  print(f"  max |lt_aRel| = {max(abs(r['lt_aRel']) for r in rows):.2f} m/s²")
  print(f"  min aLeadK    = {min(r['aLeadK'] for r in rows):.2f} m/s²")
  print(f"  min mpc_proxy = {min(r['mpc_proxy'] for r in rows):.2f} m/s²")
  print(f"  floor frames  = {sum(1 for r in rows if r['mpc_proxy'] <= ACCEL_MIN + 0.05)}")


def run_scenario(label: str, distances: list[float], v_ego_kph: float,
                 prewarm_vrel: float, glitch_d: float | None) -> None:
  zone = "low-speed guards" if v_ego_kph < LOW_SPEED_LEAD_SMOOTH_VEGO * 3.6 else (
    "highway guards" if v_ego_kph >= POSITION_ONLY_HIGHWAY_VEGO * 3.6 else "unguarded gap")
  print(f"\n{'='*72}")
  print(f"{label}")
  print(f"  vEgo={v_ego_kph:.0f} km/h  prewarm vRel={prewarm_vrel:+.1f} m/s  "
        f"d0={distances[0]:.1f}m  radard zone: {zone}")
  stock = run_chain(distances, False, v_ego_kph, prewarm_vrel)
  fixed = run_chain(distances, True, v_ego_kph, prewarm_vrel)
  summarize("STOCK", stock)
  summarize("FIX", fixed)

  print("\n--- Key frames ---")
  print(f"{'i':>3} {'dRel':>6} | {'stk_aR':>7} {'stk_aK':>7} {'stk_mpc':>7} | "
        f"{'fix_aR':>7} {'fix_aK':>7} {'fix_mpc':>7}")
  for s, f in zip(stock, fixed):
    show = (s["aLeadK"] < -1.0 or f["aLeadK"] < -0.5
            or (glitch_d is not None and abs(s["dRel"] - glitch_d) < 0.3))
    if show:
      print(f"{s['i']:3d} {s['dRel']:6.1f} | {s['lt_aRel']:+7.2f} {s['aLeadK']:+7.2f} {s['mpc_proxy']:+7.2f} | "
            f"{f['lt_aRel']:+7.2f} {f['aLeadK']:+7.2f} {f['mpc_proxy']:+7.2f}")


def glitch_sequence(d_hold: float, d_step: float, n_hold: int = 15, n_after: int = 45) -> list[float]:
  return [d_hold] * n_hold + [d_step] * n_after


def main() -> None:
  ap = argparse.ArgumentParser()
  ap.add_argument("csv", nargs="?",
                  default="/home/quangnm/openpilot_vinfast/tools/debug/captures/vf7_brake_20260622_165001_f1847.csv")
  ap.add_argument("--vego-kph", type=float, default=None,
                  help="Single scenario ego speed (default: run comparison sweep)")
  ap.add_argument("--prewarm-vrel", type=float, default=None,
                  help="Closing speed during prewarm [m/s], negative = closing")
  args = ap.parse_args()

  print("Fix: warmup=%d ramp=%d |aRel|<=%.1f sustained<=%.1f" % (
    KIN_WARMUP_KIN_UPDATES, KIN_AREL_RAMP_UPDATES, KIN_AREL_RAMP_MAX, INFO_SUSTAINED_MAX_AREL))

  if args.vego_kph is not None:
    distances = load_distances(args.csv)
    vrel = args.prewarm_vrel if args.prewarm_vrel is not None else -8.5
    run_scenario("custom", distances, args.vego_kph, vrel, distances[min(15, len(distances)-1)])
    return

  capture = load_distances(args.csv)
  # Highway replay (capture): 103 km/h, lead ~72 km/h → vRel ≈ -8.7 m/s
  run_scenario("HIGHWAY capture replay", capture, 103.0, -8.5, 109.8)

  # Moderate: 60 km/h ego, lead ~48 km/h → vRel ≈ -3.3 m/s, typical suburban range
  mod_dist = glitch_sequence(50.0, 49.6)
  run_scenario("MODERATE 60 km/h (50m, -0.4m glitch)", mod_dist, 60.0, -3.3, 49.6)

  # Same glitch at 60 km/h but long range like capture
  mod_long = glitch_sequence(110.2, 109.8)
  run_scenario("MODERATE 60 km/h (110m, -0.4m glitch)", mod_long, 60.0, -3.3, 109.8)

  # City: 30 km/h ego — low-speed radard guards active (<20 km/h threshold is 20, so 30 is above)
  city_dist = glitch_sequence(35.0, 34.6)
  run_scenario("CITY 30 km/h (35m, -0.4m glitch)", city_dist, 30.0, -1.4, 34.6)

  # Very low: 15 km/h — low-speed guards ON
  creep_dist = glitch_sequence(20.0, 19.6)
  run_scenario("CREEP 15 km/h (20m, -0.4m glitch)", creep_dist, 15.0, -0.5, 19.6)

  print(f"\n{'='*72}")
  print("Guard thresholds: low-speed <%.0f km/h | highway >%.0f km/h" % (
    LOW_SPEED_LEAD_SMOOTH_VEGO * 3.6, POSITION_ONLY_HIGHWAY_VEGO * 3.6))


if __name__ == "__main__":
  main()
