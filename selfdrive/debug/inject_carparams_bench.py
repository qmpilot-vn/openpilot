#!/usr/bin/env python3
"""
Bench-only: write Params that pandad reads for safety (same idea as test_pandad_loopback).

WARNING: If `card` is running, it will overwrite CarParams/CarParamsSP on startup and
during operation. Use only when card is stopped, or for a quick test before card wins.

Does not edit pandad — only Params.

Usage:
  python selfdrive/debug/inject_carparams_bench.py --dry-run
  python selfdrive/debug/inject_carparams_bench.py --vinfast-param 1
"""

from __future__ import annotations

import argparse
import os
import sys

_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
if _ROOT not in sys.path:
  sys.path.insert(0, _ROOT)

import cereal.messaging as messaging
from cereal import car, custom
from openpilot.common.params import Params


def _card_running_from_msgq() -> bool | None:
  sm = messaging.SubMaster(["managerState"])
  for _ in range(50):
    sm.update(100)
    if sm.updated.get("managerState") and sm.valid.get("managerState"):
      try:
        for p in sm["managerState"].processes:
          if p.name == "card" and p.shouldBeRunning:
            return bool(p.running)
      except Exception:
        return None
  return None


def main() -> int:
  ap = argparse.ArgumentParser(description="Bench: inject CarParams + gates for pandad safety test")
  ap.add_argument("--dry-run", action="store_true", help="Print what would be written, no writes")
  ap.add_argument(
    "--skip-card-check",
    action="store_true",
    help="Do not warn when card process appears running",
  )
  ap.add_argument(
    "--vinfast-param",
    type=int,
    default=None,
    help="If set, use vinfast safety model with this safetyParam (e.g. 1 for long bit)",
  )
  args = ap.parse_args()

  if not args.skip_card_check:
    cr = _card_running_from_msgq()
    if cr is True:
      print(
        "WARNING: managerState reports `card` is running. card will overwrite CarParams soon.\n"
        "  Use --skip-card-check to force, or stop openpilot/card for an isolated bench.",
        file=sys.stderr,
      )
      return 2
    if cr is None:
      print("NOTE: could not read managerState; proceeding without card-running confirmation.", file=sys.stderr)

  cp = car.CarParams.new_message()
  sc = car.CarParams.SafetyConfig.new_message()
  if args.vinfast_param is not None:
    sc.safetyModel = car.CarParams.SafetyModel.vinfast
    sc.safetyParam = int(args.vinfast_param)
  else:
    sc.safetyModel = car.CarParams.SafetyModel.allOutput
    sc.safetyParam = 0
  cp.safetyConfigs = [sc]

  cp_sp = custom.CarParamsSP.new_message()
  cp_sp.safetyParam = 0

  blobs = {
    "CarParams": cp.to_bytes(),
    "CarParamsSP": cp_sp.to_bytes(),
  }
  flags = {
    "IsOnroad": True,
    "FirmwareQueryDone": True,
    "ControlsReady": True,
  }

  if args.dry_run:
    print("Would put CarParams + CarParamsSP and bools:", list(blobs.keys()), list(flags.keys()))
    print(f"  safetyModel={sc.safetyModel} safetyParam={sc.safetyParam}")
    return 0

  p = Params()
  for k, v in blobs.items():
    p.put(k, v)
  for k, v in flags.items():
    p.put_bool(k, v)

  print("Wrote Params for pandad bench test (CarParams, CarParamsSP, IsOnroad, FirmwareQueryDone, ControlsReady).")
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
