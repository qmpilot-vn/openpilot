#!/usr/bin/env python3
"""
Manually configure Panda for VinFast without MADS.

This is useful when openpilot reports controlsMismatchDetail like:
  panda:    [19, 0, 0, false]   (NOOUTPUT)
  expected: [35, 1, 1024]      (VINFAST, long, alt-exp)

This script sets:
  - alternativeExperience = 0 (no MADS)
  - CarParamsSP safetyParam   = 0
  - safety mode = CarParams.SafetyModel.vinfast
  - safetyParam = 1 (bit0: longitudinal enabled)

Then it prints PandaState via messaging so you can confirm the Panda reports the
new mode/params.

Usage:
  python selfdrive/debug/manual_set_panda_vinfast.py --apply
  python selfdrive/debug/manual_set_panda_vinfast.py --apply --poll 5
"""

import argparse
import time

import cereal.messaging as messaging
from cereal import car
from panda import Panda


def _enum_to_int(x) -> int:
  try:
    return int(x)
  except Exception:
    return int(getattr(x, "raw", 0))


def _print_panda_states() -> bool:
  sm = messaging.SubMaster(["pandaStates"])
  ok = False
  for _ in range(50):  # up to ~5s
    sm.update(100)
    ps = sm["pandaStates"]
    pandas = list(getattr(ps, "pandaStates", []))
    if len(pandas) == 0:
      continue
    ok = True
    for i, pstate in enumerate(pandas):
      print(
        f"panda[{i}]: safetyModel={_enum_to_int(pstate.safetyModel)} "
        f"safetyParam={int(pstate.safetyParam)} altExp={int(pstate.alternativeExperience)} "
        f"controlsAllowed={bool(pstate.controlsAllowed)} rxInvalid={bool(getattr(pstate, 'safetyRxChecksInvalid', False))}"
      )
    break
  return ok


def _print_panda_health(p: Panda, label: str = "") -> None:
  try:
    h = p.health()
    prefix = f"{label}: " if label else ""
    print(prefix + "Panda health:")
    print(prefix + f"  safety_mode={int(h.get('safety_mode', -1))} safety_param={int(h.get('safety_param', -1))} "
          f"altExp={int(h.get('alternative_experience', -1))} controlsAllowed={bool(h.get('controls_allowed', False))} "
          f"rxInvalid={bool(h.get('safety_rx_invalid', False))} rxChecksInvalid={bool(h.get('safety_rx_checks_invalid', False))}")
    print(prefix + f"  ignition_line={bool(h.get('ignition_line', False))} ignition_can={bool(h.get('ignition_can', False))} "
          f"car_harness_status={int(h.get('car_harness_status', -1))}")
  except Exception as e:
    print(f"{label}: failed to read Panda health: {e}")


def main() -> int:
  parser = argparse.ArgumentParser(description="Manually set Panda to VinFast safety mode")
  parser.add_argument("--apply", action="store_true", help="Actually apply changes to Panda")
  parser.add_argument("--poll", type=float, default=0.0, help="Poll pandaStates for N seconds after apply")
  parser.add_argument("--serial", type=str, default=None, help="Panda serial to use (avoid interactive prompt)")
  parser.add_argument("--all", action="store_true", help="Apply to all connected pandas (recommended for multipanda)")
  args = parser.parse_args()

  print("Current pandaStates (if available):")
  _print_panda_states()
  print("-" * 80)

  if not args.apply:
    print("Not applying (dry-run). Re-run with --apply to set Panda.")
    return 0

  VINFAST_LONGITUDINAL_PARAM = 1  # bit 0
  serials = []
  if args.all:
    serials = Panda.list()
    if len(serials) == 0:
      print("No pandas found")
      return 1
  else:
    serials = [args.serial] if args.serial else [None]

  for s in serials:
    print("-" * 80)
    if s is None:
      print("Connecting to Panda over USB/SPI...")
      p = Panda()
      label = "default"
    else:
      print(f"Connecting to Panda serial={s}...")
      p = Panda(serial=s)
      label = s

    _print_panda_health(p, label=f"Before[{label}]")
    print("Setting Panda alternative experience to 0 (no MADS)...")
    p.set_alternative_experience(0, 0)
    print("Setting Panda safety mode to VINFAST with safetyParam=1...")
    p.set_safety_mode(car.CarParams.SafetyModel.vinfast, param=VINFAST_LONGITUDINAL_PARAM)
    try:
      p.send_heartbeat(engaged=True)
    except Exception:
      pass
    _print_panda_health(p, label=f"After[{label}]")

  print("Applied. pandaStates now:")
  if not _print_panda_states():
    print("Warning: no pandaStates received (is the onroad stack running?)")

  if args.poll > 0:
    print(f"Polling for {args.poll:.1f}s...")
    t_end = time.time() + args.poll
    while time.time() < t_end:
      _print_panda_states()
      time.sleep(0.5)

  return 0


if __name__ == "__main__":
  raise SystemExit(main())

