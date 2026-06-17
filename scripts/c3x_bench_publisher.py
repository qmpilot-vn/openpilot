#!/usr/bin/env python3
"""
C3X bench publisher — fake can, pandaStates, and peripheralState on msgq.

Used with launch_c3x_bench.sh so openpilot can go onroad without a panda or real car.
Simulates Honda Civic 2022 (matches FINGERPRINT=HONDA_CIVIC_2022 / SimulatedCar).

PID file: /tmp/c3x_bench_publisher.pid
Log:      /tmp/c3x_bench_publisher.log (when started by launch_c3x_bench.sh)
"""
from __future__ import annotations

import os
import signal
import sys

# openpilot root (scripts/ -> parent)
_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if _ROOT not in sys.path:
  sys.path.insert(0, _ROOT)

import cereal.messaging as messaging
from cereal import log
from openpilot.common.realtime import Ratekeeper
from openpilot.tools.sim.lib.common import SimulatorState, vec3
from openpilot.tools.sim.lib.simulated_car import SimulatedCar

PID_FILE = "/tmp/c3x_bench_publisher.pid"
PERIPHERAL_EVERY_N_FRAMES = 25


def _write_pid() -> None:
  with open(PID_FILE, "w") as f:
    f.write(str(os.getpid()))


def _remove_pid() -> None:
  try:
    os.remove(PID_FILE)
  except FileNotFoundError:
    pass


def main() -> int:
  state = SimulatorState()
  state.valid = True
  state.ignition = True
  state.velocity = vec3(10.0, 0.0, 0.0)

  car = SimulatedCar()
  pm = messaging.PubMaster(["peripheralState"])
  rk = Ratekeeper(100)

  def _shutdown(*_args) -> None:
    _remove_pid()
    sys.exit(0)

  signal.signal(signal.SIGTERM, _shutdown)
  signal.signal(signal.SIGINT, _shutdown)

  _write_pid()
  print("c3x_bench_publisher running (Honda Civic 2022 sim)", flush=True)

  while True:
    car.update(state)
    if rk.frame % PERIPHERAL_EVERY_N_FRAMES == 0:
      dat = messaging.new_message("peripheralState")
      dat.valid = True
      dat.peripheralState.pandaType = log.PandaState.PandaType.blackPanda
      dat.peripheralState.voltage = 12000
      dat.peripheralState.fanSpeedRpm = 1000
      pm.send("peripheralState", dat)
    rk.keep_time()


if __name__ == "__main__":
  raise SystemExit(main())
