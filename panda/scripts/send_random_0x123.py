#!/usr/bin/env python3
"""Send random 8-byte CAN frames at a fixed rate (default 50 Hz)."""
import argparse
import os
import time

from opendbc.car.structs import CarParams
from panda import Panda
from panda.python.spi import PandaSpiNackResponse


def main() -> None:
  parser = argparse.ArgumentParser(description="Send random CAN frames at a fixed rate")
  parser.add_argument("--serial", help="panda serial (default: only attached panda or prompt)")
  parser.add_argument("--addr", type=lambda x: int(x, 0), default=0x123, help="CAN ID (default: 0x123)")
  parser.add_argument("--bus", type=int, default=2, help="CAN bus (default: 1)")
  parser.add_argument("--hz", type=float, default=10.0, help="send rate in Hz (default: 50)")
  args = parser.parse_args()

  period = 1.0 / args.hz
  p = Panda(serial=args.serial, cli=args.serial is None)
  p.set_safety_mode(CarParams.SafetyModel.allOutput,1)

  print(f"Sending random 8-byte frames on 0x{args.addr:x} bus {args.bus} at {args.hz} Hz (Ctrl+C to stop)")
  next_tx = time.monotonic()
  while True:
    dat = os.urandom(8)
    while True:
      try:
        p.can_send(args.addr, dat, args.bus)
        break
      except PandaSpiNackResponse:
        time.sleep(0.001)

    next_tx += period
    sleep_for = next_tx - time.monotonic()
    if sleep_for > 0:
      time.sleep(sleep_for)
    else:
      next_tx = time.monotonic()


if __name__ == "__main__":
  main()
