#!/usr/bin/env python3
import argparse
import time

from panda import Panda

from opendbc.can import CANPacker
from opendbc.car import Bus
from opendbc.car.structs import CarParams
from opendbc.car.vinfast.values import CAR, CANBUS
from opendbc.car.vinfast.vinfastcan import create_steering_control


VINFAST_CHASSIS_BUS = CANBUS.chassis  # 0


def clamp(v: float, vmin: float, vmax: float) -> float:
  return max(vmin, min(vmax, v))


def parse_args() -> argparse.Namespace:
  p = argparse.ArgumentParser(description="Minimal VinFast steering-only tester (VinFast safety).")
  p.add_argument("--serial", default=None, help="Panda serial. Use panda lists to find SPI/USB.")
  p.add_argument("--list-pandas", action="store_true", help="List Panda USB/SPI serials and exit.")

  p.add_argument("--angle", type=float, default=0.0, help="Target steering angle in deg (sign per create_steering_control).")
  p.add_argument("--rate-hz", type=float, default=100.0, help="Steering message rate (default: 100Hz).")
  p.add_argument("--duration", type=float, default=10.0, help="Seconds to run (default: 10).")
  p.add_argument("--ramp", type=float, default=1.0, help="Seconds to ramp from 0 to target angle (default: 1).")

  p.add_argument("--sweep", action="store_true", help="Sweep angle between -angle and +angle.")
  p.add_argument("--sweep-period", type=float, default=4.0, help="Sweep period seconds (default: 4). Requires --sweep.")
  return p.parse_args()


def main() -> None:
  args = parse_args()

  usb = Panda.usb_list()
  spi = Panda.spi_list()
  print("[PANDA] USB serials:", usb if usb else "(none)")
  print("[PANDA] SPI serials:", spi if spi else "(none)")
  if args.list_pandas:
    return

  print("[MAIN] Connecting to Panda...")
  panda = Panda(serial=args.serial, cli=args.serial is None)

  try:
    panda.set_power_save(False)
  except Exception as e:
    print(f"[MAIN] set_power_save failed (not critical): {e}")

  link = "SPI (internal)" if panda.spi else "USB"
  print(f"[MAIN] Connected via {link}, serial={panda.get_serial()}")

  # Build minimal CP/packer for VF8 EPS message.
  CP = CarParams.new_message()
  CP.carFingerprint = "VINFAST_VF8"
  CP.brand = "vinfast"
  CP.openpilotLongitudinalControl = False  # steering-only test; don't request long

  dbc_name = CAR.VINFAST_VF8.config.dbc_dict[Bus.chassis]
  packer = CANPacker(dbc_name)
  print(f"[MAIN] Using DBC: {dbc_name}")

  # VinFast safety; param bit0 is "OP longitudinal" in the main script, keep it 0 here.
  panda.set_alternative_experience(0, 0)
  panda.set_safety_mode(CarParams.SafetyModel.vinfast, param=0)

  start = time.monotonic()
  next_steer_t = start
  next_hb_t = start
  period = 1.0 / float(args.rate_hz)

  print(
    f"[RUN] Steering-only test: rate={args.rate_hz:.1f}Hz duration={args.duration:.1f}s "
    f"{'SWEEP' if args.sweep else 'HOLD'} target_angle={args.angle:.1f}deg"
  )

  frame = 0
  try:
    while True:
      now = time.monotonic()
      if now - start >= args.duration:
        break

      # Heartbeat at 10Hz, time-based.
      if now >= next_hb_t:
        panda.send_heartbeat(engaged=True)
        next_hb_t += 0.1
        if next_hb_t < now - 0.5:
          next_hb_t = now

      # Steering at requested rate, time-based.
      if now >= next_steer_t:
        if args.sweep:
          # Triangle wave in [-angle, +angle]
          a = abs(float(args.angle))
          if a <= 0.0:
            target = 0.0
          else:
            phase = ((now - start) / max(0.1, args.sweep_period)) % 1.0
            tri = (4.0 * phase - 1.0) if phase < 0.5 else (-4.0 * phase + 3.0)
            target = clamp(tri * a, -a, a)
        else:
          # Ramp from 0 to target over args.ramp seconds to avoid step input.
          if args.ramp <= 0.0:
            target = float(args.angle)
          else:
            alpha = clamp((now - start) / args.ramp, 0.0, 1.0)
            target = alpha * float(args.angle)

        eps_addr, eps_data, _ = create_steering_control(
          packer, CP, frame, -target, True
        )
        panda.can_send(eps_addr, eps_data, VINFAST_CHASSIS_BUS)

        frame += 1
        next_steer_t += period
        if next_steer_t < now - 0.2:
          # If we stalled, drop backlog rather than burst.
          next_steer_t = now

      # Lightweight sleep until next event.
      sleep_until = min(next_steer_t, next_hb_t)
      dt = sleep_until - time.monotonic()
      if dt > 0:
        time.sleep(min(dt, 0.01))

  except KeyboardInterrupt:
    pass
  finally:
    try:
      panda.send_heartbeat(engaged=False)
    except Exception:
      pass
    try:
      panda.set_safety_mode(CarParams.SafetyModel.silent)
    except Exception:
      pass

  print("[MAIN] Done.")


if __name__ == "__main__":
  main()

