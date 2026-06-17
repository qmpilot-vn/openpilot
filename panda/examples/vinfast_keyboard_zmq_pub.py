#!/usr/bin/env python3
import argparse
import json
import select
import sys
import termios
import time
import tty


def clamp(v: float, vmin: float, vmax: float) -> float:
  return max(vmin, min(vmax, vmax if v > vmax else v))


STEER_MIN = -120.0
STEER_MAX = 120.0
STEER_STEP = 3.0
FINE_CONTROL_THRESHOLD = 80.0
FINE_CONTROL_STEP = 10.0
CENTER_TARGET_STEP = 2.0

ACCEL_MIN = -3.5
ACCEL_MAX = 2.0
ACCEL_STEP = 0.1


def parse_args() -> argparse.Namespace:
  p = argparse.ArgumentParser(description="Keyboard -> ZMQ publisher for vinfast_keyboard_control.py")
  p.add_argument("--bind", default="tcp://*:5557", help="ZMQ bind endpoint (PUB). Default tcp://*:5557")
  p.add_argument("--topic", default="vf", help="ZMQ topic prefix (default: vf)")
  return p.parse_args()


def main() -> None:
  args = parse_args()
  try:
    import zmq  # type: ignore
  except Exception as e:
    print(f"[ZMQ] pyzmq not available: {e}")
    print("[ZMQ] Install with: pip3 install pyzmq")
    sys.exit(1)

  ctx = zmq.Context.instance()
  sock = ctx.socket(zmq.PUB)
  sock.bind(args.bind)

  steer_target = 0.0
  accel = 0.0
  auto_center = False

  print(
    "[KEYBOARD->ZMQ] Controls: 'a'=left, 'd'=right, 's'=center, "
    "'w'=accelerate, 'x'=decelerate, 'q'=quit"
  )
  print(f"[KEYBOARD->ZMQ] PUB bind={args.bind} topic={args.topic}")

  fd = sys.stdin.fileno()
  old_settings = termios.tcgetattr(fd)
  last_send = 0.0

  def send_state(force: bool = False) -> None:
    nonlocal last_send
    now = time.monotonic()
    if not force and (now - last_send) < 0.02:
      return
    msg = {
      "steer_angle_target": steer_target,
      "accel_value": accel,
      "auto_center": auto_center,
      "t": now,
    }
    sock.send_multipart([args.topic.encode("utf-8"), json.dumps(msg).encode("utf-8")])
    last_send = now

  try:
    tty.setcbreak(fd)
    send_state(force=True)
    while True:
      rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
      if not rlist:
        continue

      ch = sys.stdin.read(1).lower()
      if ch == "a":
        auto_center = False
        step = FINE_CONTROL_STEP if abs(steer_target) <= FINE_CONTROL_THRESHOLD else STEER_STEP
        steer_target = max(STEER_MIN, steer_target - step)
        print(f"[KEYBOARD->ZMQ] steer_target={steer_target:.1f}")
        send_state(force=True)
      elif ch == "d":
        auto_center = False
        step = FINE_CONTROL_STEP if abs(steer_target) <= FINE_CONTROL_THRESHOLD else STEER_STEP
        steer_target = min(STEER_MAX, steer_target + step)
        print(f"[KEYBOARD->ZMQ] steer_target={steer_target:.1f}")
        send_state(force=True)
      elif ch == "s":
        auto_center = True
        accel = 0.0
        print("[KEYBOARD->ZMQ] auto_center=1 accel=0")
        send_state(force=True)
      elif ch == "w":
        accel = min(ACCEL_MAX, accel + ACCEL_STEP)
        print(f"[KEYBOARD->ZMQ] accel={accel:.1f}")
        send_state(force=True)
      elif ch == "x":
        accel = max(ACCEL_MIN, accel - ACCEL_STEP)
        print(f"[KEYBOARD->ZMQ] accel={accel:.1f}")
        send_state(force=True)
      elif ch == "q":
        print("[KEYBOARD->ZMQ] quit")
        send_state(force=True)
        break

  finally:
    termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    try:
      sock.close(0)
    except Exception:
      pass


if __name__ == "__main__":
  main()

