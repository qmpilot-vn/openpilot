import argparse
import collections
import select
import sys
import termios
import threading
import time
import tty

from panda import Panda

from opendbc.can import CANPacker
from opendbc.car import Bus
from opendbc.car.structs import CarParams, CarParamsSP
from opendbc.car.vinfast.vinfastcan import create_acc_control, create_idb_control, create_steering_control
from opendbc.car.vinfast.values import CAR, CANBUS
from opendbc.car.vinfast.carstate import CarState

# VF8 merged stream (per opendbc CANBUS): chassis = 0, ADAS/SCAM = 2, radar FD = 1, info = 4
VINFAST_CHASSIS_BUS = CANBUS.chassis  # 0 — EPS, wheels, IDB, etc.
VINFAST_ADAS_BUS = CANBUS.cam  # 2 — ADAS_ACC_Status and other SCAM traffic when stock-long path

# Steering constraints
STEER_MIN = -120.0
STEER_MAX = 120.0
STEER_STEP = 3.0
# Default: enable lateral immediately (keyboard tester). Use --lat-delay-frames if EPS needs settle time.
DEFAULT_LAT_ACTIVATION_DELAY_FRAMES = 0
FINE_CONTROL_THRESHOLD = 80.0
FINE_CONTROL_STEP = 10.0
CENTER_RETURN_STEP = 2.0
CENTER_TARGET_STEP = 2.0

# Acceleration limits (m/s^2)
ACCEL_MIN = -3.5
ACCEL_MAX = 2.0
ACCEL_STEP = 0.1

# Loop frequency
DESIRED_FREQUENCY = 100.0  # Hz
PERIOD = 1.0 / DESIRED_FREQUENCY

stop_event = threading.Event()

steer_angle_lock = threading.Lock()
steer_angle_target = 0.0
steer_angle_current = 0.0
auto_center_active = False

accel_lock = threading.Lock()
accel_value = 0.0

# CarState display
carstate_lock = threading.Lock()
last_carstate_update = 0.0

# RX thread (optional): keep control loop from blocking on can_recv()
rx_lock = threading.Lock()
rx_queue = collections.deque(maxlen=10000)  # holds (address, dat, src)


def rx_worker(panda: Panda):
    while not stop_event.is_set():
        try:
            msgs = panda.can_recv()
        except Exception:
            continue
        if not msgs:
            continue
        with rx_lock:
            rx_queue.extend(msgs)


def clamp(value: float, min_value: float, max_value: float) -> float:
    return max(min_value, min(value, max_value))


def format_value(value, unit=""):
    """Format a value with unit, handling None and floats."""
    if value is None:
        return "N/A"
    if isinstance(value, float):
        return f"{value:.2f}{unit}"
    return f"{value}{unit}"


def print_car_state_compact(cs):
    """Print compact car state information for real-time display."""
    print("\n" + "─" * 80)
    print("CAR STATE")
    print("─" * 80)

    # Vehicle speed
    speed_kph = cs.vEgo * 3.6 if cs.vEgo else 0.0
    print(f"Speed: {format_value(cs.vEgo, ' m/s')} ({format_value(speed_kph, ' km/h')}) | "
          f"Standstill: {cs.standstill if hasattr(cs, 'standstill') else 'N/A'}")

    # Steering
    print(f"Steering: {format_value(cs.steeringAngleDeg, '°')} | "
          f"Rate: {format_value(cs.steeringRateDeg, '°/s')} | "
          f"Torque: {format_value(cs.steeringTorque, ' Nm')}")

    # Wheel speeds
    if hasattr(cs, 'wheelSpeeds') and cs.wheelSpeeds:
        ws = cs.wheelSpeeds
        print(f"Wheels: FL={format_value(ws.fl, ' m/s')} FR={format_value(ws.fr, ' m/s')} "
              f"RL={format_value(ws.rl, ' m/s')} RR={format_value(ws.rr, ' m/s')}")

    # Cruise control
    if hasattr(cs, 'cruiseState'):
        cruise = cs.cruiseState
        print(f"Cruise: Available={cruise.available if hasattr(cruise, 'available') else 'N/A'} | "
              f"Enabled={cruise.enabled if hasattr(cruise, 'enabled') else 'N/A'}")

    print("─" * 80)


def keyboard_listener():
    """
    Non-blocking keyboard controls:
      - 'a'/'d': steer left/right
      - 's': smooth return to center
      - 'w'/'x': increase/decrease acceleration
      - 'q': quit
    """
    global steer_angle_target, auto_center_active, accel_value

    print(
        "[KEYBOARD] Controls: 'a'=left, 'd'=right, 's'=center, "
        "'w'=accelerate, 'x'=decelerate, 'q'=quit"
    )

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    try:
        tty.setcbreak(fd)
        while not stop_event.is_set():
            rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
            if not rlist:
                continue

            ch = sys.stdin.read(1).lower()

            if ch == "a":
                with steer_angle_lock:
                    auto_center_active = False
                    step = (
                        FINE_CONTROL_STEP
                        if abs(steer_angle_target) <= FINE_CONTROL_THRESHOLD
                        else STEER_STEP
                    )
                    steer_angle_target = clamp(
                        steer_angle_target - step, STEER_MIN, STEER_MAX
                    )
                print(f"[KEYBOARD] Target steer angle: {steer_angle_target:.1f} deg")

            elif ch == "d":
                with steer_angle_lock:
                    auto_center_active = False
                    step = (
                        FINE_CONTROL_STEP
                        if abs(steer_angle_target) <= FINE_CONTROL_THRESHOLD
                        else STEER_STEP
                    )
                    steer_angle_target = clamp(
                        steer_angle_target + step, STEER_MIN, STEER_MAX
                    )
                print(f"[KEYBOARD] Target steer angle: {steer_angle_target:.1f} deg")

            elif ch == "s":
                with steer_angle_lock:
                    auto_center_active = True
                with accel_lock:
                    accel_value = 0.0
                print("[KEYBOARD] Smooth center return enabled, acceleration set to 0")

            elif ch == "w":
                with accel_lock:
                    accel_value = clamp(accel_value + ACCEL_STEP, ACCEL_MIN, ACCEL_MAX)
                print(f"[KEYBOARD] Acceleration: {accel_value:.1f} m/s²")

            elif ch == "x":
                with accel_lock:
                    accel_value = clamp(accel_value - ACCEL_STEP, ACCEL_MIN, ACCEL_MAX)
                print(f"[KEYBOARD] Acceleration: {accel_value:.1f} m/s²")

            elif ch == "q":
                print("[KEYBOARD] Quit requested")
                stop_event.set()
                break

    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


def zmq_listener(connect: str, topic: str):
    """
    Receive steering/accel commands over ZMQ (PUB/SUB), non-blocking.
    Message format (JSON): {"steer_angle_target": float, "accel_value": float, "auto_center": bool}
    """
    global steer_angle_target, auto_center_active, accel_value
    try:
        import zmq  # type: ignore
    except Exception as e:
        print(f"[ZMQ] pyzmq not available: {e}")
        print("[ZMQ] Install with: pip3 install pyzmq")
        stop_event.set()
        return

    ctx = zmq.Context.instance()
    sock = ctx.socket(zmq.SUB)
    sock.connect(connect)
    sock.setsockopt(zmq.SUBSCRIBE, topic.encode("utf-8"))
    sock.setsockopt(zmq.RCVTIMEO, 100)  # ms

    print(f"[ZMQ] Subscribed: connect={connect} topic={topic}")

    while not stop_event.is_set():
        try:
            parts = sock.recv_multipart()
        except Exception:
            continue

        if len(parts) < 2:
            continue

        try:
            payload = parts[1].decode("utf-8")
            msg = __import__("json").loads(payload)
        except Exception:
            continue

        try:
            if "steer_angle_target" in msg:
                with steer_angle_lock:
                    steer_angle_target = float(msg["steer_angle_target"])
            if "auto_center" in msg:
                with steer_angle_lock:
                    auto_center_active = bool(msg["auto_center"])
            if "accel_value" in msg:
                with accel_lock:
                    accel_value = float(msg["accel_value"])
        except Exception:
            continue


def send_loop(panda: Panda, packer: CANPacker, car_state: CarState, can_parsers: dict, CP: CarParams,
              rx_debug: bool = False, carstate_rate: float = 1.0,
              lat_delay_frames: int = DEFAULT_LAT_ACTIVATION_DELAY_FRAMES,
              health_interval_s: float = 0.0,
              max_rx_msgs_per_cycle: int = 500,
              no_rx: bool = False,
              profile: bool = False,
              rx_in_thread: bool = True):
    """
    Main control loop (matches openpilot carcontroller + vinfast.h forwarding):
      - ADAS_EPS_LATE_CON (0x37A) @ 100Hz on chassis (0)
      - ADAS_ACC_Status (0x32D) @ 50Hz — bus from create_acc_control (chassis when OP long)
      - ADAS_IDB (0x131) @ 50Hz on chassis — required to keep IDB alive; when OP long, safety
        blocks stock SCAM->chassis IDB/ACC so the host must send both on chassis (see vinfast_fwd_hook).
      - CarState from chassis RX for standstill / drive-off request
    """
    global steer_angle_current, steer_angle_target, auto_center_active, accel_value
    global last_carstate_update

    frame = 0
    last_health_print = 0.0

    print("[SEND] Starting control loop (100Hz); heartbeat @10Hz in-loop (no background thread)")
    if lat_delay_frames > 0:
        print(
            f"[SEND] Lateral (AOLAct) disabled for first {lat_delay_frames} frames "
            f"(~{lat_delay_frames / 100:.1f}s). Use --immediate-lateral or --lat-delay-frames 0 for instant."
        )

    # Use a monotonic clock for scheduling; wall-clock time can jump (NTP) and cause apparent "lag".
    next_frame_time = time.monotonic()
    next_hb_time = next_frame_time
    next_long_time = next_frame_time  # 50Hz ACC/IDB cadence
    prof_last_print = next_frame_time
    prof = {
        "loops": 0,
        "t_total_max_ms": 0.0,
        "t_rx_max_ms": 0.0,
        "t_parse_max_ms": 0.0,
        "t_cs_max_ms": 0.0,
        "t_send_max_ms": 0.0,
    }

    try:
        while not stop_event.is_set():
            loop_t0 = time.monotonic()
            # 1. CRITICAL: Drain RX without blocking the 100Hz loop.
            t0 = time.monotonic()
            if no_rx:
                incoming_messages = []
            elif rx_in_thread:
                with rx_lock:
                    incoming_messages = list(rx_queue)
                    rx_queue.clear()
            else:
                incoming_messages = panda.can_recv()
            t_rx_ms = (time.monotonic() - t0) * 1000.0

            # Parse incoming messages for CarState
            now = time.monotonic()
            timestamp_nanos = time.monotonic_ns()
            frames_for_parser = []

            if max_rx_msgs_per_cycle > 0 and len(incoming_messages) > max_rx_msgs_per_cycle:
                # If we spend too long parsing RX, the 100Hz control loop will miss deadlines.
                # Keeping the *most recent* frames is generally what matters for CarState.
                incoming_messages = incoming_messages[-max_rx_msgs_per_cycle:]

            t0 = time.monotonic()
            for address, dat, src in incoming_messages:
                # Collect messages from chassis bus for parsing
                if src == VINFAST_CHASSIS_BUS:
                    frames_for_parser.append((address, dat, src))

                if rx_debug:
                    # Filter for specific IDs to avoid flooding console
                    if address in (0x37A, 0x132, 0x32D):
                        print(f"[RX] ID: 0x{address:X}, Bus: {src}, Data: {bytes(dat).hex()}")

            # Update CAN parsers with received messages
            if frames_for_parser and can_parsers:
                if Bus.chassis in can_parsers:
                    parser = can_parsers[Bus.chassis]
                    parser.update([(timestamp_nanos, frames_for_parser)])
            t_parse_ms = (time.monotonic() - t0) * 1000.0

            # CarState every frame (standstill for ACC / IDB); print at carstate_rate
            standstill = False
            if car_state and can_parsers and Bus.chassis in can_parsers:
                try:
                    t0 = time.monotonic()
                    cs, _cs_sp = car_state.update(can_parsers)
                    t_cs_ms = (time.monotonic() - t0) * 1000.0
                    standstill = bool(cs.out.standstill)
                    if carstate_rate > 0 and (now - last_carstate_update) >= carstate_rate:
                        print_car_state_compact(cs)
                        last_carstate_update = now
                except Exception as e:
                    if rx_debug:
                        print(f"[CARSTATE] Error updating: {e}")
                    t_cs_ms = 0.0
            else:
                t_cs_ms = 0.0

            # 2. Control Logic
            with accel_lock:
                accel_cmd = accel_value

            with steer_angle_lock:
                # [Logic remains the same as your original script...]
                if auto_center_active:
                    if abs(steer_angle_target) <= CENTER_TARGET_STEP:
                        steer_angle_target = 0.0
                        auto_center_active = False
                    else:
                        steer_angle_target -= (
                            CENTER_TARGET_STEP
                            if steer_angle_target > 0
                            else -CENTER_TARGET_STEP
                        )

                error = steer_angle_target - steer_angle_current
                if error != 0.0:
                    if steer_angle_target == 0.0:
                        step = (
                            FINE_CONTROL_STEP
                            if abs(steer_angle_current) <= FINE_CONTROL_THRESHOLD
                            else CENTER_RETURN_STEP
                        )
                    else:
                        step = (
                            FINE_CONTROL_STEP
                            if abs(steer_angle_current) <= FINE_CONTROL_THRESHOLD
                            else STEER_STEP
                        )

                    if abs(error) <= step:
                        steer_angle_current = steer_angle_target
                    else:
                        steer_angle_current += step if error > 0 else -step

                apply_angle = clamp(steer_angle_current, STEER_MIN, STEER_MAX)

            lat_active = frame >= lat_delay_frames
            long_active = True

            # 3. Packing & Sending
            eps_addr, eps_data, _ = create_steering_control(
                packer, CP, frame, -apply_angle, lat_active
            )

            # Steering is nominally 100Hz; keep sending it every cycle.
            # Longitudinal (ACC/IDB) should be 50Hz based on *time*, not frame count, so lag doesn't
            # accidentally drop the cadence and trip safety timeouts.
            t0 = time.monotonic()
            if now >= next_long_time:
                acc_frame = frame // 2  # still used for counters inside the message definitions
                acc_addr, acc_data, acc_bus = create_acc_control(
                    packer, CP, acc_frame, accel_cmd, long_active, standstill,
                )
                drive_off_request = 1 if (standstill and accel_cmd > 0.01) else 0
                idb_addr, idb_data, _idb_bus = create_idb_control(
                    packer, acc_frame, drive_off_request, bus=VINFAST_CHASSIS_BUS
                )
                # One SPI/USB transaction for steering + ACC + IDB reduces contention vs three sends.
                panda.can_send_many(
                    [
                        (eps_addr, eps_data, VINFAST_CHASSIS_BUS),
                        (acc_addr, acc_data, acc_bus),
                        (idb_addr, idb_data, VINFAST_CHASSIS_BUS),
                    ]
                )
                # Keep 50Hz cadence with bounded catch-up.
                next_long_time += 0.02
                if next_long_time < now - 0.1:
                    next_long_time = now
            else:
                panda.can_send(eps_addr, eps_data, VINFAST_CHASSIS_BUS)

            # Heartbeat must stay ~10Hz for engaged controls; schedule by time so loop lag doesn't drop it.
            if now >= next_hb_time:
                panda.send_heartbeat(engaged=True)
                next_hb_time += 0.1
                if next_hb_time < now - 0.5:
                    next_hb_time = now
            t_send_ms = (time.monotonic() - t0) * 1000.0

            if health_interval_s > 0 and (now - last_health_print) >= health_interval_s:
                try:
                    h = panda.health()
                    print(
                        "[HEALTH] "
                        f"controls_allowed={h['controls_allowed']} safety_mode={h['safety_mode']} "
                        f"safety_param={h['safety_param']} tx_blocked={h['safety_tx_blocked']} "
                        f"heartbeat_lost={h['heartbeat_lost']} faults={h['faults']}"
                    )
                except Exception as e:
                    print(f"[HEALTH] read failed: {e}")
                last_health_print = now

            frame += 1
            loop_ms = (time.monotonic() - loop_t0) * 1000.0
            if profile:
                prof["loops"] += 1
                prof["t_total_max_ms"] = max(prof["t_total_max_ms"], loop_ms)
                prof["t_rx_max_ms"] = max(prof["t_rx_max_ms"], t_rx_ms)
                prof["t_parse_max_ms"] = max(prof["t_parse_max_ms"], t_parse_ms)
                prof["t_cs_max_ms"] = max(prof["t_cs_max_ms"], t_cs_ms)
                prof["t_send_max_ms"] = max(prof["t_send_max_ms"], t_send_ms)
                if now - prof_last_print >= 1.0:
                    hz = prof["loops"] / max(1e-6, (now - prof_last_print))
                    print(
                        "[PROFILE] "
                        f"loop_hz={hz:.1f} "
                        f"max_total={prof['t_total_max_ms']:.1f}ms "
                        f"max_rx={prof['t_rx_max_ms']:.1f}ms "
                        f"max_parse={prof['t_parse_max_ms']:.1f}ms "
                        f"max_cs={prof['t_cs_max_ms']:.1f}ms "
                        f"max_send={prof['t_send_max_ms']:.1f}ms"
                    )
                    prof_last_print = now
                    prof.update(
                        loops=0,
                        t_total_max_ms=0.0,
                        t_rx_max_ms=0.0,
                        t_parse_max_ms=0.0,
                        t_cs_max_ms=0.0,
                        t_send_max_ms=0.0,
                    )

            # 4. Accurate Timing Loop
            # Calculate how much sleep is needed to hit exactly the next 10ms mark
            next_frame_time += PERIOD
            sleep_time = next_frame_time - time.monotonic()

            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                # If we are falling behind, reset the clock to avoid trying to catch up fast
                # This prevents "dense" bursts if the computer lags
                if frame % 100 == 0:
                    print(f"[WARN] Lagging behind by {-sleep_time*1000:.2f}ms")
                next_frame_time = time.monotonic()

    except KeyboardInterrupt:
        print("[SEND] KeyboardInterrupt")
        stop_event.set()
    except Exception as e:
        print(f"[SEND] Error: {e}")
        import traceback
        traceback.print_exc()


def parse_args():
    parser = argparse.ArgumentParser(description="VinFast keyboard control tester")
    parser.add_argument(
        "--safety",
        choices=["vinfast", "alloutput"],
        default="vinfast",
        help="Panda safety mode to use",
    )
    long = parser.add_mutually_exclusive_group()
    long.add_argument(
        "--op-long",
        action="store_true",
        default=None,
        help="ACC on chassis bus (0); matches vinfast safety longitudinal bit",
    )
    long.add_argument(
        "--stock-long",
        action="store_true",
        help="ACC on ADAS bus (2); use if car expects SCAM path",
    )
    parser.add_argument(
        "--serial",
        default=None,
        help="Panda serial (hex). Use output of --list-pandas. SPI and USB serials differ.",
    )
    parser.add_argument(
        "--list-pandas",
        action="store_true",
        help="Print USB and SPI panda serials and exit",
    )
    parser.add_argument(
        "--rx-debug",
        action="store_true",
        help="Print received CAN frames for debugging",
    )
    parser.add_argument(
        "--carstate-rate",
        type=float,
        default=0.0,
        help="CarState display update rate in seconds (default: 0.0 = disable printing)",
    )
    parser.add_argument(
        "--lat-delay-frames",
        type=int,
        default=DEFAULT_LAT_ACTIVATION_DELAY_FRAMES,
        metavar="N",
        help=f"Wait N frames at 100Hz before enabling lateral (AOLAct). Default {DEFAULT_LAT_ACTIVATION_DELAY_FRAMES}.",
    )
    parser.add_argument(
        "--immediate-lateral",
        action="store_true",
        help="Same as --lat-delay-frames 0 (steering enable from first frame).",
    )
    parser.add_argument(
        "--standalone-params",
        action="store_true",
        help="Do not read CarParams/CarParamsSP from disk; use alternativeExperience=0 and safetyParamSP=0. "
        "Use if Params are stale or cause flaky safety.",
    )
    parser.add_argument(
        "--health-interval",
        type=float,
        default=0.0,
        metavar="SEC",
        help="If >0, print panda health (controls_allowed, safety_mode, …) every SEC seconds.",
    )
    parser.add_argument(
        "--max-rx-msgs-per-cycle",
        type=int,
        default=500,
        metavar="N",
        help="Cap RX messages processed per 100Hz cycle to avoid missing deadlines (default: 500). "
        "Lower this if you see periodic 100-200ms lag spikes on slower hardware.",
    )
    parser.add_argument(
        "--no-rx",
        action="store_true",
        help="Do not call can_recv() / CarState parsing (for isolating timing issues).",
    )
    parser.add_argument(
        "--profile",
        action="store_true",
        help="Print 1Hz loop timing breakdown (max per section).",
    )
    parser.add_argument(
        "--rx-in-thread",
        action="store_true",
        default=True,
        help="Use a background RX thread so can_recv() can't stall the 100Hz loop (default: enabled).",
    )
    parser.add_argument(
        "--no-rx-thread",
        dest="rx_in_thread",
        action="store_false",
        help="Disable background RX thread (debug).",
    )
    parser.add_argument(
        "--zmq-sub",
        default=None,
        metavar="ENDPOINT",
        help="If set, receive steering/accel commands from a ZMQ PUB endpoint "
        "(e.g. tcp://127.0.0.1:5557). Disables local keyboard listener.",
    )
    parser.add_argument(
        "--zmq-topic",
        default="vf",
        help="ZMQ topic to subscribe to (default: vf).",
    )
    return parser.parse_args()


def print_panda_topology():
    """
    USB pandas: external (e.g. red panda on USB OTG).
    SPI pandas: internal to comma device (cuatro/tres on /dev/spidev0.0) — Panda() tries USB first,
    so use --serial <spi_serial> to talk to the internal panda when a USB panda is also plugged in.
    """
    usb = Panda.usb_list()
    spi = Panda.spi_list()
    print("[PANDA] USB serials (external):", usb if usb else "(none)")
    print("[PANDA] SPI serials (internal cuatro/tres):", spi if spi else "(none)")
    return usb, spi


def main():
    args = parse_args()
    lat_delay = 0 if args.immediate_lateral else max(0, int(args.lat_delay_frames))
    print_panda_topology()
    if args.list_pandas:
        return

    panda = None
    keyboard_thread = None
    zmq_thread = None
    rx_thread = None

    try:
        print("[MAIN] Connecting to Panda...")
        panda = Panda(serial=args.serial, cli=args.serial is None)

        try:
            panda.set_power_save(False)
        except Exception as e:
            print(f"[MAIN] set_power_save failed (not critical): {e}")

        link = "SPI (internal)" if panda.spi else "USB"
        print(f"[MAIN] Connected via {link}, serial={panda.get_serial()}")

        # Initialize CANPacker
        dbc_name = CAR.VINFAST_VF8.config.dbc_dict[Bus.chassis]
        packer = CANPacker(dbc_name)
        print(f"[MAIN] Using DBC: {dbc_name}")

        # Initialize CarState for parsing incoming messages
        CP = CarParams.new_message()
        CP.carFingerprint = "VINFAST_VF8"
        CP.brand = "vinfast"
        # create_acc_control bus: chassis if OP long, else ADAS (2)
        if args.stock_long:
            CP.openpilotLongitudinalControl = False
        elif args.op_long:
            CP.openpilotLongitudinalControl = True
        else:
            CP.openpilotLongitudinalControl = args.safety == "vinfast"
        acc_bus = VINFAST_CHASSIS_BUS if CP.openpilotLongitudinalControl else VINFAST_ADAS_BUS
        print(
            f"[MAIN] openpilotLongitudinalControl={CP.openpilotLongitudinalControl} "
            f"→ ADAS_ACC_Status on bus {acc_bus} (chassis=0, ADAS=2)"
        )

        CP_SP = CarParamsSP()

        car_state = CarState(CP, CP_SP)
        can_parsers = car_state.get_can_parsers(CP, CP_SP)
        print(f"[MAIN] CarState initialized, parsing on bus {VINFAST_CHASSIS_BUS}")
        print(f"[MAIN] CarState display rate: {args.carstate_rate} seconds")

        # Panda safety — same order as selfdrive/pandad/panda_safety.cc apply_safety_to_pandas():
        #   1) set_alternative_experience(alternativeExperience, safetyParam from CarParamsSP)
        #   2) set_safety_mode(safetyModel, safetyParam from SafetyConfig)
        #   3) then heartbeat (pandad: process_panda_state after configureSafetyMode)
        safety_mode = (
            CarParams.SafetyModel.vinfast
            if args.safety == "vinfast"
            else CarParams.SafetyModel.allOutput
        )
        VINFAST_PARAM_LONGITUDINAL = 1  # bit 0 — openpilot longitudinal (matches interface._get_params)
        safety_param = VINFAST_PARAM_LONGITUDINAL if args.safety == "vinfast" else 0

        alt_exp, sp_safety = 0, 0
        if args.standalone_params:
            print("[MAIN] --standalone-params: alternativeExperience=0, CarParamsSP.safetyParam=0")
        else:
            try:
                from openpilot.common.params import Params as OPParams
                from cereal import car as Ccar, custom
                import cereal.messaging as messaging
                praw = OPParams().get("CarParams")
                spraw = OPParams().get("CarParamsSP")
                if praw:
                    cp = messaging.log_from_bytes(praw, Ccar.CarParams)
                    alt_exp = int(cp.alternativeExperience)
                if spraw:
                    cpsp = messaging.log_from_bytes(spraw, custom.CarParamsSP)
                    sp_safety = int(cpsp.safetyParam)
            except Exception as e:
                print(f"[MAIN] Optional CarParams alt exp (using 0,0): {e}")

        print(
            f"[MAIN] set_alternative_experience({alt_exp}, {sp_safety}) then "
            f"safety_mode={args.safety} param={safety_param}"
        )
        panda.set_alternative_experience(alt_exp, sp_safety)
        panda.set_safety_mode(safety_mode, param=safety_param)
        panda.send_heartbeat(engaged=True)

        if not args.no_rx and args.rx_in_thread:
            rx_thread = threading.Thread(target=rx_worker, args=(panda,), name="rx-worker", daemon=True)
            rx_thread.start()

        # Start control input source
        if args.zmq_sub:
            zmq_thread = threading.Thread(
                target=zmq_listener, args=(args.zmq_sub, args.zmq_topic),
                name="zmq-listener", daemon=True
            )
            zmq_thread.start()
        else:
            keyboard_thread = threading.Thread(
                target=keyboard_listener, name="keyboard-listener", daemon=True
            )
            keyboard_thread.start()

        # Run send loop
        send_loop(
            panda,
            packer,
            car_state,
            can_parsers,
            CP,
            rx_debug=args.rx_debug,
            carstate_rate=args.carstate_rate,
            lat_delay_frames=lat_delay,
            health_interval_s=args.health_interval,
            max_rx_msgs_per_cycle=args.max_rx_msgs_per_cycle,
            no_rx=args.no_rx,
            profile=args.profile,
            rx_in_thread=args.rx_in_thread,
        )

    finally:
        stop_event.set()
        if rx_thread:
            rx_thread.join(timeout=0.5)
        if zmq_thread:
            zmq_thread.join(timeout=0.5)
        if keyboard_thread:
            keyboard_thread.join(timeout=0.5)
        if panda:
            try:
                panda.set_safety_mode(CarParams.SafetyModel.silent)
            except Exception:
                pass

        print("[MAIN] Shutdown complete")


if __name__ == "__main__":
    main()
