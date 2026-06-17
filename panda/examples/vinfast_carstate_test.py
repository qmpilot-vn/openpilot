#!/usr/bin/env python3
"""
Test script for VinFast CarState parsing.

This script:
1. Connects to Panda
2. Receives CAN messages from the chassis bus
3. Parses them using CarState.update()
4. Displays parsed car state values in real-time
"""

import argparse
import sys
import time
from collections import defaultdict

from panda import Panda

from opendbc.car.vinfast.carstate import CarState
from opendbc.car.vinfast.values import CAR, CANBUS
from opendbc.car.structs import CarParams, CarParamsSP

# VinFast CAN buses (see opendbc.car.vinfast.values.CANBUS)
VINFAST_CHASSIS_BUS = CANBUS.chassis


def format_value(value, unit=""):
    """Format a value with unit, handling None and floats."""
    if value is None:
        return "N/A"
    if isinstance(value, float):
        return f"{value:.2f}{unit}"
    return f"{value}{unit}"


def format_wheel_speeds(wheel_speeds):
    """Format wheel speeds for display."""
    if wheel_speeds is None:
        return "N/A, N/A, N/A, N/A"
    return (
        f"{format_value(wheel_speeds.fl, 'm/s')}, "
        f"{format_value(wheel_speeds.fr, 'm/s')}, "
        f"{format_value(wheel_speeds.rl, 'm/s')}, "
        f"{format_value(wheel_speeds.rr, 'm/s')}"
    )


def print_car_state(cs):
    """Print formatted car state information."""
    print("\n" + "=" * 80)
    print("CAR STATE UPDATE")
    print("=" * 80)

    # Vehicle speed
    print(f"Vehicle Speed:     {format_value(cs.vEgo, ' m/s')} "
          f"({format_value(cs.vEgo * 3.6, ' km/h') if cs.vEgo else 'N/A'})")
    print(f"Vehicle Speed Raw: {format_value(cs.vEgoRaw, ' m/s')}")
    print(f"Standstill:        {cs.standstill if hasattr(cs, 'standstill') else 'N/A'}")

    # Steering
    print(f"\nSteering Angle:    {format_value(cs.steeringAngleDeg, ' deg')}")
    print(f"Steering Rate:     {format_value(cs.steeringRateDeg, ' deg/s')}")
    print(f"Steering Torque:   {format_value(cs.steeringTorque, ' Nm')}")
    print(f"Steering Torque EPS: {format_value(cs.steeringTorqueEps, ' Nm')}")
    print(f"Steering Pressed:  {cs.steeringPressed if hasattr(cs, 'steeringPressed') else 'N/A'}")

    # Wheel speeds
    print(f"\nWheel Speeds (FL, FR, RL, RR): {format_wheel_speeds(cs.wheelSpeeds)}")

    # Cruise control
    if hasattr(cs, 'cruiseState'):
        cruise = cs.cruiseState
        print(f"\nCruise Control:")
        print(f"  Available: {cruise.available if hasattr(cruise, 'available') else 'N/A'}")
        print(f"  Enabled:   {cruise.enabled if hasattr(cruise, 'enabled') else 'N/A'}")
        print(f"  Standstill: {cruise.standstill if hasattr(cruise, 'standstill') else 'N/A'}")

    # Gear
    if hasattr(cs, 'gearShifter'):
        gear_str = str(cs.gearShifter) if cs.gearShifter else 'N/A'
        print(f"\nGear: {gear_str}")

    # Doors
    if hasattr(cs, 'doorOpen'):
        print(f"Door Open: {cs.doorOpen}")

    print("=" * 80)


def main():
    parser = argparse.ArgumentParser(description="Test VinFast CarState parsing")
    parser.add_argument(
        "--update-rate",
        type=float,
        default=1.0,
        help="Update display every N seconds (default: 1.0)",
    )
    parser.add_argument(
        "--bus",
        type=int,
        default=VINFAST_CHASSIS_BUS,
        help=f"CAN bus to listen on (default: {VINFAST_CHASSIS_BUS})",
    )
    parser.add_argument(
        "--debug",
        action="store_true",
        help="Print debug information about received messages",
    )
    args = parser.parse_args()

    print("VinFast CarState Test")
    print(f"Listening on CAN bus {args.bus}")
    print(f"Update rate: {args.update_rate} seconds")
    print("\nPress Ctrl+C to exit\n")

    try:
        # Connect to Panda
        print("Connecting to Panda...")
        panda = Panda()
        print("Connected!")

        # Create minimal CarParams for CarState
        # CarState only needs carFingerprint to look up DBC
        CP = CarParams.new_message()
        CP.carFingerprint = "VINFAST_VF8"
        CP.brand = "vinfast"
        CP_SP = CarParamsSP()

        # Get car name from car docs
        car_name = CAR.VINFAST_VF8.config.car_docs[0].name if CAR.VINFAST_VF8.config.car_docs else "VinFast VF8"
        print(f"Car: {car_name}")

        # Initialize CarState
        print("Initializing CarState...")
        car_state = CarState(CP, CP_SP)
        can_parsers = car_state.get_can_parsers(CP, CP_SP)
        print(f"Initialized CAN parsers for buses: {list(can_parsers.keys())}")

        # Message statistics
        msg_counts = defaultdict(int)
        last_update_time = time.time()
        message_buffer = []

        print(f"\nReceiving CAN messages from bus {args.bus}...")
        print("Waiting for messages...\n")

        while True:
            # Receive CAN messages
            can_msgs = panda.can_recv()
            current_time = time.time()

            # Process received messages
            timestamp_nanos = int(current_time * 1e9)  # Convert to nanoseconds
            frames_for_parser = []

            for address, dat, src in can_msgs:
                # Filter by bus (Panda bus numbers)
                if src == args.bus:
                    msg_counts[address] += 1
                    # CANParser expects: (address, data, src_bus)
                    frames_for_parser.append((address, dat, src))

                    if args.debug:
                        print(f"RX: 0x{address:03X} on bus {src}: {dat.hex()}")

            # Update CAN parsers with received messages
            if frames_for_parser:
                # CANParser.update() expects: list of (timestamp, list_of_frames)
                # where frames are (address, data, src_bus)
                # The parser's bus number must match the src in the frames
                from opendbc.car import Bus
                logical_bus = Bus.chassis if args.bus == VINFAST_CHASSIS_BUS else Bus.cam

                if logical_bus in can_parsers:
                    parser = can_parsers[logical_bus]
                    # Update parser with messages from this bus
                    # Parser bus number should match the physical bus number in frames
                    parser.update([(timestamp_nanos, frames_for_parser)])
                    message_buffer.extend(frames_for_parser)

            # Update CarState periodically
            if current_time - last_update_time >= args.update_rate:
                if message_buffer or any(len(parser.vl) > 0 for parser in can_parsers.values()):
                    try:
                        cs, _cs_sp = car_state.update(can_parsers)
                        print_car_state(cs)
                    except Exception as e:
                        print(f"Error updating CarState: {e}")
                        import traceback
                        traceback.print_exc()

                    # Print message statistics
                    if args.debug and msg_counts:
                        print(f"\nMessage counts: {dict(msg_counts)}")
                        msg_counts.clear()

                    # Clear message buffer
                    message_buffer.clear()
                else:
                    print(f"[{current_time:.1f}] No messages received yet...")

                last_update_time = current_time

            # Small sleep to prevent CPU spinning
            time.sleep(0.01)

    except KeyboardInterrupt:
        print("\n\nExiting...")
    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
    finally:
        if 'panda' in locals():
            panda.close()


if __name__ == "__main__":
    main()

