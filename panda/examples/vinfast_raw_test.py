#!/usr/bin/env python3
"""
Simple script to send raw CAN message 0x123 with data "00112233" on bus 0 at 100Hz
using Panda safety allOutput mode.
"""

import signal
import sys
import time
from panda import Panda
from opendbc.car.structs import CarParams

# Message configuration
MSG_ID = 0x123
MSG_DATA = bytes.fromhex("00112233")  # "00112233" as hex bytes
BUS = 0  # Bus 0 (chassis)
FREQUENCY = 100.0  # Hz
PERIOD = 1.0 / FREQUENCY

stop_event = False

def signal_handler(sig, frame):
    global stop_event
    print("\n[MAIN] Received interrupt signal, stopping...")
    stop_event = True

def main():
    global stop_event

    # Setup signal handler for graceful shutdown
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    print("[MAIN] Connecting to Panda...")
    try:
        panda = Panda()
        print(f"[MAIN] Connected to Panda: {panda.get_serial()}")
    except Exception as e:
        print(f"[MAIN] Error connecting to Panda: {e}")
        return 1

    # Set safety mode to allOutput (allows all messages)
    print("[MAIN] Setting safety mode to allOutput...")
    try:
        panda.set_safety_mode(CarParams.SafetyModel.allOutput)
        print("[MAIN] Safety mode set to allOutput")
    except Exception as e:
        print(f"[MAIN] Error setting safety mode: {e}")
        panda.close()
        return 1

    # Enable CAN loopback mode
    print("[MAIN] Enabling CAN loopback mode...")
    try:
        panda.set_can_loopback(False)
        print("[MAIN] CAN loopback mode enabled")
    except Exception as e:
        print(f"[MAIN] Error enabling loopback mode: {e}")
        panda.close()
        return 1

    # Send message loop
    print(f"[MAIN] Starting to send message 0x{MSG_ID:03X} with data {MSG_DATA.hex()} on bus {BUS} at {FREQUENCY}Hz")
    print("[MAIN] Press Ctrl+C to stop")

    frame = 0
    start_time = time.time()

    try:
        while not stop_event:
            loop_start = time.time()

            # Send the message
            try:
                panda.can_send(MSG_ID, MSG_DATA, BUS)
                frame += 1

                # Print status every second
                if frame % int(FREQUENCY) == 0:
                    elapsed = time.time() - start_time
                    actual_freq = frame / elapsed if elapsed > 0 else 0
                    print(f"[SEND] Sent {frame} messages, elapsed: {elapsed:.1f}s, actual freq: {actual_freq:.1f}Hz")
            except Exception as e:
                print(f"[ERROR] Failed to send message: {e}")
                break

            # Sleep to maintain frequency
            elapsed = time.time() - loop_start
            if elapsed < PERIOD:
                time.sleep(PERIOD - elapsed)
            else:
                # Warn if we're running too slow
                if frame % int(FREQUENCY) == 0:
                    print(f"[WARN] Loop took {elapsed*1000:.1f}ms (target: {PERIOD*1000:.1f}ms)")

    except KeyboardInterrupt:
        print("\n[MAIN] Keyboard interrupt received")
    finally:
        # Disable CAN loopback mode
        print("\n[MAIN] Disabling CAN loopback mode...")
        try:
            panda.set_can_loopback(False)
            print("[MAIN] CAN loopback mode disabled")
        except Exception as e:
            print(f"[MAIN] Error disabling loopback mode: {e}")

        # Reset to silent safety mode
        print("[MAIN] Resetting Panda to SILENT safety mode...")
        try:
            panda.set_safety_mode(CarParams.SafetyModel.silent)
            print("[MAIN] Safety mode reset to SILENT")
        except Exception as e:
            print(f"[MAIN] Error resetting safety mode: {e}")

        panda.close()
        print("[MAIN] Disconnected from Panda")

        if frame > 0:
            elapsed = time.time() - start_time
            avg_freq = frame / elapsed if elapsed > 0 else 0
            print(f"[MAIN] Total: {frame} messages sent in {elapsed:.2f}s (avg: {avg_freq:.1f}Hz)")

    return 0

if __name__ == "__main__":
    sys.exit(main())

