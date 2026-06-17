from panda import Panda
from opendbc.car.structs import CarParams
from opendbc.can import CANPacker
from opendbc.car import Bus
from opendbc.car.vinfast.vinfastcan import create_steering_control
from opendbc.car.vinfast.values import CAR
import threading
import time

# VinFast message IDs (decimal)
ADAS_EPS_LATE_CON = 0x37A   # 890 - Lateral control (steering angle)
ADAS_LKA = 0x132            # 306 - LKA status message
ADAS_ACC_STATUS = 0x32D     # 813 - ACC status (longitudinal control)

# CAN bus assignments
VINFAST_CHASSIS_BUS = 0
VINFAST_CAMERA_BUS = 2


def heartbeat_thread_10hz(p):
    """Send Panda heartbeat at 10Hz (every 100 ms) to keep safety mode active."""
    while True:
        try:
            p.send_heartbeat(engaged=True)
            time.sleep(0.1)
        except Exception as e:
            print(f"[HEARTBEAT] Error: {e}, stopping heartbeat thread")
            break


def vinfast_alloutput_test():
    """
    Test VinFast car control using SAFETY_ALLOUTPUT mode.
    This mode allows all messages to pass through without safety checks.
    WARNING: Use with extreme caution - no safety limits are enforced!
    """
    p = None
    packer = None
    try:
        print("[MAIN] Connecting to Panda...")
        p = Panda()

        # Optional: make sure it doesn't go into power save
        try:
            p.set_power_save(False)
        except Exception as e:
            print(f"[MAIN] set_power_save failed (not critical): {e}")

        # Create a minimal CarParams object for message creation (not used by create functions)
        CP = None

        # Initialize CANPacker with VinFast DBC
        print("[MAIN] Initializing CANPacker...")
        try:
            dbc_name = CAR.VINFAST_VF8.config.dbc_dict[Bus.chassis]
            packer = CANPacker(dbc_name)
            print(f"[MAIN] Using DBC: {dbc_name}")

            # Test message creation to verify DBC is working
            print("[DEBUG] Testing message creation...")
            test_msg_address, test_msg_data, test_msg_bus = create_steering_control(
                packer, CP, 0, 0.0, True
            )
            print(f"[DEBUG] Test message created successfully:")
            print(f"[DEBUG]   Address: 0x{test_msg_address:x} ({test_msg_address})")
            print(f"[DEBUG]   Data: {test_msg_data.hex()}")
            print(f"[DEBUG]   Bus: {test_msg_bus}")
            print(f"[DEBUG]   Data length: {len(test_msg_data)} bytes")
        except Exception as e:
            print(f"[ERROR] Failed to initialize CANPacker or create test message: {e}")
            import traceback
            traceback.print_exc()
            raise

        # Start heartbeat first so watchdog is always happy
        print("[MAIN] Starting 10Hz heartbeat thread...")
        hb_thread = threading.Thread(target=heartbeat_thread_10hz, args=(p,), daemon=True)
        hb_thread.start()

        # Set SAFETY_ALLOUTPUT mode - allows all messages without safety checks
        print("[MAIN] Setting SAFETY_ALLOUTPUT mode...")
        print("[WARNING] SAFETY_ALLOUTPUT mode has NO safety limits - use with extreme caution!")
        try:
            # Check available safety models
            print(f"[DEBUG] Setting safety mode: {CarParams.SafetyModel.allOutput}")
            p.set_safety_mode(CarParams.SafetyModel.allOutput)
            print("[DEBUG] Safety mode set successfully")

            # Note: get_safety_mode() may not be available in all Panda versions
        except Exception as e:
            print(f"[ERROR] Failed to set safety mode: {e}")
            import traceback
            traceback.print_exc()
            raise

        # One immediate heartbeat
        try:
            p.send_heartbeat(engaged=True)
            print("[DEBUG] Initial heartbeat sent")
        except Exception as e:
            print(f"[ERROR] Failed to send heartbeat: {e}")

        # Control loop configuration
        desired_frequency = 100  # Hz
        period = 1.0 / desired_frequency

        # Control state
        frame = 0
        lat_active = True   # Lateral control active flag
        apply_angle = 0.0   # Steering angle in degrees (0 = center, positive = right, negative = left)

        print("[MAIN] Starting EPS_LATE_CON control send loop...")
        print("[INFO] Sending EPS_LATE_CON message (0x37A) at 100Hz")
        print("[INFO] Press Ctrl+C to stop")
        print()

        # Test: Send a simple CAN message to verify bus is working
        print("[DEBUG] Testing CAN bus with simple message...")
        try:
            test_raw_msg = bytearray([0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08])
            p.can_send(0x100, test_raw_msg, VINFAST_CHASSIS_BUS)
            print("[DEBUG] Test CAN message sent successfully")
        except Exception as e:
            print(f"[ERROR] Failed to send test CAN message: {e}")
            import traceback
            traceback.print_exc()

        print()

        while True:
            loop_start = time.time()

            # Send EPS_LATE_CON message (0x37A) - steering control at 100Hz
            try:
                msg_address, msg_data, msg_bus = create_steering_control(
                    packer, CP, frame, apply_angle, lat_active
                )

                # Debug: Print message details every 100 frames (once per second)
                if frame % 100 == 0:
                    print(f"[DEBUG] Frame {frame}: Sending EPS_LATE_CON")
                    print(f"[DEBUG]   Address: 0x{msg_address:x} ({msg_address})")
                    print(f"[DEBUG]   Data: {msg_data.hex()}")
                    print(f"[DEBUG]   Bus from packer: {msg_bus}, Using bus: {VINFAST_CHASSIS_BUS}")
                    print(f"[DEBUG]   Data length: {len(msg_data)} bytes")

                # Send message
                p.can_send(msg_address, msg_data, VINFAST_CHASSIS_BUS)

            except Exception as e:
                print(f"[ERROR] Failed to create/send EPS_LATE_CON message: {e}")
                import traceback
                traceback.print_exc()
                # Continue anyway to keep the loop running

            # Read debug messages from Panda
            debug_msg = p.serial_read(0)
            if debug_msg:
                try:
                    debug_str = debug_msg.decode('utf-8', errors='ignore').strip()
                    if debug_str:
                        print(f"[DEBUG] {debug_str}")
                except Exception:
                    pass  # Ignore decode errors

            # Read CAN messages from Panda (optional, can be disabled to reduce overhead)
            can_msgs = p.can_recv()
            if can_msgs:
                for msg in can_msgs:
                    # Handle both 3-tuple and 4-tuple formats
                    if len(msg) == 4:
                        address, ts, data, bus = msg
                    elif len(msg) == 3:
                        address, bus, data = msg
                        ts = None
                    else:
                        continue

                    # Print received CAN messages
                    # Filter to only show EPS_LATE_CON or interesting messages
                    if address == ADAS_EPS_LATE_CON or frame % 500 == 0:  # Show EPS_LATE_CON or every 5 seconds
                        print(f"[CAN RX] ID: 0x{address:x} ({address}), Bus: {bus}, Data: {bytes(data).hex()}")

                        # Check if we're receiving our own EPS_LATE_CON message (loopback)
                        if address == ADAS_EPS_LATE_CON:
                            print(f"[DEBUG] Received EPS_LATE_CON message on bus {bus} (possible loopback)")

            frame += 1

            # Maintain frequency
            elapsed = time.time() - loop_start
            if elapsed < period:
                time.sleep(period - elapsed)
            else:
                # Warn if we're running slow
                if frame % 100 == 0:
                    print(f"[WARNING] Loop running slow: {elapsed*1000:.2f}ms (target: {period*1000:.2f}ms)")

    except KeyboardInterrupt:
        print("\n[MAIN] Test terminated by user")
    except Exception as e:
        print(f"[MAIN] Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if p is not None:
            print("[MAIN] Resetting Panda to SILENT safety mode...")
            try:
                p.set_safety_mode(CarParams.SafetyModel.silent)
            except Exception as e:
                print(f"[MAIN] Error resetting safety: {e}")


if __name__ == "__main__":
    vinfast_alloutput_test()
