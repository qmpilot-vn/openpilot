from panda import Panda
from opendbc.car.structs import CarParams
from opendbc.can import CANPacker
from opendbc.car import Bus
from opendbc.car.vinfast.vinfastcan import create_steering_control, create_lka_control
from opendbc.car.vinfast.values import CAR
import threading
import time

# VinFast message IDs
ADAS_EPS_LATE_CON = 0x37A  # 890 - Lateral control (steering angle)
ADAS_LKA = 0x132           # 306 - LKA status message

# CAN bus assignments
VINFAST_CHASSIS_BUS = 0
VINFAST_CAMERA_BUS = 2


def heartbeat_thread_10hz(p):
    """Send Panda heartbeat at 10Hz to keep safety mode active."""
    while True:
        try:
            p.send_heartbeat(engaged=True)
            time.sleep(1)
        except Exception as e:
            print(f"[HEARTBEAT] Error: {e}")
            break


def vinfast_safety_test():
    """
    Quick test script for VinFast safety mode.
    Sends EPS_LATE_CON (steering) at 100Hz and LKA at 20Hz.
    """
    p = None
    packer = None

    try:
        print("[MAIN] Connecting to Panda...")
        p = Panda()

        # Disable power save
        try:
            p.set_power_save(False)
        except:
            pass

        # Initialize CANPacker
        print("[MAIN] Initializing CANPacker...")
        CP = None
        dbc_name = CAR.VINFAST_VF8.config.dbc_dict[Bus.chassis]
        packer = CANPacker(dbc_name)
        print(f"[MAIN] DBC: {dbc_name}")

        # Start heartbeat
        print("[MAIN] Starting heartbeat thread...")
        hb_thread = threading.Thread(target=heartbeat_thread_10hz, args=(p,), daemon=True)
        hb_thread.start()

        # Set VinFast safety mode
        print("[MAIN] Setting VinFast safety mode...")
        p.set_safety_mode(CarParams.SafetyModel.vinfast)
        p.send_heartbeat(engaged=True)

        # Control parameters
        frame = 0
        lat_active = True
        apply_angle = 0.0  # Steering angle in degrees

        print("[MAIN] Starting control loop (100Hz)...")
        print("[INFO] Sending EPS_LATE_CON (0x37A) at 100Hz")
        print("[INFO] Sending LKA (0x132) at 20Hz")
        print("[INFO] Press Ctrl+C to stop\n")

        period = 1.0 / 100.0  # 100Hz

        while True:
            loop_start = time.time()

            # Send EPS_LATE_CON at 100Hz
            msg_addr, msg_data, msg_bus = create_steering_control(
                packer, CP, frame, apply_angle, lat_active
            )
            p.can_send(msg_addr, msg_data, VINFAST_CHASSIS_BUS)

            # Send LKA at 20Hz (every 5 frames)
            if frame % 5 == 0:
                lka_addr, lka_data, lka_bus = create_lka_control(
                    packer, CP, int(frame / 5), lat_active
                )
                p.can_send(lka_addr, lka_data, VINFAST_CAMERA_BUS)

            # Read debug messages
            debug = p.serial_read(0)
            if debug:
                try:
                    print(f"[DEBUG] {debug.decode('utf-8', errors='ignore').strip()}")
                except:
                    pass

            # Read CAN messages (optional - comment out if too verbose)
            can_msgs = p.can_recv()
            for msg in can_msgs:
                if len(msg) >= 3:
                    addr = msg[0] if len(msg) == 3 else msg[0]
                    bus = msg[1] if len(msg) == 3 else msg[3]
                    # Only print interesting messages
                    if addr in (ADAS_EPS_LATE_CON, ADAS_LKA) or frame % 500 == 0:
                        print(f"[CAN RX] ID: 0x{addr:x}, Bus: {bus}")

            frame += 1

            # Maintain 100Hz
            elapsed = time.time() - loop_start
            if elapsed < period:
                time.sleep(period - elapsed)

    except KeyboardInterrupt:
        print("\n[MAIN] Stopped by user")
    except Exception as e:
        print(f"[MAIN] Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if p:
            print("[MAIN] Resetting to SILENT mode...")
            try:
                p.set_safety_mode(CarParams.SafetyModel.silent)
            except:
                pass


if __name__ == "__main__":
    vinfast_safety_test()

