from panda import Panda
from opendbc.car.structs import CarParams
from opendbc.can import CANPacker
from opendbc.car import Bus
from opendbc.car.vinfast.vinfastcan import create_steering_control
from opendbc.car.vinfast.values import CAR, CANBUS
import threading
import time

# VinFast IDs
ADAS_EPS_LATE_CON = 0x37A  # Lateral control (steering angle)
ADAS_LKA = 0x132  # LKA status message

VINFAST_CAMERA_BUS = CANBUS.cam


def heartbeat_thread_10hz(p):
    """Send Panda heartbeat at 10Hz (every 100 ms) so safety doesn't fall back to SILENT."""
    while True:
        try:
            p.send_heartbeat(engaged=True)
            time.sleep(0.1)
        except Exception as e:
            print(f"[HEARTBEAT] Error: {e}, stopping heartbeat thread")
            break


def vinfast_lka_test():
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

        # Initialize CANPacker with VinFast DBC
        print("[MAIN] Initializing CANPacker...")
        # Get DBC name from VinFast platform config
        dbc_name = CAR.VINFAST_VF8.config.dbc_dict[Bus.chassis]
        packer = CANPacker(dbc_name)

        # Create a minimal CarParams object for message creation
        # The CP object is mainly for compatibility, create_steering_control might not need it
        CP = None  # Will pass None, update if create_steering_control requires it

        # Start heartbeat first so watchdog is always happy
        print("[MAIN] Starting 10Hz heartbeat thread...")
        hb_thread = threading.Thread(target=heartbeat_thread_10hz, args=(p,), daemon=True)
        hb_thread.start()

        print("[MAIN] Setting VinFast safety mode...")
        p.set_safety_mode(CarParams.SafetyModel.vinfast)

        # One immediate heartbeat
        p.send_heartbeat(engaged=True)

        # Control loop configuration
        desired_frequency = 100  # Hz, like your original script
        period = 1.0 / desired_frequency

        frame = 0
        lat_active = True  # Lateral control active flag
        apply_angle = 0.0  # Steering angle in degrees (0 = center, positive = right, negative = left)

        print("[MAIN] Starting lateral control send loop...")
        while True:
            loop_start = time.time()

            # Create steering control message (0x37A) using VinFast car library
            # make_can_msg returns (address, data, bus) tuple
            msg_address, msg_data, msg_bus = create_steering_control(packer, CP, frame, apply_angle, lat_active)

            # Send lateral control message (0x37A) via Panda
            p.can_send(msg_address, msg_data, VINFAST_CAMERA_BUS)

            # Read debug (your "I'm from vinfast_tx_hook hehe" etc.)
            debug_msg = p.serial_read(0)
            if debug_msg:
                try:
                    print(f"Debug: {debug_msg.decode('utf-8', errors='ignore')}")
                except Exception:
                    print(f"Debug (hex): {debug_msg.hex()}")

            # Read CAN messages from Panda to see what’s going on on the bus
            can_msgs = p.can_recv()
            for msg in can_msgs:
                # Handle both 3-tuple and 4-tuple formats
                if len(msg) == 4:
                    address, ts, data, bus = msg
                elif len(msg) == 3:
                    address, bus, data = msg
                    ts = None
                else:
                    print(f"[CAN] Unknown message format: {msg}")
                    continue

                print(
                    f"[CAN] ID: 0x{address:x}, Bus: {bus}, "
                    f"Data: {bytes(data).hex()}, TS: {ts}"
                )

            frame += 1

            elapsed = time.time() - loop_start
            if elapsed < period:
                time.sleep(period - elapsed)

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
    vinfast_lka_test()
