from panda import Panda
import time
from opendbc.car.structs import CarParams
import threading


def heartbeat_thread_10hz(panda_device):
    """Send Panda heartbeat at 10Hz (every 100 ms)"""
    while True:
        try:
            panda_device.send_heartbeat(engaged=True)
            # 10Hz -> 0.1s
            time.sleep(0.1)
        except Exception as e:
            print(f"[HEARTBEAT] Error: {e}, stopping heartbeat thread")
            break


def test_vinfast_mode():
    p = None
    try:
        print("Connecting to Panda...")
        p = Panda()

        # Start dedicated heartbeat thread (10Hz)
        print("Starting 10Hz heartbeat thread...")
        hb_thread = threading.Thread(target=heartbeat_thread_10hz, args=(p,), daemon=True)
        hb_thread.start()

        print("Setting VinFast safety mode...")
        p.set_safety_mode(CarParams.SafetyModel.vinfast)

        # Send one heartbeat immediately
        p.send_heartbeat(engaged=True)

        # Main loop for CAN messages and debug output
        while True:
            # Send test CAN message every 2 seconds
            test_msg = bytearray([0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7, 0x8])
            p.can_send(0x1, test_msg, 0)

            # Debug read from Panda (safety prints, etc.)
            debug_msg = p.serial_read(0)
            if debug_msg:
                try:
                    print(f"Debug: {debug_msg.decode('utf-8', errors='ignore')}")
                except Exception:
                    print(f"Debug (hex): {debug_msg.hex()}")

            # Read CAN messages
            can_msgs = p.can_recv()
            for msg in can_msgs:
                # Handle both 3-tuple and 4-tuple formats
                if len(msg) == 4:
                    address, ts, data, bus = msg
                elif len(msg) == 3:
                    address, bus, data = msg
                    ts = None
                else:
                    print(f"Unknown CAN message format: {msg}")
                    continue

                print(
                    f"Received CAN - ID: 0x{address:x}, Bus: {bus}, Data: {bytes(data).hex()}"
                )

            time.sleep(2.0)

    except KeyboardInterrupt:
        print("\nTest terminated by user")
    except Exception as e:
        print(f"Error in main loop: {e}")
    finally:
        if p is not None:
            print("Resetting to SILENT mode...")
            try:
                p.set_safety_mode(CarParams.SafetyModel.silent)
            except Exception as e:
                print(f"Error resetting safety mode: {e}")


if __name__ == "__main__":
    test_vinfast_mode()
