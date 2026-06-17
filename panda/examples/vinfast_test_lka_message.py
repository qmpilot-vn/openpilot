import os
import zmq
import json
import can
import cantools
import time
import threading
from vinfast_create_msgs import *

import numpy as np

_OPENDBC_DBC = os.environ.get(
  "VINFAST_DBC_DIR",
  os.path.normpath(os.path.join(os.path.dirname(__file__), "..", "..", "opendbc_repo", "opendbc", "dbc")),
)

uprate = 10
downrate = -10

### Frequency of Control: 100Hz
desired_frequency = 100  # Hz
period = 1.0 / desired_frequency

### Initialize CAN buses
can0_bus = can.Bus(channel='can0', interface='socketcan')
# can1_bus = can.Bus(channel='can1', interface='socketcan')

### Initialize ZeroMQ context and socket
context = zmq.Context()
socket = context.socket(zmq.SUB)
socket.connect("tcp://localhost:5555")  # Change the address to match the publisher
socket.subscribe(b'')

### Initialize DBC
db = cantools.database.load_file(os.path.join(_OPENDBC_DBC, "vinfast_vf8_chassis_can.dbc"))

ADAS_EPS_LATE_CON = 0x37A
ADAS_LKA = 0x132
ADAS_ACC_STATUS = 0x32D
ADAS_IDB = 0x131

def map_range(x, in_min, in_max, out_min, out_max):
    """Maps a value from one range to another."""
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


def forward_can_messages(source_bus, destination_bus, filter_out_ids=None):
    """Forwards CAN messages with optional filtering."""
    try:
        for message in source_bus:
            if filter_out_ids and message.arbitration_id in filter_out_ids:
                continue  # Skip filtered IDs

            destination_message = can.Message(
                arbitration_id=message.arbitration_id,
                data=message.data,
                is_extended_id=message.is_extended_id
            )
            try:
                destination_bus.send(destination_message)
            except can.CanError as e:
                print(f"[FORWARD ERROR] {e}")
    except Exception as e:
        print(f"[FORWARDING THREAD ERROR] {e}")


def start_can_forwarding(can0_bus, can1_bus):
    """Starts threads for bidirectional CAN forwarding."""
    thread_0_to_1 = threading.Thread(
        target=forward_can_messages, args=(can0_bus, can1_bus, None), daemon=True
    )

    # filter_out_ids_1_to_0 = {}
    filter_out_ids_1_to_0 = {ADAS_EPS_LATE_CON, ADAS_LKA, ADAS_ACC_STATUS} 
    
    thread_1_to_0 = threading.Thread(
        target=forward_can_messages, args=(can1_bus, can0_bus, filter_out_ids_1_to_0), daemon=True
    )

    thread_0_to_1.start()
    thread_1_to_0.start()

    return thread_0_to_1, thread_1_to_0


def can_send_thread():
    """Sends periodic CAN messages at the desired frequency."""
    
    frame = 0
    hapWarning = 0
    re_active = 0 

    ready = 0
    previous_button_x_state = False  # False means released, True means pressed

    print("[SEND THREAD] Starting send thread...")
    try:
        while True:
            print(frame)
            start_time = time.time()

            # try:
            #     data = socket.recv_json()
            #     steerAngle = map_range(data["axes"][3], -1, 1, 700, -700)
            #     accel = map_range(data["axes"][1], -1, 1, 3, -3)
            #     current_button_x_state = data["buttons"][0]
            #     print(current_button_x_state)
            # except Exception as e:
            #     print(f"[RECEIVE ERROR] {e}")
            #     continue
            
            accel = -2
            steerAngle = 300 # Simulate a constant steer angle for testing
            
            current_button_x_state = 1 
            
            if frame < 200:
                # ready = 0
                mode = 2
            else:
                # ready = 1
                mode = 4
                # print(ready, mode)
                
            if accel < 0:
                deaccel = 1
            else:
                deaccel = 0 
                        
            # Detect change in button state
            if current_button_x_state and not previous_button_x_state:  # Button X just pressed
                ready = not ready  # Toggle the ready state
                print(f"Ready state changed to: {ready}")
            
            # Update the previous state
            previous_button_x_state = current_button_x_state
            
            request = 0 
            
            data_adas_idb = create_adas_idb_message(int(frame /2), request)
            
            data_lka = create_lka_message(int(frame / 2), hapWarning)
            
            data_adas_eps_late_con = create_adas_eps_late_con_message(frame, steerAngle, ready)
            
            data_adas_acc_status = create_adas_acc_status_message(int(frame / 2), accel, mode, deaccel)
            
            # print(steerAngle)
            
            message_adas_eps_late_con = can.Message(
                arbitration_id=ADAS_EPS_LATE_CON, data=data_adas_eps_late_con, is_extended_id=False
            )
            message_adas_lka = can.Message(
                arbitration_id=ADAS_LKA, data=data_lka, is_extended_id=False
            )

            message_adas_acc_status = can.Message(
                arbitration_id=ADAS_ACC_STATUS, data=data_adas_acc_status, is_extended_id=False
            )
            
            message_adas_idb = can.Message(
                arbitration_id=ADAS_IDB, data=data_adas_idb, is_extended_id=False
            )


            can0_bus.send(message_adas_eps_late_con)
            
            if frame % 2 == 0:
                can0_bus.send(message_adas_lka)
                can0_bus.send(message_adas_acc_status)
                can0_bus.send(message_adas_idb)
                
            frame += 1
            
            # print(frame)
            elapsed_time = time.time() - start_time
            if elapsed_time < period:
                time.sleep(period - elapsed_time)
    except KeyboardInterrupt:
        print("[SEND THREAD] Shutting down...")
        can0_bus.shutdown()
        can1_bus.shutdown()
    except Exception as e:
        print(f"[SEND THREAD ERROR] {e}")


# Start CAN forwarding and sending threads
if __name__ == "__main__":
    try:
        print("[MAIN THREAD] Starting CAN forwarding and sending...")
        # forward_threads = start_can_forwarding(can0_bus, can1_bus)

        send_thread = threading.Thread(target=can_send_thread, daemon=True)
        send_thread.start()

        # for thread in forward_threads:
        #     thread.join()
        send_thread.join()
        
    except KeyboardInterrupt:
        print("[MAIN THREAD] Stopping CAN forwarding and sending...")
    except Exception as e:
        print(f"[MAIN THREAD ERROR] {e}")
