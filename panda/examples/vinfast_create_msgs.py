import os

import cantools
import struct
import cantools.database
import crcmod

# openpilot opendbc DBCs (VF8) — override with VINFAST_DBC_DIR if needed
_OPENDBC_DBC = os.environ.get(
  "VINFAST_DBC_DIR",
  os.path.normpath(os.path.join(os.path.dirname(__file__), "..", "..", "opendbc_repo", "opendbc", "dbc")),
)

db = cantools.database.load_file(os.path.join(_OPENDBC_DBC, "vinfast_vf8_chassis_can.dbc"))
db_info = cantools.database.load_file(os.path.join(_OPENDBC_DBC, "vinfast_vf8_info_can.dbc"))
db_body = cantools.database.load_file(os.path.join(_OPENDBC_DBC, "vinfast_vf8_body_can.dbc"))

vinfast_checksum = crcmod.mkCrcFun(0x11D, initCrc=0xFF, rev=False, xorOut=0xFF)

# Define the CRC-8 function for ADAS_LKA (Polynomial: 0x11D, Initial value: 0xFF, XOR out: 0xFF)

table = [
        0x00 , 0x1D , 0x3A , 0x27 , 0x74 , 0x69 , 0x4E , 0x53 ,
        0xE8 , 0xF5 , 0xD2 , 0xCF , 0x9C , 0x81 , 0xA6 , 0xBB ,
        0xCD , 0xD0 , 0xF7 , 0xEA , 0xB9 , 0xA4 , 0x83 , 0x9E , 0x25
        , 0x38 , 0x1F , 0x02 , 0x51 , 0x4C , 0x6B , 0x76 ,
        0x87 , 0x9A , 0xBD , 0xA0 , 0xF3 , 0xEE , 0xC9 , 0xD4 ,
        0x6F , 0x72 , 0x55 , 0x48 , 0x1B , 0x06 , 0x21 , 0x3C ,
        0x4A , 0x57 , 0x70 , 0x6D , 0x3E , 0x23 , 0x04 , 0x19 ,
        0xA2 , 0xBF , 0x98 , 0x85 , 0xD6 , 0xCB , 0xEC , 0xF1 ,
        0x13 , 0x0E , 0x29 , 0x34 , 0x67 , 0x7A , 0x5D , 0x40 ,
        0xFB , 0xE6 , 0xC1 , 0xDC , 0x8F , 0x92 , 0xB5 , 0xA8 ,
        0xDE , 0xC3 , 0xE4 , 0xF9 , 0xAA , 0xB7 , 0x90 , 0x8D ,
        0x36 , 0x2B , 0x0C , 0x11 , 0x42 , 0x5F , 0x78 , 0x65 ,
        0x94 , 0x89 , 0xAE , 0xB3 , 0xE0 , 0xFD , 0xDA , 0xC7 ,
        0x7C , 0x61 , 0x46 , 0x5B , 0x08 , 0x15 , 0x32 , 0x2F ,
        0x59 , 0x44 , 0x63 , 0x7E , 0x2D , 0x30 , 0x17 , 0x0A ,
        0xB1 , 0xAC , 0x8B , 0x96 , 0xC5 , 0xD8 , 0xFF , 0xE2 ,
        0x26 , 0x3B , 0x1C , 0x01 , 0x52 , 0x4F , 0x68 , 0x75 ,
        0xCE , 0xD3 , 0xF4 , 0xE9 , 0xBA , 0xA7 , 0x80 , 0x9D ,
        0xEB , 0xF6 , 0xD1 , 0xCC , 0x9F , 0x82 , 0xA5 , 0xB8 ,
        0x03 , 0x1E , 0x39 , 0x24 , 0x77 , 0x6A , 0x4D , 0x50 ,
        0xA1 , 0xBC , 0x9B , 0x86 , 0xD5 , 0xC8 , 0xEF , 0xF2 ,
        0x49 , 0x54 , 0x73 , 0x6E , 0x3D , 0x20 , 0x07 , 0x1A ,
        0x6C , 0x71 , 0x56 , 0x4B , 0x18 , 0x05 , 0x22 , 0x3F ,
        0x84 , 0x99 , 0xBE , 0xA3 , 0xF0 , 0xED , 0xCA , 0xD7 , 0x35
        , 0x28 , 0x0F , 0x12 , 0x41 , 0x5C , 0x7B , 0x66 ,
        0xDD , 0xC0 , 0xE7 , 0xFA , 0xA9 , 0xB4 , 0x93 , 0x8E ,
        0xF8 , 0xE5 , 0xC2 , 0xDF , 0x8C , 0x91 , 0xB6 , 0xAB , 0x10
        , 0x0D , 0x2A , 0x37 , 0x64 , 0x79 , 0x5E , 0x43 ,
        0xB2 , 0xAF , 0x88 , 0x95 , 0xC6 , 0xDB , 0xFC , 0xE1 ,
        0x5A , 0x47 , 0x60 , 0x7D , 0x2E , 0x33 , 0x14 , 0x09 ,
        0x7F , 0x62 , 0x45 , 0x58 , 0x0B , 0x16 , 0x31 , 0x2C ,
        0x97 , 0x8A , 0xAD , 0xB0 , 0xE3 , 0xFE , 0xD9 , 0xC4
    ]

def mkCrcFun(data):
    """
    Tính toán CRC-8 với bảng tra cứu cho một dữ liệu đầu vào.
    :param data: Dữ liệu đầu vào dưới dạng danh sách byte hoặc mảng byte
    :return: CRC-8 tính toán
    """
    crc = 0xFF
    for byte in data:
        crc = table[crc ^ byte]
    crc ^= 0xFF  # XOR cuối cùng với 0xFF
    return crc

def create_lka_message(frame,latActive): 
    
    # Calculate the alive counter, cycling from 0 to 14
    value_ADAS_LKA_Alive = frame % 15  # Use modulo 15 to limit values to 0-14
    
    # print(f"Alive Counter: {value_ADAS_LKA_Alive}")  # Debug output for alive counter
    
    # Create the initial message without the checksum
    message_content = {
        "ADAS_LKA_Checksum": 0,  # Placeholder checksum
        "ADAS_LKA_Alive": value_ADAS_LKA_Alive,
        "LSS_Activation": 1,
        "SECCAN_ADAS_LKA": 0,
        "ADAS_HapWarning": latActive
    }
    
    # Encode the message content
    msg_lka = db.encode_message('ADAS_LKA', message_content)
    
    # Calculate the checksum, excluding the first byte (checksum itself)
    checksum = mkCrcFun(msg_lka[1:])
    
    # Update the message content with the calculated checksum
    message_content["ADAS_LKA_Checksum"] = checksum
    
    # Encode the final message
    msg_lka = db.encode_message('ADAS_LKA', message_content)
    
    # print(f"Checksum: {checksum}, alive: {value_ADAS_LKA_Alive} ")  # Debug output for checksum
    return msg_lka

def create_adas_idb_message(frame, request):

    # Calculate the alive counter, cycling from 0 to 14
    value_ADAS_IDB_Alive = frame % 15  # Use modulo 15 to limit values to 0-14
    
    # print(f"Alive Counter: {value_ADAS_LKA_Alive}")  # Debug output for alive counter
    
    # Create the initial message without the checksum
    message_content = {
        "ADAS_IDB_Checksum": 0,  # Placeholder checksum
        "ADAS_IDB_Alive": value_ADAS_IDB_Alive,
        "SECCAN_ADAS_IDB": 0,
        "ADAS_ACC_IDB_DriveOff": request
    }
    
    # Encode the message content
    msg_adas_idb = db.encode_message('ADAS_IDB', message_content)
    
    # Calculate the checksum, excluding the first byte (checksum itself)
    checksum = mkCrcFun(msg_adas_idb[1:])
    
    # Update the message content with the calculated checksum
    message_content["ADAS_IDB_Checksum"] = checksum
    
    # Encode the final message
    msg_esp_late = db.encode_message('ADAS_IDB', message_content)
    
    # print(f"Checksum: {checksum}, alive: {value_ALV_ADAS_EPS_LATE_CON} ")  # Debug output for checksum
    return msg_adas_idb


def create_adas_eps_late_con_message(frame, angle, ready):

    # Calculate the alive counter, cycling from 0 to 14
    value_ALV_ADAS_EPS_LATE_CON = frame % 15  # Use modulo 15 to limit values to 0-14
    
    # print(f"Alive Counter: {value_ADAS_LKA_Alive}")  # Debug output for alive counter
    
    # Create the initial message without the checksum
    message_content = {
        "CHKSM_ADAS_EPS_LATE_CON": 0,  # Placeholder checksum
        "ALV_ADAS_EPS_LATE_CON": value_ALV_ADAS_EPS_LATE_CON,
        "ADAS_EPS_StrWhe_TOLAct": 0,
        "ADAS_EPS_StrWhe_AOLAct": ready,
        "ADAS_EPS_AOLReq": angle,
        "ADAS_EPS_Torq_Fact_Req": 1,
    }
    
    # Encode the message content
    msg_esp_late = db.encode_message('ADAS_EPS_LATE_CON', message_content)
    
    # Calculate the checksum, excluding the first byte (checksum itself)
    checksum = mkCrcFun(msg_esp_late[1:])
    
    # Update the message content with the calculated checksum
    message_content["CHKSM_ADAS_EPS_LATE_CON"] = checksum
    
    # Encode the final message
    msg_esp_late = db.encode_message('ADAS_EPS_LATE_CON', message_content)
    
    # print(f"Checksum: {checksum}, alive: {value_ALV_ADAS_EPS_LATE_CON} ")  # Debug output for checksum
    return msg_esp_late

def create_adas_acc_status_message(frame, accel, mode, deaccel): 
    
    # Calculate the alive counter, cycling from 0 to 14
    value_Alive_ACC_STATUS = frame % 15  # Use modulo 15 to limit values to 0-14
    
    # print(f"Alive Counter: {value_ADAS_LKA_Alive}")  # Debug output for alive counter
    
    # Create the initial message without the checksum
    message_content = {
        "CRC_ACC_STATUS": 0,  # Placeholder checksum
        "Alive_ACC_STATUS": value_Alive_ACC_STATUS,
        "ADAS_ACC_Information": 0,
        "ADAS_ACC_Main_Mode": 1,
        "ADAS_ACC_AccelDecel_Cmd": accel,
        "ADAS_ACC_StandstillReq": 0,
        "ADAS_ACC_Mode": mode,
        "ADAS_ACC_IDB_DecCmdAct": deaccel 
    }
    
    # Encode the message content
    msg_adas_acc_status = db.encode_message('ADAS_ACC_Status', message_content)
    
    # Calculate the checksum, excluding the first byte (checksum itself)
    checksum = mkCrcFun(msg_adas_acc_status[1:])
    
    # Update the message content with the calculated checksum
    message_content["CRC_ACC_STATUS"] = checksum
    
    # Encode the final message
    msg_adas_acc_status = db.encode_message('ADAS_ACC_Status', message_content)
    
    # print(f"Checksum: {checksum}, alive: {value_ADAS_LKA_Alive} ")  # Debug output for checksum
    return msg_adas_acc_status


def create_mfs_control_button_message(frame, cruise_on_off=1, up_control=1):
    """
    Tạo message MFS_Control_Button với giá trị CruiseOn_Off chỉ định.
    
    :param frame: Số frame hiện tại (dùng để tính alive counter)
    :param cruise_on_off: Trạng thái của nút Cruise (1=On, 2=Off, v.v.)
    :return: bytes message đã encode
    """
    # Alive counter giới hạn từ 0 đến 14
    alive_counter = frame % 15

    # Lấy message từ DBC info
    msg_def = db_info.get_message_by_name("MFS_Control_Button")

    # Tạo dictionary với các giá trị cần thiết (chỉ thiết lập Cruise và Alive)
    message_content = {
        "CHSKM_MFS_Control_Button": 0, # placeholder checksum 
        "ALV_MFS_Control_Button": alive_counter,
        "MFS_CruiseOn_Off": cruise_on_off, 
        "MFS_UpControl": up_control
    }

    # Set các tín hiệu còn lại về 0 để tránh lỗi thiếu field
    for signal in msg_def.signals:
        if signal.name not in message_content:
            message_content[signal.name] = 0

    # Encode ra bytes
    msg_mfs_control_button = db_info.encode_message('MFS_Control_Button', message_content)
    
    # Calculate the checksum, excluding the first byte (checksum itself)
    checksum = mkCrcFun(msg_mfs_control_button[1:])
    
    # Update the message content with the calculated checksum
    message_content["CHSKM_MFS_Control_Button"] = checksum
    
    # Encode the final message
    msg_mfs_control_button = db_info.encode_message('MFS_Control_Button', message_content)

    return msg_mfs_control_button

def create_adas_bcm_status_message(frame,
                                   adas_status=1,  # Default: "Standby"
                                   headlight_request=0,  # Default: "No request"
                                   indicator_request=0,  # Default: "No request"
                                   foldmirror_request=0,  # Default: "No request"
                                   seccan_val=1):  # Default: 1 theo ảnh
    """
    Tạo message ADAS_BCM_Status (ID: 0x20F) với các tín hiệu cụ thể.

    :param frame: số thứ tự frame (dùng nếu cần AliveCounter trong tương lai)
    :param adas_status: 0~7 (Off, Standby, Enable, Active, Finished, Suspend, Abort, Failed)
    :param headlight_request: 0~1 (No request, Request turn on)
    :param indicator_request: 0~4 (No request, Left, Right, Hazard, Off)
    :param foldmirror_request: 0~3 (No request, Fold in, Fold out, Reserved)
    :param seccan_val: Giá trị SECCAN_ADAS_BCM (thường là 0 hoặc 1)
    :return: bytes message đã encode
    """
    msg_def = db_body.get_message_by_name("ADAS_BCM_Status")

    message_content = {
        "ADAS_BCM_Status": adas_status,
        "ADAS_BCM_HeadlightReq": headlight_request,
        "ADAS_BCM_IndicatorlightReq": indicator_request,
        "ADAS_BCM_FoldMirrorReq": foldmirror_request,
        "SECCAN_ADAS_BCM_Sts": seccan_val
    }

    # Đảm bảo các tín hiệu khác được set 0 để tránh lỗi encode thiếu field
    for signal in msg_def.signals:
        if signal.name not in message_content:
            message_content[signal.name] = 0

    # Encode message thành bytes
    msg_encoded = db_body.encode_message("ADAS_BCM_Status", message_content)
    return msg_encoded

def create_bcm_central_lock_message(frame,
                                    stat_cdl_led=0,
                                    door_unlock_fl=0,
                                    atws=0,
                                    door_lock_fl=0,
                                    central_lock_sw_sts=0):
    """
    Tạo message BCM_STAT_CENTRAL_LOCK từ DBC body.
    
    :param frame: số frame hiện tại, dùng để tính alive counter
    :param stat_cdl_led: Trạng thái đèn CDL (0~3)
    :param door_unlock_fl: Trạng thái mở cửa trước trái (0~3)
    :param atws: Anti-theft warning system trạng thái (0~7)
    :param door_lock_fl: Trạng thái khóa cửa trước trái (0~3)
    :param central_lock_sw_sts: Trạng thái công tắc khóa trung tâm (0~1)
    :return: bytes message đã encode
    """
    msg_def = db_body.get_message_by_name("BCM_STAT_CENTRAL_LOCK")
    alive_counter = frame % 15

    message_content = {
        "CHKSM_BCM_CENTRAL_LOCK": 0,  # Placeholder
        "ALV_BCM_CENTRAL_LOCK": alive_counter,
        "STAT_CDL_LED": stat_cdl_led,
        "STAT_DoorUnlockFL": door_unlock_fl,
        "STAT_ATWS": atws,
        "STAT_DoorLockFL": door_lock_fl,
        "BCM_CentralLockSwSts": central_lock_sw_sts
    }

    # Set tất cả tín hiệu còn lại về 0 nếu chưa có
    for signal in msg_def.signals:
        if signal.name not in message_content:
            message_content[signal.name] = 0

    # Encode tạm để tính checksum
    msg_encoded = db_body.encode_message("BCM_STAT_CENTRAL_LOCK", message_content)

    # Tính checksum bỏ qua byte đầu tiên
    checksum = mkCrcFun(msg_encoded[1:])

    # Gán lại checksum
    message_content["CHKSM_BCM_CENTRAL_LOCK"] = checksum

    # Encode lại
    msg_encoded = db_body.encode_message("BCM_STAT_CENTRAL_LOCK", message_content)
    
    return msg_encoded


def create_doorfl_lock_command_message(counter=0):
    """
    Tạo bản tin để khóa cửa trước bên trái.

    :param counter: giá trị bộ đếm (0~255)
    :return: bytes CAN message đã encode
    """
    message_content = {
        "NDoorFLVehicleLocked": 1,         #Xe đang khóa
        "NDoorFLVehicleMoving": 1,         #Xe đứng yên
        "NDoorFLOpenInvRequested": 0,
        "NDoorFLOpenRequested": 1,         # Không mở cửa
        "NDoorFLChildLocked": 0,
        "NDoorFLVehicleCrash": 0,
        "NDoorFLCommandCounter": counter,
        "NDoorFLCloseInvRequested": 0,
        "NDoorFLCloseRequested": 1         #Đóng cửa
    }

    msg = db_body.encode_message("DoorFLCommand", message_content)
    return msg

