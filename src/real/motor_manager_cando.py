# ============================================================================
# This module is deprecated and no longer maintained.
# It supports legacy cando hardware which is now replaced by python-can drivers.
# Keep only for historical reference and troubleshooting.
# ============================================================================

import os
import time
import numpy as np
import struct
import threading
import cando
import logging
import usb.util
from real.can_config import *
from real.motor_message import MotorMessage
from real.motor import Motor

ID_STD_OFFSET = 6
ID_EXT_OFFSET = 24

class MotorManager:
    def __init__(self):
        self.logger = logging.getLogger(__name__)
        self.logger.warning("MotorManager is deprecated and no longer maintained. "
                            "It supports legacy cando hardware which is now replaced by python-can drivers. "
                            "Keep only for historical reference and troubleshooting.")
        
        self.device = None
        self.received_frame = cando.Frame()

        self.device = self.connect_can_device()
        self.motor_dict = self.connect_motors()

        self.thread_pause_event = threading.Event()
        self.thread_stop_event = threading.Event()
        self.reading_thread = threading.Thread(target = self.can_read_handle)

        self.thread_stop_event.set()
        self.reading_thread.start()
        self.thread_pause_event.set() # Set auto receive

# -------------------------- CAN and Thread methods -------------------------- #
    def connect_can_device(self):
        device_list = cando.list_scan()

        if len(device_list) == 0:
            raise ConnectionError("No CAN device found")
        
        device = device_list[0]

        # Detach kernel driver if on Linux
        if os.name == 'posix' and device.is_kernel_driver_active(0):
            device.detach_kernel_driver(0)
            self.logger.debug("Detached kernel driver from CAN device")

        usb.util.claim_interface(device, 0)
        cando.dev_set_timing(device, 1, 12, 6, 1, 6) # set baudrate: 500K, sample point: 87.5%
        cando.dev_start(device, cando.CANDO_MODE_NORMAL | cando.CANDO_MODE_NO_ECHO_BACK)

        self.logger.info("CAN device connected")
        return device

    def connect_motors(self) -> dict[int, Motor]:
        # Create 12 motor objects
        # Motor ID starts from 1, ends at 12
        motor_dict = {}
        for motor_id in range(1, 13):
            motor_dict[motor_id] = Motor(motor_id, False)
                
        self.logger.info("Connecting motors...")
        for motor_id in range(1, 13):
            # Prepare frame
            send_frame = cando.Frame()
            send_frame.can_id = 0x00 | (motor_id << 6)
            send_frame.can_id |= cando.CANDO_ID_RTR
            # send_frame.can_dlc = 8
            # send_frame.data = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08]
            cando.dev_frame_send(self.device, send_frame)
            
            time.sleep(0.001)

            # Check if the motor responds
            if not cando.dev_frame_read(self.device, self.received_frame, 1):
                motor_dict[motor_id].exist = False
                self.logger.warning(f"Motor {motor_id} does not respond")
                continue

            # Check if the message has error
            if self.msg_error(self.received_frame):
                motor_dict[motor_id].exist = False
                self.logger.warning(f"Motor {motor_id} has message error")
                continue
            
            # Decode the message
            # and check if the motor ID matches
            can_pack = self.decode_msg(self.received_frame)
            if motor_id == can_pack.motor_id:
                motor_dict[motor_id].exist = True
                self.logger.info(f"Motor {motor_id} connected successfully")
            else:
                self.disconnect_can_device()
                raise ConnectionError(f"Expected motor {motor_id}, but got {can_pack.motor_id}")

        # Read frames to clear the buffer
        for x in range(10):
            cando.dev_frame_read(self.device, self.received_frame, 1)

        self.logger.info("Finished connecting motors")
        return motor_dict
    
    def disconnect_can_device(self) -> None:
        # Read frames to clear the buffer
        for _ in range(10):
            cando.dev_frame_read(self.device, self.received_frame, 1)

        cando.dev_stop(self.device)
        usb.util.release_interface(self.device, 0)

        # Reattach kernel driver if on Linux
        if os.name == 'posix' and hasattr(self.device, 'attach_kernel_driver'):
            self.device.attach_kernel_driver(0)
            self.logger.debug("Reattached kernel driver to CAN device")

        self.device = None
        self.logger.info("Disconnected CAN device")
    
    def shutdown(self) -> None:
        self.thread_stop_event.clear()
        self.thread_pause_event.clear()

        if self.reading_thread.is_alive():
            self.reading_thread.join()

        self.disconnect_can_device()

    def can_read_handle(self) -> None:
        while self.thread_stop_event.isSet():
            self.thread_pause_event.wait()
            
            # Check if receive any frame
            if not cando.dev_frame_read(self.device, self.received_frame, 1):
                continue

            # Check if the message has error
            if self.msg_error(self.received_frame):
                self.shutdown()
                raise RuntimeError("CAN frame read error")
            
            can_pack = self.decode_msg(self.received_frame)
            self.motor_dict[can_pack.motor_id].update_param(can_pack.id_type, can_pack.msg_id, can_pack.data)

    def send_motor_cmd(self, motor_id: int, cmd: IntEnum, value: int) -> None:
        if self.motor_dict[motor_id].exist == False:
            self.logger.warning(f"Motor {motor_id} does not exist, cannot send command")
            return

        send_frame = cando.Frame()
        
        if (cmd.__class__) == CAN_STD_TYPE:
            send_frame.can_id = 0x00 | (motor_id << ID_STD_OFFSET) | cmd.value
        else:
            send_frame.can_id = 0x00 | (motor_id << ID_EXT_OFFSET) | cmd.value
            send_frame.can_id |= cando.CANDO_ID_EXTENDED

        send_frame.can_dlc = 2
        value = int(value)
        send_frame.data = [value >> 8, value & 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]

        cando.dev_frame_send(self.device, send_frame)
    
    def msg_error(self, received_frame) -> bool:
        if received_frame.can_id & cando.CANDO_ID_ERR:
            error_code, err_tx, err_rx = cando.parse_err_frame(received_frame)

            self.logger.error("CAN Error detected: code=%s", error_code)

            if error_code & cando.CAN_ERR_BUSOFF:
                self.logger.critical(" CAN_ERR_BUSOFF")
            if error_code & cando.CAN_ERR_RX_TX_WARNING:
                self.logger.error(" CAN_ERR_RX_TX_WARNING")
            if error_code & cando.CAN_ERR_RX_TX_PASSIVE:
                self.logger.error(" CAN_ERR_RX_TX_PASSIVE")
            if error_code & cando.CAN_ERR_OVERLOAD:
                self.logger.error(" CAN_ERR_OVERLOAD")
            if error_code & cando.CAN_ERR_STUFF:
                self.logger.error(" CAN_ERR_STUFF")
            if error_code & cando.CAN_ERR_FORM:
                self.logger.error(" CAN_ERR_FORM")
            if error_code & cando.CAN_ERR_ACK:
                self.logger.error(" CAN_ERR_ACK")
            if error_code & cando.CAN_ERR_BIT_RECESSIVE:
                self.logger.error(" CAN_ERR_BIT_RECESSIVE")
            if error_code & cando.CAN_ERR_BIT_DOMINANT:
                self.logger.error(" CAN_ERR_BIT_DOMINANT")
            if error_code & cando.CAN_ERR_CRC:
                self.logger.error(" CAN_ERR_CRC")
                self.logger.error(" err_tx=%s", err_tx)
                self.logger.error(" err_rx=%s", err_rx)

            return True
        else:
            return False

    def decode_msg(self, received_frame) -> MotorMessage:
        can_id = received_frame.can_id & cando.CANDO_ID_MASK
        can_dlc = received_frame.can_dlc
        group_mag = False

        if (received_frame.can_id & cando.CANDO_ID_EXTENDED):
            # Extended ID
            motor_id = can_id >> 24
            id_type = CAN_ID_TYPE.EXTENDED
            msg_id = can_id & 0xFFFFF
            value = struct.unpack("<h",bytes(received_frame.data[0:2]))[0]
        else:
            # Standard ID
            motor_id = can_id >> 6
            if (motor_id >> 10 > 0):
                group_mag = True
            id_type = CAN_ID_TYPE.STANDARD
            msg_id = can_id & 0x1F
            value = struct.unpack("<h",bytes(received_frame.data[0:2]))[0]

        if (group_mag):
            pass

        return MotorMessage(motor_id, id_type, msg_id, value)

# -------------------------- Motor api for leg group ------------------------- #
    def enable_motor_torque(self, motor_id: int, enable: bool) -> None:
        self.send_motor_cmd(motor_id, CAN_STD_TYPE.CAN_STDID_TORQUE_ENABLE, enable)

    def set_control_mode(self, motor_id: int, mode: int) -> None:
        self.send_motor_cmd(motor_id, CAN_STD_TYPE.CAN_STDID_CONTROL_MODE, mode)

    def set_motor_angle(self, motor_id: int, angle) -> None:
        can_signal = int(round(angle * 32768 / np.pi))
        self.send_motor_cmd(motor_id, CAN_STD_TYPE.CAN_STDID_GOAL_POSITION_DEG, can_signal)
    
    def set_motor_omega(self, motor_id: int, omega) -> None:
        can_signal = int(round(omega * 32768 / np.pi))
        self.send_motor_cmd(motor_id, CAN_STD_TYPE.CAN_STDID_GOAL_VELOCITY_DPS, can_signal)

    def get_zero_state(self, motor_id: int) -> int:
        self.send_motor_cmd(motor_id, CAN_STD_TYPE.CAN_STDID_ZERO_STATE, 0)
        zero_state = self.motor_dict[motor_id].get_param(CMD_TYPE.ZERO_STATE)
        zero_state = int(zero_state)

        return zero_state
    
    def get_motor_angle(self, motor_id) -> float:
        self.send_motor_cmd(motor_id, CAN_STD_TYPE.CAN_STDID_PRESENT_REVOLUTION, 0)
        self.send_motor_cmd(motor_id, CAN_STD_TYPE.CAN_STDID_PRESENT_POSITION_DEG, 0)
        
        revolution = self.motor_dict[motor_id].get_param(CMD_TYPE.PRESENT_REVOLUTION)
        angle_16 = self.motor_dict[motor_id].get_param(CMD_TYPE.PRESENT_POSITION_DEG)
        # print("rev: " + str(revolution) + "  angle: " + str(angle_16))

        angle = (revolution + (angle_16 / 65536)) / 71.96 * 2 * np.pi
        # print("degree: " + str(math.degrees(angle)))
        
        return angle

    def get_motor_omega(self, motor_id) -> float:
        self.send_motor_cmd(motor_id, CAN_STD_TYPE.CAN_STDID_PRESENT_VELOCITY_DPS, 0)
        velocity_01 = self.motor_dict[motor_id].get_param(CMD_TYPE.PRESENT_VELOCITY_DPS)

        omega = (((velocity_01 * 10) / 65536)) / 71.96 * 2 * np.pi
        # print("omega: " + str(omega))
        return omega
    