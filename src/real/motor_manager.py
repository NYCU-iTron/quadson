import numpy as np
import struct
import threading
import can
import logging
import time
import real.can_config as can_config
from real.motor_message import MotorMessage
from real.motor import Motor

class MotorManager:
    def __init__(self):
        self.logger = logging.getLogger(__name__)

        self.bus = None
        self.received_message = can.Message()
        self.error_flag = False

        self.bus = self.connect_can_bus()
        self.motor_dict = self.connect_motors()

        self.thread_pause_event = threading.Event()
        self.thread_stop_event = threading.Event()
        self.reading_thread = threading.Thread(target = self.can_read_handle)

        self.thread_stop_event.set()
        self.reading_thread.start()
        self.thread_pause_event.set() # Set auto receive

# -------------------------- CAN and Thread methods -------------------------- #
    def connect_can_bus(self):
        bus = can.interface.Bus(bustype='socketcan', channel='can0', bitrate=500000)
        self.logger.info("CAN device connected")
        return bus

    def connect_motors(self) -> dict[int, Motor]:
        motor_dict = {mid: Motor(mid, False) for mid in range(12)}

        self.logger.info("Connecting motors...")

        # Clear the buffer
        while self.bus.recv(timeout=0) is not None:
            pass

        # Send ping to all motors
        for motor_id in range(12):
            can_id = 0x00 | (motor_id << can_config.ID_STD_OFFSET)
            msg = can.Message(
                arbitration_id=can_id,
                is_remote_frame=True,
                data=[],
                is_extended_id=False
            )

            try:
                self.bus.send(msg)
            except can.CanError as e:
                self.logger.error(f"Failed to send ping to motor {motor_id}: {e}")

        # Wait for responses
        timeout_start = time.time()
        wait_time = 0.2 # 200 ms

        while (time.time() - timeout_start) < wait_time:
            rx_msg = self.bus.recv(timeout=0.01)

            if rx_msg is None:
                continue

            if not self.is_message_valid(rx_msg):
                continue

            try:
                decoded = self.decode_msg(rx_msg)
                target_id = decoded.motor_id

                if target_id in motor_dict and not motor_dict[target_id].connected:
                    motor_dict[target_id].connected = True
                    self.logger.info(f"Motor {target_id} connected successfully")

                if all(m.connected for m in motor_dict.values()):
                    break

            except Exception as e:
                self.logger.warning(f"Error decoding message: {e}")

        # Log motors that failed to connect
        for mid, motor in motor_dict.items():
            if not motor.connected:
                self.logger.warning(f"Failed to connect motor {mid}")

        self.logger.info("Finished connecting motors")
        return motor_dict

    def shutdown(self) -> None:
        self.thread_stop_event.clear()
        self.thread_pause_event.clear()

        if self.reading_thread.is_alive():
            self.reading_thread.join()

        # Clear the buffer
        for _ in range(10):
            msg = self.bus.recv(timeout=0.001)

        self.bus.shutdown()

    def can_read_handle(self) -> None:
        while self.thread_stop_event.isSet():
            self.thread_pause_event.wait()
            
            self.received_message = self.bus.recv(timeout=0.01)
            if self.received_message is None:
                continue

            if not self.is_message_valid(self.received_message):
                self.error_flag = True
                raise RuntimeError("CAN message error detected")
            
            motor_message = self.decode_msg(self.received_message)
            self.motor_dict[motor_message.motor_id].update_param(motor_message)

    def send_motor_cmd(self, motor_id: int, cmd, value: int) -> None:
        if self.motor_dict[motor_id].connected == False:
            self.logger.warning(f"Motor {motor_id} does not connected, cannot send command")
            return
        
        if (cmd.__class__) == can_config.CAN_STD_TYPE:
            arbitration_id = 0x00 | (motor_id << can_config.ID_STD_OFFSET) | cmd.value
        else:
            arbitration_id = 0x00 | (motor_id << can_config.ID_EXT_OFFSET) | cmd.value
            arbitration_id |= can_config.CANDO_ID_EXTENDED
            
        dlc = 2
        value = int(value) & 0xFFFF
        data = [(value >> 8) & 0xFF, value & 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]

        send_msg = can.Message(
            arbitration_id=arbitration_id,
            is_extended_id=(cmd.__class__ == can_config.CAN_EXT_TYPE),
            dlc=dlc,
            data=data,
        )
        self.bus.send(send_msg)
    
    def is_message_valid(self, msg: can.Message) -> bool:
        """
        The error is referenced from parse_err_frame in cando.py
        """
        if msg.is_error_frame:

            error_code = 0

            # Bus Off
            if msg.arbitration_id & 0x00000040:
                error_code |= can_config.CAN_ERR_BUSOFF

            # RX, TX
            if len(msg.data) > 1:
                if msg.data[1] & 0x04:
                    error_code |= can_config.CAN_ERR_RX_TX_WARNING
                elif msg.data[1] & 0x10:
                    error_code |= can_config.CAN_ERR_RX_TX_PASSIVE

            if len(msg.data) > 2:
                if msg.data[2] & 0x04:
                    error_code |= can_config.CAN_ERR_STUFF
                if msg.data[2] & 0x02:
                    error_code |= can_config.CAN_ERR_FORM

                # Bit error
                if msg.data[2] & 0x10:
                    error_code |= can_config.CAN_ERR_BIT_RECESSIVE
                if msg.data[2] & 0x08:
                    error_code |= can_config.CAN_ERR_BIT_DOMINANT

            # ACK error
            if msg.arbitration_id & 0x00000020:
                error_code |= can_config.CAN_ERR_ACK

            # CRC error
            if len(msg.data) > 3:
                if msg.data[3] & 0x08:
                    error_code |= can_config.CAN_ERR_CRC

            err_tx = int(msg.data[6]) if len(msg.data) > 6 else 0
            err_rx = int(msg.data[7]) if len(msg.data) > 7 else 0

            self.logger.error("CAN Error detected: code=%s", error_code)

            if error_code == 0:
                self.logger.error(
                    "Unknown CAN error frame: arbitration_id=0x%X, data_len=%d, data=%s, err_tx=%d, err_rx=%d",
                    msg.arbitration_id,
                    len(msg.data),
                    msg.data.hex() if msg.data else "empty",
                    err_tx,
                    err_rx
                )
                
            if error_code & can_config.CAN_ERR_BUSOFF:
                self.logger.critical("CAN_ERR_BUSOFF")
            if error_code & can_config.CAN_ERR_RX_TX_WARNING:
                self.logger.error("CAN_ERR_RX_TX_WARNING")
            if error_code & can_config.CAN_ERR_RX_TX_PASSIVE:
                self.logger.error("CAN_ERR_RX_TX_PASSIVE")
            if error_code & can_config.CAN_ERR_OVERLOAD:
                self.logger.error("CAN_ERR_OVERLOAD")
            if error_code & can_config.CAN_ERR_STUFF:
                self.logger.error("CAN_ERR_STUFF")
            if error_code & can_config.CAN_ERR_FORM:
                self.logger.error("CAN_ERR_FORM")
            if error_code & can_config.CAN_ERR_ACK:
                self.logger.error("CAN_ERR_ACK")
            if error_code & can_config.CAN_ERR_BIT_RECESSIVE:
                self.logger.error("CAN_ERR_BIT_RECESSIVE")
            if error_code & can_config.CAN_ERR_BIT_DOMINANT:
                self.logger.error("CAN_ERR_BIT_DOMINANT")
            if error_code & can_config.CAN_ERR_CRC:
                self.logger.error("CAN_ERR_CRC")
                self.logger.error("err_tx=%s", err_tx)
                self.logger.error("err_rx=%s", err_rx)

            return False
        else:
            return True

    def decode_msg(self, msg: can.Message) -> MotorMessage:
        can_id = msg.arbitration_id
        can_dlc = msg.dlc
        group_msg = False
        
        if msg.is_extended_id:
            # Extended ID
            motor_id = can_id >> can_config.ID_EXT_OFFSET
            id_type = can_config.CAN_ID_TYPE.EXTENDED
            msg_id = can_id & 0xFFFFF

            if msg.is_remote_frame:
                return MotorMessage(motor_id, id_type, msg_id, None)
            
            value = struct.unpack("<h", msg.data[0:2])[0]
        else:
            # Standard ID
            motor_id = can_id >> can_config.ID_STD_OFFSET
            if (motor_id >> 10 > 0):
                group_msg = True
            id_type = can_config.CAN_ID_TYPE.STANDARD
            msg_id = can_id & 0x1F

            if msg.is_remote_frame:
                return MotorMessage(motor_id, id_type, msg_id, None)
            
            value = struct.unpack("<h",bytes(msg.data[0:2]))[0]

        if (group_msg):
            pass

        return MotorMessage(motor_id, id_type, msg_id, value)
    
    def is_motor_id_valid(self, motor_id: int) -> bool:
        if motor_id not in self.motor_dict:
            self.logger.warning(f"Motor ID {motor_id} does not exist")
            return False
        
        if not self.motor_dict[motor_id].connected:
            self.logger.warning(f"Motor ID {motor_id} is not connected")
            return False
        
        return True

# -------------------------- Motor api for leg group ------------------------- #
    def get_zero_state(self, motor_id: int) -> int:
        if not self.is_motor_id_valid(motor_id):
            return None
        
        self.send_motor_cmd(motor_id, can_config.CAN_STD_TYPE.CAN_STDID_ZERO_STATE, 0)
        zero_state = self.motor_dict[motor_id].get_param(can_config.CMD_TYPE.ZERO_STATE)
        zero_state = int(zero_state)

        return zero_state
    
    def get_motor_angle(self, motor_id) -> float:
        if not self.is_motor_id_valid(motor_id):
            return None
        
        self.send_motor_cmd(motor_id, can_config.CAN_STD_TYPE.CAN_STDID_PRESENT_REVOLUTION, 0)
        self.send_motor_cmd(motor_id, can_config.CAN_STD_TYPE.CAN_STDID_PRESENT_POSITION_DEG, 0)
        
        revolution = self.motor_dict[motor_id].get_param(can_config.CMD_TYPE.PRESENT_REVOLUTION)
        angle_16 = self.motor_dict[motor_id].get_param(can_config.CMD_TYPE.PRESENT_POSITION_DEG)
        
        angle = (revolution + (angle_16 / 65536)) / 71.96 * 2 * np.pi

        return angle
    
    def get_motor_omega(self, motor_id) -> float:
        if not self.is_motor_id_valid(motor_id):
            return None
                
        self.send_motor_cmd(motor_id, can_config.CAN_STD_TYPE.CAN_STDID_PRESENT_VELOCITY_DPS, 0)
        velocity_01 = self.motor_dict[motor_id].get_param(can_config.CMD_TYPE.PRESENT_VELOCITY_DPS)

        omega = (((velocity_01 * 10) / 65536)) / 71.96 * 2 * np.pi
        return omega
    
    def enable_motor_torque(self, motor_id: int, enable: bool) -> None:
        if not self.is_motor_id_valid(motor_id):
            return
        
        self.send_motor_cmd(motor_id, can_config.CAN_STD_TYPE.CAN_STDID_TORQUE_ENABLE, enable)

    def set_control_mode(self, motor_id: int, mode: int) -> None:
        if not self.is_motor_id_valid(motor_id):
            return
        
        self.send_motor_cmd(motor_id, can_config.CAN_STD_TYPE.CAN_STDID_CONTROL_MODE, mode)

    def set_motor_angle(self, motor_id: int, angle) -> None:
        if not self.is_motor_id_valid(motor_id):
            return
        
        self.logger.info(f"Setting motor {motor_id} angle to {angle} rad")
        
        can_signal = int(round(angle * 32768 / np.pi))
        self.send_motor_cmd(motor_id, can_config.CAN_STD_TYPE.CAN_STDID_GOAL_POSITION_DEG, can_signal)
    
    def set_motor_omega(self, motor_id: int, omega) -> None:
        if not self.is_motor_id_valid(motor_id):
            return
        
        can_signal = int(round(omega * 32768 / np.pi))
        self.send_motor_cmd(motor_id, can_config.CAN_STD_TYPE.CAN_STDID_GOAL_VELOCITY_DPS, can_signal)

    def stop_motor(self, motor_id: int) -> None:
        if not self.is_motor_id_valid(motor_id):
            return
        
        self.set_motor_omega(motor_id, 0)
        self.set_control_mode(motor_id, 0)
        self.enable_motor_torque(motor_id, False)
