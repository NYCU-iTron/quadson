import os
import time
import math
import struct
import threading
import cando
from real.can_config import *
from real.can_message import CanMessage
from real.motor import Motor

ID_STD_OFFSET = 6
ID_EXT_OFFSET = 24

class MotorManager:
	def __init__(self):
		self.device = None
		self.received_frame = cando.Frame()

		self.device = self.connect_can_device()
		self.motor_dict = self.connect_motors()

		# Set threading events
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
			raise Exception(f"{self.__class__.__name__}: No CAN device found")
		
		device = device_list[0]

		# For linux os
		if os.name == 'posix' and device.is_kernel_driver_active(0):
			device.detach_kernel_driver(0)

		# set baudrate: 500K, sample point: 87.5%
		cando.dev_set_timing(device, 1, 12, 6, 1, 6)

		cando.dev_start(device, cando.CANDO_MODE_NORMAL | cando.CANDO_MODE_NO_ECHO_BACK)

		return device
	
	def disconnect_can_device(self) -> None:
		self.thread_stop_event.clear()
		cando.dev_stop(self.device)

	def connect_motors(self) -> dict[int, Motor]:
		# Create 12 motor objects
		# Motor ID starts from 1, ends at 12
		motor_dict = {}
		for motor_id in range(1, 13):
			motor_dict[motor_id] = Motor(motor_id, False)
		
		print(f"{self.__class__.__name__} starts scaning motors")
		
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
				print(f"Motor {motor_id} no response")
				continue

			# Check if the message has error
			if self.msg_error(self.received_frame):
				motor_dict[motor_id].exist = False
				print(f"Motor {motor_id} has message error")
				continue
			
			# Decode the message
			# and check if the motor ID matches
			can_pack = self.decode_msg(self.received_frame)
			if motor_id == can_pack.motor_id:
				motor_dict[motor_id].exist = True
				print(f"Motor {motor_id} received")
			else:
				self.disconnect_can_device()
				raise Exception(f"{self.__class__.__name__}: expected motor {motor_id}, but got {can_pack.motor_id}")

		# Read frames to clear the buffer
		for x in range(10):
			cando.dev_frame_read(self.device, self.received_frame, 1)

		print(f"{self.__class__.__name__} ends scaning motors")
		return motor_dict

	def can_read_handle(self) -> None:
		while self.thread_stop_event.isSet():
			self.thread_pause_event.wait()
			
			# Check if receive any frame
			if not cando.dev_frame_read(self.device, self.received_frame, 1):
				continue

			# Check if the message has error
			if self.msg_error(self.received_frame):
				self.disconnect_can_device()
				raise Exception(f"{self.__class__.__name__}: can frame read error")
			
			can_pack = self.decode_msg(self.received_frame)
			self.motor_dict[can_pack.motor_id].update_param(can_pack.id_type, can_pack.msg_id, can_pack.data)

	def send_motor_cmd(self, motor_id: int, cmd: CAN_STD_TYPE | CAN_EXT_TYPE, value) -> None:
		if self.motor_dict[motor_id].exist == False:
			print(f"Motor {motor_id} not connect")
			return

		send_frame = cando.Frame()
		
		if (cmd.__class__) == CAN_STD_TYPE:
			send_frame.can_id = 0x00 | (motor_id << ID_STD_OFFSET) | cmd.value
		else:
			send_frame.can_id = 0x00 | (motor_id << ID_EXT_OFFSET) | cmd.value
			send_frame.can_id |= cando.CANDO_ID_EXTENDED

		send_frame.can_dlc = 2
		send_frame.data = [value >> 8, value & 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
		cando.dev_frame_send(self.device, send_frame)
		# print("sending message id:" + str(send_frame.can_id))
	
	def msg_error(self, received_frame) -> bool:
		if received_frame.can_id & cando.CANDO_ID_ERR:
			error_code, err_tx, err_rx = cando.parse_err_frame(received_frame)
			print("Error: ")
			print(error_code)

			if error_code & cando.CAN_ERR_BUSOFF:
				print(" CAN_ERR_BUSOFF")
			if error_code & cando.CAN_ERR_RX_TX_WARNING:
				print(" CAN_ERR_RX_TX_WARNING")
			if error_code & cando.CAN_ERR_RX_TX_PASSIVE:
				print(" CAN_ERR_RX_TX_PASSIVE")
			if error_code & cando.CAN_ERR_OVERLOAD:
				print(" CAN_ERR_OVERLOAD")
			if error_code & cando.CAN_ERR_STUFF:
				print(" CAN_ERR_STUFF")
			if error_code & cando.CAN_ERR_FORM:
				print(" CAN_ERR_FORM")
			if error_code & cando.CAN_ERR_ACK:
				print(" CAN_ERR_ACK")
			if error_code & cando.CAN_ERR_BIT_RECESSIVE:
				print(" CAN_ERR_BIT_RECESSIVE")
			if error_code & cando.CAN_ERR_BIT_DOMINANT:
				print(" CAN_ERR_BIT_DOMINANT")
			if error_code & cando.CAN_ERR_CRC:
				print(" CAN_ERR_CRC")
				print(" err_tx: " + str(err_tx))
				print(" err_rx: " + str(err_rx))
			return True
		else:
			return False

	def decode_msg(self, received_frame) -> CanMessage:
		# print(" is_extend : " + ("True" if received_frame.can_id & CANDO_ID_EXTENDED else "False"))
		# print(" is_rtr : " + ("True" if received_frame.can_id & CANDO_ID_RTR	else "False"))
		# print(" can_id : " + str(received_frame.can_id & CANDO_ID_MASK))
		
		can_id = received_frame.can_id & cando.CANDO_ID_MASK
		can_dlc = received_frame.can_dlc
		group_mag = False

		if (received_frame.can_id & cando.CANDO_ID_EXTENDED):
			motor_id = can_id >> 24
			id_type = CAN_ID_TYPE.EXTENDED
			msg_id = can_id & 0xFFFFF
			value = struct.unpack("<h",bytes(received_frame.data[0:2]))[0]
			# print("from motor " + str(motor_id) + " get msg: " + str(CAN_EXT_TYPE(msg_id)) + " value: " + str(value))
		else:
			motor_id = can_id >> 6
			if (motor_id>>10 > 0):
				group_mag = True
			id_type = CAN_ID_TYPE.STANDARD
			msg_id = can_id & 0x1F
			value = struct.unpack("<h",bytes(received_frame.data[0:2]))[0]
			# print("from motor " + str(motor_id) + " get msg: " + str(CAN_STD_TYPE(msg_id)) + " value: " + str(value))

		if (group_mag):
			pass

		return CanMessage(motor_id, id_type, msg_id, value)

# -------------------------- Motor api for leg group ------------------------- #
	def enable_motor_torque(self, motor_id: int, enable: bool) -> None:
		self.send_motor_cmd(motor_id, CAN_STD_TYPE.CAN_STDID_TORQUE_ENABLE, enable)

	def set_control_mode(self, motor_id: int, mode: int) -> None:
		self.send_motor_cmd(motor_id, CAN_STD_TYPE.CAN_STDID_CONTROL_MODE, mode)

	def set_motor_angle(self, motor_id, angle) -> None:
		can_signal = angle * 32768 / 180
		self.send_motor_cmd(motor_id, CAN_STD_TYPE.CAN_STDID_GOAL_POSITION_DEG, can_signal)
	
	def set_motor_omega(self, motor_id, omega) -> None:
		can_signal = omega * 32768 / 180
		self.send_motor_cmd(motor_id, CAN_STD_TYPE.CAN_STDID_GOAL_VELOCITY_DPS, can_signal)

	def get_zero_state(self, motor_id: int) -> bool:
		self.send_motor_cmd(motor_id, CAN_STD_TYPE.CAN_STDID_ZERO_DONE, 0)
		zero_state = self.motor_dict[motor_id].get_param(CMD_TYPE.ZERO_DONE)

		return zero_state == 1
	
	def get_motor_angle(self, motor_id) -> float:
		self.send_motor_cmd(motor_id, CAN_STD_TYPE.CAN_STDID_PRESENT_REVOLUTION, 0)
		self.send_motor_cmd(motor_id, CAN_STD_TYPE.CAN_STDID_PRESENT_POSITION_DEG, 0)
		
		revolution = self.motor_dict[motor_id].get_param(CMD_TYPE.PRESENT_REVOLUTION)
		angle_16 = self.motor_dict[motor_id].get_param(CMD_TYPE.PRESENT_POSITION_DEG)
		# print("rev: " + str(revolution) + "  angle: " + str(angle_16))

		angle = (revolution + (angle_16 / 65536)) / 71.96 * 2 * math.pi
		# print("degree: " + str(math.degrees(angle)))
		
		return angle

	def get_motor_omega(self, motor_id) -> float:
		self.send_motor_cmd(motor_id, CAN_STD_TYPE.CAN_STDID_PRESENT_VELOCITY_DPS, 0)
		velocity_01 = self.motor_dict[motor_id].get_param(CMD_TYPE.PRESENT_VELOCITY_DPS)

		omega = (((velocity_01 * 10) / 65536)) / 71.96 * 2 * math.pi
		# print("omega: " + str(omega))
		return omega