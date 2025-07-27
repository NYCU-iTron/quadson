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

		# Create 12 motor objects
		self.motor_dict = {}
		for x in range(1, 13):
			# Motor ID starts from 1, ends at 12
			self.motor_dict[x] = Motor(x, False)

		self.connect_can_device()
		self.thread_pause_event = threading.Event()
		self.thread_stop_event = threading.Event()
		self.reading_thread = threading.Thread(target = self._can_read_handle)

		self.thread_stop_event.set()
		self.reading_thread.start()

		self.scan_motors()
		self.set_auto_receive(True)
	
	def connect_can_device(self) -> None:
		device_list = cando.list_scan()

		if len(device_list) == 0:
			raise Exception("No CAN device found!")
		
		self.device = device_list[0]

		# For linux os
		if os.name == 'posix' and self.device.is_kernel_driver_active(0):
			self.device.detach_kernel_driver(0)

		# set baudrate: 500K, sample point: 87.5%
		cando.dev_set_timing(self.device, 1, 12, 6, 1, 6)

		cando.dev_start(self.device, cando.CANDO_MODE_NORMAL | cando.CANDO_MODE_NO_ECHO_BACK)

	def disconnect_can_device(self) -> None:
		self.thread_stop_event.clear()
		cando.dev_stop(self.device)
	
	def send_motor_cmd(self, motor_index, cmd, value) -> None:
		if self.motor_dict[motor_index].exist == False:
			print("Motor unconnected")
			return

		send_frame = cando.Frame()
		
		if (cmd.__class__) == CAN_STD_TYPE:
			send_frame.can_id = 0x00 | (motor_index << ID_STD_OFFSET) | cmd.value
		else:
			send_frame.can_id = 0x00 | (motor_index << ID_EXT_OFFSET) | cmd.value
			send_frame.can_id |= cando.CANDO_ID_EXTENDED

		send_frame.can_dlc = 2
		send_frame.data = [value >> 8, value & 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
		cando.dev_frame_send(self.device, send_frame)
		# print("sending message id:" + str(send_frame.can_id))
		
	def get_angle(self, motor_id) -> float:
		self.send_motor_cmd(motor_id, CAN_STD_TYPE.CAN_STDID_PRESENT_REVOLUTION, 0)
		self.send_motor_cmd(motor_id, CAN_STD_TYPE.CAN_STDID_PRESENT_POSITION_DEG, 0)
		
		revolution = self.motor_dict[motor_id].get_param(CMD_TYPE.PRESENT_REVOLUTION)
		angle_16 = self.motor_dict[motor_id].get_param(CMD_TYPE.PRESENT_POSITION_DEG)
		# print("rev: " + str(revolution) + "  angle: " + str(angle_16))

		angle = (revolution + ((angle_16) / 65536))/71.96 * 2*math.pi
		# print("degree: " + str(math.degrees(angle)))
		
		return angle

	def set_angle(self, motor_id, angle) -> None:
		can_signal = angle * 32768 / 180
		self.send_motor_cmd(motor_id, CAN_STD_TYPE.CAN_STDID_GOAL_POSITION_DEG, can_signal)

	def get_omega(self, motor_id) -> float:
		self.send_motor_cmd(motor_id, CAN_STD_TYPE.CAN_STDID_PRESENT_VELOCITY_DPS, 0)
		velocity_01 = self.motor_dict[motor_id].get_param(CMD_TYPE.PRESENT_VELOCITY_DPS)

		omega = (((velocity_01 * 10) / 65536)) / 71.96 * 2 * math.pi
		# print("omega: " + str(omega))
		return omega
	
	def set_omega(self, motor_id, omega):
		can_signal = omega * 32768 / 180
		self.send_motor_cmd(motor_id, CAN_STD_TYPE.CAN_STDID_GOAL_VELOCITY_DPS, can_signal)

	def _can_read_handle(self) -> None:
		while self.thread_stop_event.isSet():
			self.thread_pause_event.wait()
			if cando.dev_frame_read(self.device, self.received_frame, 1):
				if (self._msg_error(self.received_frame)):
					self.disconnect_can_device()
					raise Exception("Reading error")
				self._decode_msg(self.received_frame)
	
	def scan_motors(self) -> None:
		for i in range(1, 13):
			# Prepare frame
			send_frame = cando.Frame()
			send_frame.can_id = 0x00 | (i << 6)
			send_frame.can_id |= cando.CANDO_ID_RTR
			# send_frame.can_dlc = 8
			# send_frame.data = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08]
			cando.dev_frame_send(self.device, send_frame)
			
			time.sleep(0.001)

			if cando.dev_frame_read(self.device, self.received_frame, 1):
				if not self._msg_error(self.received_frame):
					can_pack = self._decode_msg(self.received_frame)
					if i == can_pack.motor_id:
						self.motor_dict[i].exist = True
						print("Receive from motor " + str(i))
					else:
						self.disconnect_can_device()
						raise Exception("Scanning error")
			else:
				self.motor_dict[i].exist = False
				print("Motor " + str(i) + " no response")

		for x in range(10):
			cando.dev_frame_read(self.device, self.received_frame, 1)

	def set_auto_receive(self, enable) -> None:
		if enable:
			self.thread_pause_event.set()
		else:
			self.thread_pause_event.clear()

	def _msg_error(self, received_frame) -> bool:
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

	def _decode_msg(self, received_frame) -> CanMessage:
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

		self.motor_dict[motor_id].update_param(id_type, msg_id, value)
		
		return CanMessage(motor_id, id_type, msg_id, value)