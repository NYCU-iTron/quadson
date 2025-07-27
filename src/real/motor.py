from cando import *
from real.can_config import *

class Motor:
  def __init__(self, motor_id: int, exist: bool):
    self.motor_id = motor_id
    self.exist = exist
    self.params = dict(Param_Dict)
    
  def get_param(self, param_id: CMD_TYPE) -> float:
    return self.params[param_id.value]

  def update_param(self, id_type: CAN_ID_TYPE, msg_id: CAN_STD_TYPE, value: float) -> None:
    if id_type == CAN_ID_TYPE.EXTENDED:
      self.params[msg_id] = value
    else:
      if msg_id == CAN_STD_TYPE.CAN_STDID_TORQUE_ENABLE:
        self.params[CMD_TYPE.TORQUE_ENABLE.value] = value
      elif msg_id == CAN_STD_TYPE.CAN_STDID_STATE_MACHINE:
        self.params[CMD_TYPE.STATE_MACHINE.value] = value
      elif msg_id == CAN_STD_TYPE.CAN_STDID_CONTROL_MODE:
        self.params[CMD_TYPE.CONTROL_MODE.value] = value

      # Goal
      elif msg_id == CAN_STD_TYPE.CAN_STDID_GOAL_REVOLUTION:
        self.params[CMD_TYPE.GOAL_REVOLUTION.value] = value
      elif msg_id == CAN_STD_TYPE.CAN_STDID_GOAL_POSITION_DEG:
        self.params[CMD_TYPE.GOAL_POSITION_DEG.value] = value
      elif msg_id == CAN_STD_TYPE.CAN_STDID_GOAL_VELOCITY_DPS:
        self.params[CMD_TYPE.GOAL_VELOCITY_DPS.value] = value
      elif msg_id == CAN_STD_TYPE.CAN_STDID_GOAL_TORQUE_CURRENT_MA:
        self.params[CMD_TYPE.GOAL_TORQUE_CURRENT_MA.value] = value
      elif msg_id == CAN_STD_TYPE.CAN_STDID_GOAL_FLUX_CURRENT_MA:
        self.params[CMD_TYPE.GOAL_FLUX_CURRENT_MA.value] = value

      # Present
      elif msg_id == CAN_STD_TYPE.CAN_STDID_PRESENT_REVOLUTION:
        self.params[CMD_TYPE.PRESENT_REVOLUTION.value] = value
      elif msg_id == CAN_STD_TYPE.CAN_STDID_PRESENT_POSITION_DEG:
        self.params[CMD_TYPE.PRESENT_POSITION_DEG.value] = value
      elif msg_id == CAN_STD_TYPE.CAN_STDID_PRESENT_VELOCITY_DPS:
        self.params[CMD_TYPE.PRESENT_VELOCITY_DPS.value] = value
      elif msg_id == CAN_STD_TYPE.CAN_STDID_PRESENT_TORQUE_CURRENT_MA:
        self.params[CMD_TYPE.PRESENT_TORQUE_CURRENT_MA.value] = value
      elif msg_id == CAN_STD_TYPE.CAN_STDID_PRESENT_FLUX_CURRENT_MA:
        self.params[CMD_TYPE.PRESENT_FLUX_CURRENT_MA.value] = value

      elif msg_id == CAN_STD_TYPE.CAN_STDID_PRESENT_VOLTAGE:
        self.params[CMD_TYPE.PRESENT_VOLTAGE.value] = value
      elif msg_id == CAN_STD_TYPE.CAN_STDID_PRESENT_TEMPERATURE:
        self.params[CMD_TYPE.PRESENT_TEMPERATURE.value] = value
      elif msg_id == CAN_STD_TYPE.CAN_STDID_MOVING:
        self.params[CMD_TYPE.MOVING.value] = value