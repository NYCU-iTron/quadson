import real.can_config as can_config
from real.motor_message import MotorMessage

class Motor:
    def __init__(self, motor_id: int, connected: bool):
        self.motor_id = motor_id
        self.connected = connected
        self.params = dict(can_config.Param_Dict)
    
    def get_param(self, param_id: can_config.CMD_TYPE) -> float:
        return self.params[param_id.value]

    def update_param(self, motor_message: MotorMessage) -> None:
        id_type = motor_message.id_type
        msg_id = motor_message.msg_id
        data = motor_message.data

        # EXTENDED ID
        if id_type == can_config.CAN_ID_TYPE.EXTENDED:
            self.params[msg_id] = data
            return
        
        # Standard ID
        if msg_id == can_config.CAN_STD_TYPE.CAN_STDID_TORQUE_ENABLE:
            self.params[can_config.CMD_TYPE.TORQUE_ENABLE.value] = data
        elif msg_id == can_config.CAN_STD_TYPE.CAN_STDID_STATE_MACHINE:
            self.params[can_config.CMD_TYPE.STATE_MACHINE.value] = data
        elif msg_id == can_config.CAN_STD_TYPE.CAN_STDID_CONTROL_MODE:
            self.params[can_config.CMD_TYPE.CONTROL_MODE.value] = data
    
        # Goal
        elif msg_id == can_config.CAN_STD_TYPE.CAN_STDID_GOAL_REVOLUTION:
            self.params[can_config.CMD_TYPE.GOAL_REVOLUTION.value] = data
        elif msg_id == can_config.CAN_STD_TYPE.CAN_STDID_GOAL_POSITION_DEG:
            self.params[can_config.CMD_TYPE.GOAL_POSITION_DEG.value] = data
        elif msg_id == can_config.CAN_STD_TYPE.CAN_STDID_GOAL_VELOCITY_DPS:
            self.params[can_config.CMD_TYPE.GOAL_VELOCITY_DPS.value] = data
        elif msg_id == can_config.CAN_STD_TYPE.CAN_STDID_GOAL_TORQUE_CURRENT_MA:
            self.params[can_config.CMD_TYPE.GOAL_TORQUE_CURRENT_MA.value] = data
        elif msg_id == can_config.CAN_STD_TYPE.CAN_STDID_GOAL_FLUX_CURRENT_MA:
            self.params[can_config.CMD_TYPE.GOAL_FLUX_CURRENT_MA.value] = data
    
        # Present
        elif msg_id == can_config.CAN_STD_TYPE.CAN_STDID_PRESENT_REVOLUTION:
            self.params[can_config.CMD_TYPE.PRESENT_REVOLUTION.value] = data
        elif msg_id == can_config.CAN_STD_TYPE.CAN_STDID_PRESENT_POSITION_DEG:
            self.params[can_config.CMD_TYPE.PRESENT_POSITION_DEG.value] = data
        elif msg_id == can_config.CAN_STD_TYPE.CAN_STDID_PRESENT_VELOCITY_DPS:
            self.params[can_config.CMD_TYPE.PRESENT_VELOCITY_DPS.value] = data
        elif msg_id == can_config.CAN_STD_TYPE.CAN_STDID_PRESENT_TORQUE_CURRENT_MA:
            self.params[can_config.CMD_TYPE.PRESENT_TORQUE_CURRENT_MA.value] = data
        elif msg_id == can_config.CAN_STD_TYPE.CAN_STDID_PRESENT_FLUX_CURRENT_MA:
            self.params[can_config.CMD_TYPE.PRESENT_FLUX_CURRENT_MA.value] = data
    
        elif msg_id == can_config.CAN_STD_TYPE.CAN_STDID_PRESENT_VOLTAGE:
            self.params[can_config.CMD_TYPE.PRESENT_VOLTAGE.value] = data
        elif msg_id == can_config.CAN_STD_TYPE.CAN_STDID_PRESENT_TEMPERATURE:
            self.params[can_config.CMD_TYPE.PRESENT_TEMPERATURE.value] = data
        elif msg_id == can_config.CAN_STD_TYPE.CAN_STDID_ZERO_STATE:
            self.params[can_config.CMD_TYPE.ZERO_STATE.value] = data