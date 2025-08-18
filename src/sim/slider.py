import pybullet as p
import numpy as np
import logging
from common.config import LegName, OrientationType, CommandType, Command

class Slider:
    def __init__(self, command_type: CommandType):
        self.logger = logging.getLogger(__name__)

        function_map = {
            CommandType.MOTOR_ANGLE: (self.init_motor_angle_slider, self.get_motor_angle_command),
            CommandType.EE_POINT: (self.init_ee_point_slider, self.get_ee_point_command),
            CommandType.ORIENTATION: (self.init_orientation_slider, self.get_orientation_command),
        }

        if command_type in function_map:
            init_method, command_method = function_map[command_type]
            init_method()
            self.get_command = command_method
        else:
            self.logger.error("Unsupported command type for slider initialization: %s", command_type)
            self.get_command = None

    def init_motor_angle_slider(self) -> None:
        self.sliders = {}
        for name in LegName:
            for i in range(3):
                id = 3 * name.value + i
                slider_id = p.addUserDebugParameter(str(id), -np.pi, np.pi, 0)
                self.sliders[id] = slider_id

    def init_ee_point_slider(self) -> None:
        self.sliders = {}
        for name in LegName:
            for i in range(3): # x, y, z
                id = 3 * name.value + i
                slider_id = p.addUserDebugParameter(str(id), -np.pi, np.pi, 0)
                self.sliders[id] = slider_id

    def init_orientation_slider(self) -> None:
        self.sliders = {
            OrientationType.ROLL: p.addUserDebugParameter(OrientationType.ROLL.name, -np.pi, np.pi, 0),
            OrientationType.PITCH: p.addUserDebugParameter(OrientationType.PITCH.name, -np.pi, np.pi, 0),
            OrientationType.YAW: p.addUserDebugParameter(OrientationType.YAW.name, -np.pi, np.pi, 0),
        }
    
    def get_motor_angle_command(self) -> Command:
        data = {}
        for name in LegName:
            motor_angles = []
            for i in range(3):
                id = 3 * name.value + i
                angle = p.readUserDebugParameter(self.sliders[id])
                motor_angles.append(angle)
            data[name] = motor_angles
        
        command = Command(
            type=CommandType.MOTOR_ANGLE,
            data=data
        )
        return command
    
    def get_ee_point_command(self) -> Command:
        data = {}
        for name in LegName:
            ee_point = []
            for i in range(3):
                id = 3 * name.value + i
                angle = p.readUserDebugParameter(self.sliders[id])
                ee_point.append(angle)
            data[name] = ee_point
        
        command = Command(
            type=CommandType.EE_POINT,
            data=data
        )
        return command

    def get_orientation_command(self) -> Command:
        data = {}
        for type, id in self.sliders.items():
            data[type] = p.readUserDebugParameter(id)
        
        command = Command(
            type=CommandType.ORIENTATION,
            data=data
        )

        return command
    