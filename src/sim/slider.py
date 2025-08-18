import pybullet as p
import numpy as np
import logging
from common.config import LegName, OrientationType, CommandType, Command

class Slider:
    def __init__(self, command_type: CommandType):
        self.logger = logging.getLogger(__name__)

        function_map = {
            CommandType.MOTOR_ANGLES: (self.init_motor_angle_slider, self.get_motor_angle_command),
            CommandType.EE_POINTS: (self.init_ee_point_slider, self.get_ee_point_command),
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
            leg_id = 3 * name.value
            id_0 = leg_id + 0
            id_1 = leg_id + 1
            id_2 = leg_id + 2

            slider_id_0 = p.addUserDebugParameter(str(id_0), -np.pi, np.pi, 0)
            slider_id_1 = p.addUserDebugParameter(str(id_1), 0, 2 * np.pi, np.pi)
            slider_id_2 = p.addUserDebugParameter(str(id_2), -np.pi / 2, 3 * np.pi / 2, np.pi / 2)

            self.sliders[id_0] = slider_id_0
            self.sliders[id_1] = slider_id_1
            self.sliders[id_2] = slider_id_2

    def init_ee_point_slider(self) -> None:
        self.sliders = {}
        for name in LegName:
            leg_id = 3 * name.value
            id_0 = leg_id + 0
            id_1 = leg_id + 1
            id_2 = leg_id + 2

            slider_id_0 = p.addUserDebugParameter(str(id_0), -5, 20, -2.16)
            slider_id_1 = p.addUserDebugParameter(str(id_1), -30, 0, -17.03)
            slider_id_2 = p.addUserDebugParameter(str(id_2), -10, 10, 0)

            self.sliders[id_0] = slider_id_0
            self.sliders[id_1] = slider_id_1
            self.sliders[id_2] = slider_id_2

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
            type=CommandType.MOTOR_ANGLES,
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
            type=CommandType.EE_POINTS,
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
    