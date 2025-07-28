import pybullet as p
import numpy as np
from common.config import Config

class Interface:
    def __init__(self, type='gui', target='motor'):
        """
        type: str
            'gui': GUI interface
            'num': Numerical interface
        target: str
            'motor': 12 motor of 4 legs
            'orientation': orientation of body
        """
        valid_types = ['gui', 'num', 'model']
        valid_targets = ['motor', 'orientation', 'ee_offset']

        if type not in valid_types:
            print('[ERROR] Invalid control type')
        if target not in valid_targets:
            print('[ERROR] Invalid control target')

        self.type = type
        self.target = target
        self.config = Config()

        self.init_handlers = {
            ('gui', 'motor'): self._init_gui_motor,
            ('num', 'motor'): self._init_num_motor,
            ('gui', 'orientation'): self._init_gui_orientation,
            ('num', 'orientation'): self._init_num_orientation,
            ('model', 'ee_offset'): self._init_model_ee_offset,
        }

        # Set the command value to the output dictionary
        self.cmd_handlers = {
            ('gui', 'motor'): self._send_cmd_gui_motor,
            ('num', 'motor'): self._send_cmd_num_motor,
            ('gui', 'orientation'): self._send_cmd_gui_orientation,
            ('num', 'orientation'): self._send_cmd_num_orientation,
            ('model', 'ee_offset'): self._send_cmd_model_ee_offset,
        }

        self.init_handlers[self.type, self.target]()
    
    def _init_gui_motor(self):
        self.output_dict = {self.target: {}}
        self.sliders = {}
        for leg_name, leg_motors in self.config.motor_dict.items():
            self.output_dict[self.target][leg_name] = []
            for motor_name in leg_motors:
                slider_id = p.addUserDebugParameter(motor_name, -np.pi, np.pi, 0)
                self.sliders[motor_name] = slider_id
                self.output_dict[self.target][leg_name].append(0)
    
    def _init_gui_orientation(self):
        self.sliders = {
            'roll': p.addUserDebugParameter('roll', -np.pi, np.pi, 0),
            'pitch': p.addUserDebugParameter('pitch', -np.pi, np.pi, 0),
            'yaw': p.addUserDebugParameter('yaw', -np.pi, np.pi, 0),
        }
        self.output_dict = {
            self.target: {'roll': 0,
                                        'pitch': 0,
                                        'yaw': 0,}
        }

    def _init_num_motor(self):
        self.output_dict = {self.target:{}}
        for leg_name, leg_motors in self.config.motor_dict.items():
            self.output_dict[self.target][leg_name] = []
            for motor_name in leg_motors:
                self.output_dict[self.target][leg_name].append(0)

    def _init_num_orientation(self):
        self.output_dict = {
            self.target: {'roll': 0, 'pitch': 0, 'yaw': 0,}}

    def _init_model_ee_offset(self):
        self.output_dict = {self.target:{}}
        for leg_name, leg_motors in self.config.motor_dict.items():
            self.output_dict[self.target][leg_name] = []
            for motor_name in leg_motors:
                self.output_dict[self.target][leg_name].append(0)

    def _send_cmd_gui_motor(self):
        for leg_name, leg_motors in self.config.motor_dict.items():
            motor_angles = []
            for motor_name in leg_motors:
                angle = p.readUserDebugParameter(self.sliders[motor_name])
                motor_angles.append(angle)
            self.output_dict[self.target][leg_name] = motor_angles

    def _send_cmd_gui_orientation(self):
        for key, slider_id in self.sliders.items():
            self.output_dict[self.target][key] = p.readUserDebugParameter(slider_id)

    def _send_cmd_num_motor(self, cmd_dict):
        """
        cmd_dict: dict{'leg_name': {'motor_name': float}, ...}

        Valid leg names: 
            'fl', 'fr', 'rl', 'rr'

        Valid motor names: 
            '${leg_name}_joint0',
            '${leg_name}_joint1',
            '${leg_name}_joint2',
            '${leg_name}_joint4',
            '${leg_name}_joint5'
        """
        for leg_name, leg_motors in self.config.motor_dict.items():
            motor_angles = []
            for motor_name in leg_motors:
                motor_angles.append(cmd_dict[leg_name][motor_name])
            self.output_dict[self.target][leg_name] = motor_angles

    def _send_cmd_num_orientation(self, cmd_dict):
        """
        cmd_dict: dict
            {
                'roll': float,
                'pitch': float,
                'yaw': float,
            }
        """
        for key, value in cmd_dict.items():
            self.output_dict[self.target][key] = value

    def _send_cmd_model_ee_offset(self, cmd_dict):
        for leg_name in self.config.legs:
            self.output_dict[self.target][leg_name] = cmd_dict[leg_name]

    def send_cmd(self, cmd_dict=None):
        key = (self.type, self.target)
        handler = self.cmd_handlers.get(key, self._default_handler)
        if cmd_dict is None:
            handler()
        else:
            handler(cmd_dict)

    def _default_handler(self):
        print("Invalid interface type or target")
        