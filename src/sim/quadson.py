import pybullet as p
from stable_baselines3 import PPO
import numpy as np
from pathlib import Path
from copy import deepcopy
import logging
from common.config import LegName, GaitType, CommandType, Command, RobotState
from common.body_kinematics import BodyKinematics
from common.locomotion import Locomotion
from sim.leg import Leg
from sim.motor_manager import MotorManager

class Quadson:
    def __init__(self):
        self.logger = logging.getLogger(__name__)
        
        # Load the mesh
        urdf_path = Path(__file__).resolve().parent.parent.parent / "assets/whole_body/urdf/quadson_modified.urdf"
        self.robot_id = p.loadURDF(
            str(urdf_path),
            basePosition=[0, 0, 0.2],
            useFixedBase=False
        )

        # Initialize components
        self.locomotion = Locomotion(GaitType.TROT)
        self.body_kinematics = BodyKinematics()
        self.motor_manager = MotorManager(self.robot_id)
        
        # Initialize legs
        self.leg_dict = {}
        for name in LegName:
            self.leg_dict[name] = Leg(name, self.motor_manager)

        self.time_step = 1 / 240

        # Initialize robot state
        self.robot_state = RobotState(
            time = 0,
            linear_velocity=[0, 0, 0],
        )
        self.prev_robot_state = self.robot_state

        self.setup_colors()
        self.setup_friction()

        self.logger.info("Quadson initialized successfully.")

    def setup_friction(self) -> None:
        joint2s = [2, 7, 12, 17]
        for joint_id in joint2s: 
            p.changeDynamics(
                self.robot_id, 
                joint_id,
                lateralFriction = 1.0,
                spinningFriction = 0.1,
                rollingFriction = 0.03
            )

    def setup_colors(self) -> None:
        base_color = [0.2, 0.2, 0.2, 1]
        shoulder_color = [0.8, 0.5, 0.2, 1]
        leg_color = [0.7, 0.7, 0.7, 1]

        # Set base link color
        p.changeVisualShape(self.robot_id, -1, rgbaColor=base_color)

        # Iterate through all links
        joint0s = [0, 5, 10, 15]
        for joint_index in range(20):
            if joint_index in joint0s:
                p.changeVisualShape(self.robot_id, joint_index, rgbaColor=shoulder_color)
            else:
                p.changeVisualShape(self.robot_id, joint_index, rgbaColor=leg_color)

    def update_state(self) -> None:
        self.prev_robot_state = deepcopy(self.robot_state)
        self.robot_state.time = self.prev_robot_state.time + self.time_step

        pose, orientation = p.getBasePositionAndOrientation(self.robot_id)
        euler_orientation = p.getEulerFromQuaternion(orientation)
        linear_velocity, angular_velocity = p.getBaseVelocity(self.robot_id)
        linear_accleration = [
            (current - previous) / self.time_step
            for current, previous in zip(linear_velocity, self.prev_robot_state.linear_velocity)
        ]

        # Round the values
        self.robot_state.pose = self.round_tuple(pose, 5)
        self.robot_state.orientation = self.round_tuple(orientation, 5)
        self.robot_state.euler_orientation = self.round_tuple(euler_orientation, 5)
        self.robot_state.linear_velocity = self.round_tuple(linear_velocity, 5)
        self.robot_state.angular_velocity = self.round_tuple(angular_velocity, 5)
        self.robot_state.linear_accleration = self.round_tuple(linear_accleration, 5)

        # Get joint state
        joints = []
        for name in LegName:
            motor_angles = self.leg_dict[name].get_motor_angles()
            joints.extend(motor_angles)
        joints = np.array(joints)
        self.robot_state.joints = joints

        # Get phase
        phase_dict = self.locomotion.get_current_phase(self.robot_state.time)
        phases = []
        for name in LegName:
            phases.append(np.sin(phase_dict[name]))
            phases.append(np.cos(phase_dict[name]))
        phases = np.array(phases)
        self.robot_state.phases = phases

    def get_robot_state(self) -> RobotState:
        self.update_state()
        return self.robot_state

    def process_command(self, command: Command) -> None:
        if command.type == CommandType.MOTOR_ANGLES:
            for name, motor_angles in command.data.items():
                self.leg_dict[name].set_motor_angles(motor_angles)

        elif command.type == CommandType.EE_POINTS:
            for name, ee_point in command.data.items():
                self.leg_dict[name].set_ee_point(ee_point)

        elif command.type == CommandType.ORIENTATION:
            pass

        elif command.type == CommandType.TEST_LOCOMOTION:
            ee_points = self.locomotion.get_ee_points(self.robot_state.time)
            for name, ee_point in ee_points.items():
                self.leg_dict[name].set_ee_point(ee_point)

        elif command.type == CommandType.TEST_MODEL_CALIBRATION:            
            ee_points = self.locomotion.get_ee_points(self.robot_state.time)
            ee_offsets = self.get_model_calibration(self.robot_state)
            
            for name, ee_point in ee_points.items():
                # Apply the model calibration offsets
                calibrated_ee_point = np.array(ee_point) + np.array(ee_offsets[name])
                self.leg_dict[name].set_ee_point(calibrated_ee_point)
        
        elif command.type == CommandType.TRAIN_MODEL:
            ee_points = self.locomotion.get_ee_points(self.robot_state.time)
            ee_offsets = command.data

            for name, ee_point in ee_points.items():
                # Apply the model calibration offsets
                calibrated_ee_point = np.array(ee_point) + np.array(ee_offsets[name])
                self.leg_dict[name].set_ee_point(calibrated_ee_point)

        elif command.type == CommandType.TWIST:
            pass

        else:
            self.logger.error("Wrong input command type")
    
    def load_model(self, model_path: str) -> None:
        model_path = Path(__file__).resolve().parent.parent.parent / "assets/" / model_path
        self.model = PPO.load(model_path, device='cpu', print_system_info=True)

    def get_model_calibration(self, robot_state: RobotState) -> dict:
        if not hasattr(self, 'model') or self.model is None:
            self.logger.error("Model not loaded.")
            return None
        
        observation = np.concatenate([
            robot_state.euler_orientation,
            robot_state.linear_velocity,
            robot_state.angular_velocity,
            robot_state.joints,
            robot_state.phases
        ])

        action, _ = self.model.predict(observation, deterministic=True)

        ee_offsets = {}
        for name in LegName:
            start = name.value * 3
            end = start + 3
            ee_offsets[name] = action[start:end].tolist()

        return ee_offsets
    
    def round_tuple(self, values, digits) -> tuple:
        return tuple(round(v, digits) for v in values)
    