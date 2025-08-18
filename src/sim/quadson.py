import pybullet as p
import numpy as np
from pathlib import Path
import logging
from common.config import LegName, GaitType, CommandType, Command
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

        self.locomotion = Locomotion(GaitType.TROT)
        self.body_kinematics = BodyKinematics()
        self.motor_manager = MotorManager(self.robot_id)
        
        self.leg_dict = {}
        for leg_name in LegName:
            self.leg_dict[leg_name] = Leg(leg_name, self.motor_manager)

        self.sim_time = 0
        self.time_step = 1 / 240
        self.linear_vel = [0, 0, 0]
        self.robot_state = None

        self.setup_colors()
        self.setup_friction()

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
        self.sim_time += self.time_step
        self.prev_linear_vel = self.linear_vel
        self.linear_vel, self.angular_vel = p.getBaseVelocity(self.robot_id)
        self.linear_acc = [
            (self.linear_vel[0] - self.prev_linear_vel[0]) / self.time_step,
            (self.linear_vel[1] - self.prev_linear_vel[1]) / self.time_step,
            (self.linear_vel[2] - self.prev_linear_vel[2]) / self.time_step
        ]
        self.pos, self.ori = p.getBasePositionAndOrientation(self.robot_id)
        self.euler_ori = p.getEulerFromQuaternion(self.ori)
        
    def process_command(self, command: Command) -> None:
        if command.type == CommandType.MOTOR_ANGLES:
            for name, motor_angles in command.data.items():
                if name not in self.leg_dict:
                    self.logger.error(f"Invalid leg name: {name}")
                    continue
                self.leg_dict[name].set_motor_angles(motor_angles)

        elif command.type == CommandType.EE_POINTS:
            for name, ee_point in command.data.items():
                if name not in self.leg_dict:
                    self.logger.error(f"Invalid leg name: {name}")
                    continue
                self.leg_dict[name].set_ee_point(ee_point)

        elif command.type == CommandType.ORIENTATION:
            pass

        elif command.type == CommandType.TEST_LOCOMOTION:
            ee_points = self.locomotion.get_ee_points(self.robot_state.time)
            ee_offsets = self.get_model_calibration(self.robot_state)
            # ...

        elif command.type == CommandType.TWIST:
            pass

        else:
            self.logger.error("Wrong input command type")
    
    def get_model_calibration(self, robot_state) -> dict:
        pass

# ------------------------------- PPO Training ------------------------------- #
    def get_observation(self) -> dict:
        # Get body state
        self.linear_vel, self.angular_vel = p.getBaseVelocity(self.robot_id)
        self.pos, self.ori = p.getBasePositionAndOrientation(self.robot_id)
        self.euler_ori = p.getEulerFromQuaternion(self.ori)

        # Round the values
        digits = 5
        pos = self.round_tuple(self.pos, digits)
        linear_vel = self.round_tuple(self.linear_vel, digits)
        angular_vel = self.round_tuple(self.angular_vel, digits)
        euler_ori = self.round_tuple(self.euler_ori, digits)

        # Get joint state
        joints = []
        for name in LegName:
            motor_angles = self.leg_dict[name].get_motor_angles()
            joints.extend(motor_angles)
        joints = np.array(joints)

        # Get phase
        phase = self.locomotion.get_current_phase(self.sim_time)
        phase_list = []
        for name in LegName:
            phase_list.append(np.sin(phase[name.value]))
            phase_list.append(np.cos(phase[name.value]))
        phase_list = np.array(phase_list)

        observation = np.concatenate([euler_ori, linear_vel, angular_vel, joints, phase_list])
        return observation
    
    def get_angular_velocity(self) -> tuple:
        return self.angular_vel
    
    def get_linear_velocity(self) -> tuple:
        return self.linear_vel
    
    def get_orientation_rpy(self) -> tuple:
        return self.euler_ori
    
    def get_position(self) -> tuple:
        return self.pos
    
    def round_tuple(self, t, digits) -> tuple:
        return tuple(round(x, digits) for x in t)
    