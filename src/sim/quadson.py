import pybullet as p
import numpy as np
from typing import Dict
from pathlib import Path
from common.config import Config
from common.body_kinematics import BodyKinematics
from common.locomotion import Locomotion
from sim.interface import Interface
from sim.leg_group import LegGroup

class Quadson:
    def __init__(self, interface: Interface = None):
        urdf_path = Path(__file__).resolve().parent.parent.parent / "assets/whole_body/urdf/quadson_modified.urdf"
        self.robot_id = p.loadURDF(
            str(urdf_path),
            basePosition=[0, 0, 0.2],
            useFixedBase=False)
        self.config = Config()
        self.body_kinematics = BodyKinematics()
        self.locomotion = Locomotion(gait_type='trot')
        self.interface = interface
        self.joint_dict = self.setup_joint_dict()
        self.leg_group_dict = self.setup_leg_group()
        self.setup_colors()
        self.setup_friction()

        self.sim_time = 0
        self.time_step = 1/240
        self.linear_vel = [0, 0, 0]

        self.cmd_handlers = {
            'motor': self._update_motor,
            'orientation': self._update_orientation,
            'ee_offset': self._update_ee_offset, # PPO model input
        }

    def setup_joint_dict(self) -> Dict:
        """
        Return a dictionary of joint names and their indices
        """
        joint_dict = {
            p.getJointInfo(self.robot_id, i)[1].decode("utf-8"): i
            for i in range(p.getNumJoints(self.robot_id))
        }
        return joint_dict
    
    def setup_colors(self) -> None:
        base_color = [0.2, 0.2, 0.2, 1]
        shoulder_color = [0.8, 0.5, 0.2, 1]
        leg_color = [0.7, 0.7, 0.7, 1]
        default_color = [0.2, 0.2, 0.2, 1] # Dark gray for anything else

        # Set base link color
        p.changeVisualShape(self.robot_id, -1, rgbaColor=base_color)

        # Iterate through all links
        for joint_name, joint_index in self.joint_dict.items():
            if 'joint0' in joint_name.lower():
                color = shoulder_color
            elif 'joint' in joint_name.lower():
                color = leg_color
            else:
                color = default_color

            p.changeVisualShape(self.robot_id, joint_index, rgbaColor=color)

    def setup_friction(self) -> None:
        for joint_name, joint_index in self.joint_dict.items():
            if 'joint2' in joint_name.lower():
                p.changeDynamics(self.robot_id, joint_index,
                                    lateralFriction=1.0,
                                    spinningFriction=0.1,
                                    rollingFriction=0.03)
                
    def setup_leg_group(self) -> Dict:
        """
        Initialize leg groups dynamically from joint dictionary.
        Return a dictionary of leg name and the leg group object.
        """
        leg_group_dict = {}
        for leg_name, leg_joints in self.config.joint_dict.items():
            if not all(joint in self.joint_dict for joint in leg_joints):
                print(f"[Warning] Skipping {leg_name} leg group. Some joints are missing in URDF.")
                continue
            joint_indices = [self.joint_dict[joint_name] for joint_name in leg_joints]
            leg_group_dict[leg_name] = LegGroup(self.robot_id, joint_indices)
        
        return leg_group_dict

    def update(self, cmd_dict=None) -> None:
        # ----------------------------- Update the state ----------------------------- #
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

        # ---------------------------- Process the command --------------------------- #
        if cmd_dict != None:
            self.interface.send_cmd(cmd_dict)
        else:
            self.interface.send_cmd()

        self.input_dict = self.interface.output_dict[self.interface.target]
        self.cmd_handlers[self.interface.target]()

    def step(self, time: float) -> None:
        ee_points = self.locomotion.get_ee_points(time)
        for leg_name, ee_point in ee_points.items():
            self.leg_group_dict[leg_name].set_ee_point(ee_point)

    def _update_motor(self) -> None:
        base_angles = np.array([0, np.pi, np.pi/2])
        for leg_name in self.config.legs:
            motor_angles = base_angles - np.array(self.input_dict[leg_name])
            self.leg_group_dict[leg_name].set_motor_angles(motor_angles)

    def _update_orientation(self) -> None:
        [roll, pitch, yaw] = [value for _, value in self.input_dict.items()]
        self.body_kinematics.update_body_pose(roll, pitch, yaw)
        ee_points = self.body_kinematics.get_ee_points()
        for leg_name, ee_point in zip(self.config.legs, ee_points):
            self.leg_group_dict[leg_name].set_ee_point(ee_point)

    def _update_ee_offset(self) -> None:
        ee_points = self.locomotion.get_ee_points(self.sim_time)
        for leg_name, ee_point in ee_points.items():
            offset = self.input_dict[leg_name]
            self.leg_group_dict[leg_name].set_ee_point(ee_point+offset)

# ------------------------------- PPO Training ------------------------------- #
    def get_observation(self) -> Dict:
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
        for leg_name in self.config.legs:
            motor_angles = self.leg_group_dict[leg_name].get_motor_angles()
            joints.extend(motor_angles)
        joints = np.array(joints)

        # Get phase
        phase = self.locomotion.get_current_phase(self.sim_time)
        phase_list = []
        for leg in self.config.legs:
            phase_list.append(np.sin(phase[leg]))
            phase_list.append(np.cos(phase[leg]))
        phase_list = np.array(phase_list)

        obs = np.concatenate([euler_ori, linear_vel, angular_vel, joints, phase_list])
        return obs
    
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
    