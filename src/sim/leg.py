import numpy as np
import logging
from common.config import LegName
from common.leg_kinematics import LegKinematics, LegState
from sim.motor_manager import MotorManager

class Leg:
    def __init__(self, name: LegName, motor_manager: MotorManager):
        self.logger = logging.getLogger(__name__)

        self.motor_manager = motor_manager
        self.name = name
        self.motor_indices = [
            5 * name.value + 0,  # Motor 0
            5 * name.value + 1,  # Motor 1
            5 * name.value + 2,  # Motor 2
            5 * name.value + 4,  # Motor 4
            5 * name.value + 3,  # Motor 5
        ]

        self.leg_kinematics = LegKinematics()
        self.leg_kinematics.set_motor_angles([0, np.pi, np.pi/2]) # initial motor angles

    def get_leg_state(self) -> LegState:
        return self.leg_kinematics.get_leg_state()
     
    def get_motor_angles(self)-> list[float]:
        return self.leg_kinematics.get_motor_angles()
    
    def get_ee_point(self) -> list[float]:
        return self.leg_kinematics.get_ee_point()

    def set_motor_angles(self, motor_angles: list[float]) -> None:
        self.leg_kinematics.set_motor_angles(motor_angles)
        theory_angles = self.leg_kinematics.get_joint_angles()
        env_angles = self.calc_env_angles(theory_angles)
        for motor_id, motor_angle in zip(self.motor_indices, env_angles):
            self.motor_manager.set_motor_angle(motor_id, motor_angle)
    
    def set_ee_point(self, ee_point: list[float]) -> None:
        self.leg_kinematics.set_ee_point(ee_point)
        theory_angles = self.leg_kinematics.get_joint_angles()
        env_angles = self.calc_env_angles(theory_angles)
        for motor_id, motor_angle in zip(self.motor_indices, env_angles):
            self.motor_manager.set_motor_angle(motor_id, motor_angle)

    def calc_env_angles(self, theory_angles: list[float]) -> list[float]:
        # Active motor
        j0_env = theory_angles[0]
        j1_env = np.pi - theory_angles[1] # pi: init theory angle of joint 1 in env
        j5_env = np.pi/2 - theory_angles[5] # pi/2: init theory angle of joint 5 in env

        # Passive joints, to enforce closure
        j2_env = 1.2406 - (np.pi + theory_angles[2] - theory_angles[1]) # 1.2406: init theory angle of joint 2 in env
        j4_env =  - 1.6833 + (np.pi + theory_angles[5] - theory_angles[4]) # 1.6833: init theory angle of joint 4 in env

        return [j0_env, j1_env, j2_env, j4_env, j5_env]
    