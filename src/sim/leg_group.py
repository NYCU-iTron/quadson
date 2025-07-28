import numpy as np
from common.leg_kinematics import LegKinematics
from sim.motor import Motor

class LegGroup:
    def __init__(self, robot_id, joint_indices):
        self.robot_id = robot_id
        self.motors = [Motor(robot_id, joint_index) for joint_index in joint_indices]
        self.leg_kinematics = LegKinematics()
        self.leg_kinematics.set_motor_angles([0, np.pi, np.pi/2]) # initial motor angles

    def set_motor_angles(self, motor_angles):
        self.leg_kinematics.set_motor_angles(motor_angles)
        theory_angles = self.leg_kinematics.get_angles()
        env_angles = self.get_env_angles(theory_angles)
        for motor, motor_angle in zip(self.motors, env_angles):
            motor.set_angle(motor_angle)

    def get_env_angles(self, theory_angles):
        # Active motor
        j0_env = theory_angles[0]
        j1_env = np.pi - theory_angles[1] # pi: init theory angle of joint 1 in env
        j5_env = np.pi/2 - theory_angles[5] # pi/2: init theory angle of joint 5 in env

        # Passive joints, to enforce closure
        j2_env = 1.2406 - (np.pi + theory_angles[2] - theory_angles[1]) # 1.2406: init theory angle of joint 2 in env
        j4_env =  - 1.6833 + (np.pi + theory_angles[5] - theory_angles[4]) # 1.6833: init theory angle of joint 4 in env

        return [j0_env, j1_env, j2_env, j4_env, j5_env]

    def set_ee_point(self, ee_point):
        self.leg_kinematics.set_ee_point(ee_point)
        motor_angles = self.leg_kinematics.get_motor_angles()
        self.set_motor_angles(motor_angles)

    def get_ee_point(self):
        return self.leg_kinematics.get_ee_point()
    
    def get_motor_angles(self):
        return self.leg_kinematics.get_motor_angles()
    