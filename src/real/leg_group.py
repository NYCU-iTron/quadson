import numpy as np
from real.motor_manager import MotorManager
from common.leg_kinematics import LegKinematics

class LegGroup:
    def __init__(self, leg_index: int, motor_manager: MotorManager):
        self.motor_manager = motor_manager
        self.leg_index = leg_index
        self.leg_kinematics = LegKinematics()
        self.leg_kinematics.set_motor_angles([0, np.pi, np.pi/2]) # initial motor angles
  
    def enable_torque(self, enable: bool) -> None:
        for index in range(3):
            motor_index = 3 * self.leg_index + index + 1
            self.motor_manager.enable_motor_torque(motor_index, enable)

    def set_motor_angles(self, motor_angles: list[float]) -> None:
        self.leg_kinematics.set_motor_angles(motor_angles)
        model_angles = motor_angles

        # Convert model angles to mechanical angles and set them
        mech_angles = self.convert_model2mech_angles(model_angles)
        self.set_mech_angles(mech_angles)

    def set_ee_point(self, ee_point: list[float]) -> None:
        self.leg_kinematics.set_ee_point(ee_point)
        model_angles = self.leg_kinematics.get_motor_angles()

        # Convert model angles to mechanical angles and set them
        mech_angles = self.convert_model2mech_angles(model_angles)
        self.set_mech_angles(mech_angles)
  
    def convert_model2mech_angles(self, model_angles: list[float]) -> list[float]:
        mech_angles = model_angles
        return mech_angles
  
    def set_mech_angles(self, mech_angles: list[float]) -> None:
        # Set the angles for each motor in the leg group
        # Leg group index ranges from 0 to 3.
        # Index of mech_angles ranges from 0 to 2.
        # Motor index ranges from 1 to 12.
        for index, angle in enumerate(mech_angles):
            motor_index = 3 * self.leg_index + index + 1
            self.motor_manager.set_motor_angle(motor_index, angle)

    def get_motor_angles(self) -> list[float]:
        return self.leg_kinematics.get_motor_angles()

    def get_ee_point(self) -> list[float]:
        return self.leg_kinematics.get_ee_point()