import numpy as np
import logging
from real.motor_manager import MotorManager
from common.leg_kinematics import LegKinematics

class Leg:
    def __init__(self, leg_id: int, motor_manager: MotorManager):
        self.logger = logging.getLogger(__name__)

        self.motor_manager = motor_manager

        # Leg group index ranges from 0 to 3.
        # Motor index in single leg ranges from 0 to 2.
        # Motor index in total ranges from 0 to 11.
        self.leg_id = leg_id
        self.motor_indices = [
            3 * leg_id + 0,  # Motor 0
            3 * leg_id + 1,  # Motor 1
            3 * leg_id + 2   # Motor 2
        ]

        self.leg_kinematics = LegKinematics()
        self.leg_kinematics.set_motor_angles([0, np.pi, np.pi/2]) # initial motor angles

    def get_zero_states(self) -> list[bool]:
        zero_states = []
        for motor_id in self.motor_indices:
            zero_state = self.motor_manager.get_zero_state(motor_id)
            zero_states.append(zero_state)
        
        return zero_states
    
    def get_motor_angles(self) -> list[float]:
        motor_angles = []
        for motor_id in self.motor_indices:
            motor_angle = self.motor_manager.get_motor_angle(motor_id)
            motor_angles.append(motor_angle)

        return motor_angles
    
    def get_ee_point(self) -> list[float]:
        return self.leg_kinematics.get_ee_point()
    
    def enable_torque(self, enable: bool) -> None:
        for motor_id in self.motor_indices:
            self.motor_manager.enable_motor_torque(motor_id, enable)

    def set_control_mode(self, mode: int) -> None:
        for motor_id in self.motor_indices:
            self.motor_manager.set_control_mode(motor_id, mode)
            
    def set_motor_angles(self, motor_angles: list[float]) -> None:
        for motor_id, motor_angle in zip(self.motor_indices, motor_angles):
            if motor_angle is None:
                continue
            self.motor_manager.set_motor_angle(motor_id, motor_angle)

    def set_motor_omegas(self, motor_omegas: list[float]) -> None:
        for motor_id, motor_omega in zip(self.motor_indices, motor_omegas):
            if motor_omega is None:
                continue
            self.motor_manager.set_motor_omega(motor_id, motor_omega)

    def set_ee_point(self, ee_point: list[float]) -> None:
        self.leg_kinematics.set_ee_point(ee_point)
        model_angles = self.leg_kinematics.get_motor_angles()

        motor_angles = self.convert_model2motor_angles(model_angles)

        for motor_id, motor_angle in zip(self.motor_indices, motor_angles):
            self.motor_manager.set_motor_omega(motor_id, motor_angle)

    def convert_model2motor_angles(self, model_angles: list[float]) -> list[float]:
        mech_angles = model_angles
        return mech_angles
    
    def stop_leg(self) -> None:
        for motor_id in self.motor_indices:
            self.motor_manager.stop_motor(motor_id)
