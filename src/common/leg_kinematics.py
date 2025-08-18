import numpy as np
import logging
from common.config import LegState

class LegKinematics:
    """
    Represents a single leg of the quadruped robot. The leg is a five-bar linkage mechanism with three motors.
    
    Attributes
    ----------
    Ls            : [L0, L1, L2, L3, L4, L5], length of the links, cm
    _motor_angles : [angle0, angle1, angle5], angle of the motors, rad
    _ee_point     : [x, y, z], position of the end effector, cm
    _points       : shape (6, 3), position of the joints, cm
    _angles       : shape (6,), angle of the joints, rad
    _ee_velocity  : [vx, vy, vz], linear velocity of the end effector, cm/s
    _motor_omegas : [omega0, omega1, omega5], angular velocity of the motors, rad/s

    Methods
    -------
    get_motor_angles   : Return current motor angles.
    set_motor_angles   : Set new motor angles, and update end effector position.
    get_ee_point       : Return current end effector position.
    set_ee_point       : Set new end effector position, and update motor angles.
    get_points         : Return current position of the joints.
    get_angles         : Return current angle of the joints.
    get_ee_velocity    : Return current linear velocity of the end effector.
    set_ee_velocity    : Set new linear velocity of the end effector, and update angular velocity of the motors.
    get_motor_omegas   : Return current angular velocity of the motors.
    set_motor_omegas   : Set new angular velocity of the motors, and update linear velocity of the end effector.
    calc_ang2pnt       : Forward kinematics (motor angles -> end effector position)
    calc_pnt2ang       : Inverse kinematics (end effector position -> motor angles)
    calc_omg2vel       : Forward differential kinematics (motor omegas -> end effector velocity)
    calc_vel2omg       : Inverse differential kinematics (end effector velocity -> motor omegas)
    numerical_jacobian : Calculate numerical Jacobian matrix for differential kinematics.
    """

    def __init__(self):
        self.logger = logging.getLogger(__name__)

        self.Ls = [8.16, 8.0, 10.0, 8.0, 13.0, 8.0]
        self.leg_state = LegState()
    
    def get_leg_state(self) -> LegState:
        return self.leg_state

    def get_motor_angles(self) -> list[float]:
        """
        Return current motor angles.

        :return _motor_angles: [angle0, angle1, angle5], rad
        """
        if self.leg_state.motor_angles is None:
            self.logger.warning("Motor angles are not set")
        return self.leg_state.motor_angles
    
    def get_motor_omegas(self) -> list[float]:
        """
        Return the current angular velocity of the motors
        
        :return _motor_omegas: [omega0, omega1, omega5], rad/s
        """
        if self.leg_state.motor_omegas is None:
            self.logger.warning("Motor omegas are not set")
        return self.leg_state.motor_omegas
    
    def get_joint_points(self) -> list[float]:
        """
        Return current position of the joints.

        :return _points: shape (6, 3), cm
        """
        if self.leg_state.joint_points is None:
            self.logger.warning("Joint points are not set")
        return self.leg_state.joint_points
    
    def get_joint_angles(self) -> list[float]:
        """
        Return current angle of the joints.

        :return _angles: shape (6, ), rad
        """
        if self.leg_state.joint_angles is None:
            self.logger.warning("Joint angles are not set")
        return self.leg_state.joint_angles
    
    def get_ee_point(self) -> list[float]:
        """
        Return current end effector position.
        
        :return _ee_point: [x, y, z], cm
        """
        if self.leg_state.ee_point is None:
            self.logger.warning("End effector point is not set")
        return self.leg_state.ee_point
    
    def get_ee_velocity(self) -> list[float]:
        """
        Return current linear velocity of the end effector.

        :return _velocity: [vx, vy, vz], cm/s
        """
        if self.leg_state.ee_velocity is None:
            self.logger.warning("End effector velocity is not set")
        return self.leg_state.ee_velocity
    
    def set_motor_angles(self, motor_angles: list[float]) -> None:
        """
        Set new motor angles, and update end effector position.
        
        :param motor_angles: [angle0, angle1, angle5], rad
        """
        leg_state = self.calc_forward_kinematics(motor_angles)
        self.leg_state = leg_state

    def set_ee_point(self, ee_point: list[float]) -> None:
        """
        Set new end effector position, and update motor angles.
        
        :param ee_point: [x, y, z], cm
        """
        leg_state = self.calc_backward_kinematics(ee_point)
        self.leg_state = leg_state

    def set_motor_omegas(self, motor_angles: list[float], motor_omegas: list[float]) -> None:
        """
        Set new angular velocity of the motors, and update linear velocity of the end effector.
        
        :param motor_omegas: [omega0, omega1, omega5], rad/s
        """
        leg_state = self.calc_forward_kinematics_diff(motor_angles, motor_omegas)
        self.leg_state = leg_state

    def set_ee_velocity(self, ee_point: list[float], ee_velocity: list[float]) -> None:
        """
        Set new linear velocity of the end effector, and update angular velocity of the motors.
        
        :param ee_velocity: [vx, vy, vz], cm/s
        """
        leg_state = self.calc_backward_kinematics_diff(ee_point, ee_velocity)
        self.leg_state = leg_state
        
    def calc_forward_kinematics(self, motor_angles: list[float]) -> LegState:
        '''
        Given motor angles, calculate end effector position
        
        :param motor_angles: [angle0, angle1, angle5], angle of the motors, rad
        :return: [x, y, z], end effector position, cm
        '''
        motor_angles = np.array(motor_angles)
        angle0, angle1, angle5 = motor_angles

        # Safety check
        if angle0 > np.pi / 2 or angle0 < -np.pi / 2:
            self.logger.warning("angle0 out of range")
            angle0 = np.clip(angle0, -np.pi / 2, np.pi / 2)
        if angle1 > 1.2 * np.pi or angle1 < 0.25 * np.pi:
            self.logger.warning("angle1 out of range")
            angle1 = np.clip(angle1, 0.25 * np.pi, 1.2 * np.pi)
        if angle5 > 0.75 * np.pi or angle5 < 0:
            self.logger.warning("angle5 out of range")
            angle5 = np.clip(angle5, 0, 0.75 * np.pi)

        # Safety check, prevent excessive toe-in
        if angle5 - angle1 > 0.06:
            self.logger.warning("Excessive toe-in")
            angle5 = angle1 + 0.06
            
        # Start from 2D kinematics
        p1 = np.array([0, 0])
        p2 = np.array([self.Ls[1] * np.cos(angle1), -self.Ls[1] * np.sin(angle1)])
        p5 = np.array([self.Ls[0], 0])
        p4 = np.array([self.Ls[5] * np.cos(angle5), -self.Ls[5] * np.sin(angle5)]) + p5
        
        L14 = np.linalg.norm(p4-p1)
        L24 = np.linalg.norm(p4-p2)

        # Safety check
        if L24 < 5:
            self.logger.warning("p2, p4 too close")
        elif (L24 == self.Ls[4] + self.Ls[2]):
            self.logger.warning("Leg reaches singularity")
        elif(L24 >= self.Ls[4] + self.Ls[2]):
            self.logger.error("Leg is not reachable, L24 > Ls[4] + Ls[2]")

        # Calculate p3 from angle2
        angle_324 = self.safe_acos((self.Ls[2]**2 + L24**2 - self.Ls[4]**2) / (2 * self.Ls[2] * L24))
        angle_421 = self.safe_acos((self.Ls[1]**2 + L24**2 - L14**2) / (2 * self.Ls[1] * L24))
        angle2 = angle_324 + angle_421 - (np.pi - angle1)
        p3 = p2 + self.Ls[2] * np.array([np.cos(angle2), -np.sin(angle2)])

        # Calculate p3 from angle4
        angle_342 = self.safe_acos((self.Ls[4]**2 + L24**2 - self.Ls[2]**2) / (2 * self.Ls[4] * L24))
        L25 = np.linalg.norm(p2-p5)
        angle_245 = self.safe_acos((L24**2 + self.Ls[5]**2 - L25**2) / (2 * L24 * self.Ls[5]))
        angle4 = np.pi + angle5 - (angle_342 + angle_245)
        p3_alt = p4 + self.Ls[4] * np.array([np.cos(angle4), -np.sin(angle4)])

        # Safety check
        if not np.allclose(p3, p3_alt, atol=1e-4):
            self.logger.warning("Open Chain P3 not matching")

        # Calculate pe and p3 by averaging the positions
        pe = np.array([(self.Ls[2] + self.Ls[3]) * np.cos(angle2), -(self.Ls[2] + self.Ls[3]) * np.sin(angle2)]) + p2

        # Translate points from 2D to 3D
        points = np.array([p1, p2, p3, pe, p4, p5])
        points = np.concatenate([points, np.zeros((points.shape[0], 1))], axis=1)
        
        # Rotate the current plane to the x-y plane
        transformation_matrix = np.array([
            [1, 0, 0],
            [0, np.cos(-angle0), -np.sin(-angle0)],
            [0, np.sin(-angle0), np.cos(-angle0)]
        ])
        joint_points = points @ transformation_matrix
        
        ee_point = joint_points[3]
        joint_angles = np.array([angle0, angle1, angle2, np.nan, angle4, angle5])

        leg_state = LegState(
            motor_angles = motor_angles,
            ee_point = ee_point,
            joint_points = joint_points,
            joint_angles = joint_angles,
        )

        return leg_state
    
    def calc_backward_kinematics(self, ee_point: list[float]) -> LegState:
        '''
        Given end effector position, calculate motor angles and update states of the leg.
        
        :param ee_point: [x, y, z], cm
        :return motor_angles: [angle0, angle1, angle5], rad
        '''
        ee_point = np.array(ee_point)
        x, y, z = ee_point

        # Calculate the angle of the motor 0
        angle0 = -np.atan2(z, -y)

        # Translate the points from 3D to 2D
        offset_angle = -angle0
        transformation_matrix = np.array([
            [1, 0, 0],
            [0, np.cos(offset_angle), -np.sin(offset_angle)],
            [0, np.sin(offset_angle), np.cos(offset_angle)]
        ])
        [x, y, z] = transformation_matrix @ np.array([x, y, z])
        pe_2d = np.array([x, y])

        # Angle 1
        L1e = np.linalg.norm(pe_2d)
        angle_e15 = np.atan2(-y, x)
        if angle_e15 < 0:
            angle_e15 += np.pi
        angle_e12 = self.safe_acos((self.Ls[1]**2 + L1e**2 - (self.Ls[2] + self.Ls[3])**2) / (2 * self.Ls[1] * L1e))
        angle1 = angle_e15 + angle_e12

        # Points based on angle 1
        p1 = np.array([0, 0])  
        p2 = np.array([self.Ls[1] * np.cos(angle1), -self.Ls[1] * np.sin(angle1)])
        p3 = (pe_2d * self.Ls[2] + p2 * self.Ls[3]) / (self.Ls[2] + self.Ls[3])
        p5 = np.array([self.Ls[0], 0])

        # Angle 5
        L35 = np.linalg.norm(p3 - p5)
        angle_350 = np.atan2(-p3[1], self.Ls[0] - p3[0])
        if angle_350 < 0:
            angle_350 += np.pi
        angle_354 = self.safe_acos((L35**2 + self.Ls[5]**2 - self.Ls[4]**2) / (2 * L35 * self.Ls[5]))
        angle5 = np.pi - (angle_350 + angle_354)

        # Point 4
        p4 = np.array([self.Ls[5] * np.cos(angle5), -self.Ls[5] * np.sin(angle5)]) + p5

        # Store the states
        angle2 = self.safe_acos((pe_2d-p2)[0] / self.safe_norm(pe_2d-p2))
        angle4 = self.safe_acos((p3-p4)[0] / self.safe_norm(p3-p4))
        if p3[1] > p4[1]:
            angle4 = 2*np.pi - angle4
        joint_angles = np.array([angle0, angle1, angle2, np.nan, angle4, angle5])

        # Translate the points from 2D to 3D
        joint_points = np.array([p1, p2, p3, pe_2d, p4, p5])
        joint_points = np.concatenate([joint_points, np.zeros((joint_points.shape[0], 1))], axis=1)

        transformation_matrix = np.array([
            [1, 0, 0],
            [0, np.cos(-angle0), -np.sin(-angle0)],
            [0, np.sin(-angle0), np.cos(-angle0)]
        ])
        joint_points = joint_points @ transformation_matrix

        # Safety Check
        if angle0 > np.pi/2 or angle0 < -np.pi/2:
            self.logger.warning("angle0 out of range")
        if angle1 > 1.2 * np.pi or angle1 < 0.25 * np.pi:
            self.logger.warning("angle1 out of range")
        if angle5 > 0.75 * np.pi or angle5 < 0:
            self.logger.warning("angle5 out of range")

        motor_angles = np.array([angle0, angle1, angle5])

        leg_state = LegState(
            motor_angles=motor_angles,
            ee_point=ee_point,
            joint_points=joint_points,
            joint_angles=joint_angles,
        )
        return leg_state
    
    def calc_forward_kinematics_diff(self, motor_angles: list[float], motor_omegas: list[float]) -> LegState:
        '''
        Given angular velocity of the motors, calculate linear velocity of the end effector.

        :param motor_omegas: [omega0, omega1, omega5], rad/s
        :return ee_velocity: [vx, vy, vz], cm/s
        '''
        leg_state = self.calc_forward_kinematics(motor_angles)
        J = self.calc_numerical_jacobian(leg_state.motor_angles, leg_state.ee_point)
        ee_velocity = J @ motor_omegas

        leg_state.ee_velocity = ee_velocity
        leg_state.motor_omegas = motor_omegas
        
        return leg_state
    
    def calc_backward_kinematics_diff(self, ee_point: list[float], ee_velocity: list[float]) -> LegState:
        '''
        Given linear velocity of the end effector, calculate angular velocity of the motors.

        :param ee_velocity: [vx, vy, vz], cm/s
        :return motor_omegas: [omega0, omega1, omega5], rad/s
        '''
        leg_state = self.calc_backward_kinematics(ee_point)
        J = self.calc_numerical_jacobian(leg_state.motor_angles, leg_state.ee_point)
        J_inv = np.linalg.pinv(J)
        motor_omegas = J_inv @ np.array(ee_velocity)
        
        leg_state.ee_velocity = ee_velocity
        leg_state.motor_omegas = motor_omegas

        return leg_state
    
    def calc_numerical_jacobian(self, motor_angles: list[float], ee_point: list[float], delta=1e-4) -> list[float]:
        """
        Compute jacobian matrix based on current motor angles and position of the end effector.

        :param delta: the perturbation step size, float
        :return J: shape (3, 3)
        """
        J = np.zeros((3, 3))
        for i in range(3):
            angles_perturbed = motor_angles.copy()
            angles_perturbed[i] += delta
            leg_state_perturbed = self.calc_forward_kinematics(angles_perturbed)
            pe_perturbed = leg_state_perturbed.ee_point
            J[0:3, i] = (pe_perturbed - ee_point) / delta
        return J
    
    def safe_acos(self, x) -> float:
        """
        Perform safety arccos

        :param x: input value
        :return: arccos value
        """
        if x < -1.0 or x > 1.0:
            self.logger.warning("acos input out of range")
        return np.acos(np.clip(x, -1.0, 1.0))

    def safe_norm(self, vec, eps=1e-6) -> float:
        """
        Perform safety norm

        :param vec: input vector
        :param eps: epsilon value
        :return: norm value
        """
        if np.linalg.norm(vec) < eps:
            self.logger.warning("norm input too small")
        return np.linalg.norm(vec) + eps
    