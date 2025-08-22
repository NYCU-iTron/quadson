from math import *
import numpy as np

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

    def __init__(self, DEBUG=False):
        self.Ls = [8.16, 8.0, 10.0, 8.0, 13.0, 8.0]
        self._motor_angles = None
        self._ee_point = None
        self._points = None
        self._angles = None
        self._ee_velocity = None
        self._motor_omegas = None
        self.unsafe_reasons = []
        self.DEBUG = DEBUG

    def get_motor_angles(self) -> list[float]:
        """
        Return current motor angles.

        :return _motor_angles: [angle0, angle1, angle5], rad
        """
        if self._motor_angles is None:
            raise Exception("[Error] _motor_angles is None")
        return self._motor_angles
    
    def set_motor_angles(self, motor_angles: list[float]) -> None:
        """
        Set new motor angles, and update end effector position.
        
        :param motor_angles: [angle0, angle1, angle5], rad
        """
        self._ee_point = self.calc_ang2pnt(motor_angles)
        if self.unsafe_reasons:
            if self.DEBUG:
                print(f"[WARN] Unsafe conditions detected: {', '.join(self.unsafe_reasons)}")
            self.unsafe_reasons.clear()
        else:
            self._motor_angles = motor_angles

    def get_ee_point(self) -> list[float]:
        """
        Return current end effector position.
        
        :return _ee_point: [x, y, z], cm
        """
        if self._ee_point is None:
            raise Exception("[Error] _ee_point is None")
        return self._ee_point
    
    def set_ee_point(self, ee_point: list[float]) -> None:
        """
        Set new end effector position, and update motor angles.
        
        :param ee_point: [x, y, z], cm
        """
        self._motor_angles = self.calc_pnt2ang(ee_point)
        if self.unsafe_reasons:
            if self.DEBUG:
                print(f"[WARN] Unsafe conditions detected: {', '.join(self.unsafe_reasons)}")
            self.unsafe_reasons.clear()
        else:
            self._ee_point = ee_point

    def get_points(self) -> list[float]:
        """
        Return current position of the joints.

        :return _points: shape (6, 3), cm
        """
        if self._points is None:
            raise Exception("[Error] _points is None")
        return self._points

    def get_angles(self) -> list[float]:
        """
        Return current angle of the joints.

        :return _angles: shape (6, ), rad
        """
        if self._angles is None:
            raise Exception("[Error] _angles is None")
        return self._angles

    def get_ee_velocity(self) -> list[float]:
        """
        Return current linear velocity of the end effector.

        :return _velocity: [vx, vy, vz], cm/s
        """
        if self._ee_velocity is None:
            raise Exception("[Error] _velocity is None")
        return self._ee_velocity
    
    def set_ee_velocity(self, ee_velocity: list[float]) -> None:
        """
        Set new linear velocity of the end effector, and update angular velocity of the motors.
        
        :param ee_velocity: [vx, vy, vz], cm/s
        """
        self._motor_omegas = self.calc_vel2omg(ee_velocity)
        if self.unsafe_reasons:
            if self.DEBUG:
                print(f"[WARN] Unsafe conditions detected: {', '.join(self.unsafe_reasons)}")
            self.unsafe_reasons.clear()
        else:
            self._ee_velocity = ee_velocity

    def get_motor_omegas(self) -> list[float]:
        """
        Return the current angular velocity of the motors
        
        :return _motor_omegas: [omega0, omega1, omega5], rad/s
        """
        if self._motor_omegas is None:
            raise Exception("[Error] _omegas is None")
        return self._motor_omegas
    
    def set_motor_omegas(self, motor_omegas: list[float]) -> None:
        """
        Set new angular velocity of the motors, and update linear velocity of the end effector.
        
        :param motor_omegas: [omega0, omega1, omega5], rad/s
        """
        self._ee_velocity = self.calc_omg2vel(motor_omegas)
        if self.unsafe_reasons:
            if self.DEBUG:
                print(f"[WARN] Unsafe conditions detected: {', '.join(self.unsafe_reasons)}")
            self.unsafe_reasons.clear()
        else:
            self._motor_omegas = motor_omegas

    def calc_ang2pnt(self, motor_angles: list[float]) -> list[float]:
        '''
        Given motor angles, calculate end effector position and update states of the leg
        
        :param motor_angles: [angle0, angle1, angle5], angle of the motors, rad
        :return: [x, y, z], end effector position, cm
        '''
        angle0, angle1, angle5 = motor_angles

        # Safety check
        if angle0 > np.pi/2 or angle0 < -np.pi/2:
            self.unsafe_reasons.append("angle0 out of range")
            angle0 = np.clip(angle0, -np.pi/2, np.pi/2)
        if angle1 > 1.2 * np.pi or angle1 < 0.25 * np.pi:
            self.unsafe_reasons.append("angle1 out of range")
            angle1 = np.clip(angle1, 0.25 * np.pi, 1.2 * np.pi)
        if angle5 > 0.75 * np.pi or angle5 < 0:
            self.unsafe_reasons.append("angle5 out of range")
            angle5 = np.clip(angle5, 0, 0.75 * np.pi)

        # Safety check, prevent excessive toe-in
        if angle1 - angle5 < -0.06:
            self.unsafe_reasons.append("Excessive toe-in")
            return self._ee_point

        # Start from 2D kinematics
        p1 = np.array([0, 0])
        p2 = np.array([self.Ls[1]*cos(angle1), -self.Ls[1]*sin(angle1)])
        p5 = np.array([self.Ls[0], 0])
        p4 = np.array([self.Ls[5]*cos(angle5), -self.Ls[5]*sin(angle5)]) + p5
        
        L14 = np.linalg.norm(p4-p1)
        L24 = np.linalg.norm(p4-p2)

        # Safety check
        if L24 < 5:
            self.unsafe_reasons.append("P2, P4 too close")
            return self._ee_point
        elif(L24 >= self.Ls[4] + self.Ls[2]):
            self.unsafe_reasons.append("Leg is not reachable")
            return self._ee_point
        elif (L24 == self.Ls[4] + self.Ls[2]):
            self.unsafe_reasons.append("Leg reaches singularity")
            return self._ee_point

        # Calculate p3 from angle2
        angle_324 = self.safe_acos((self.Ls[2]**2 + L24**2 - self.Ls[4]**2) / (2 * self.Ls[2] * L24))
        angle_421 = self.safe_acos((self.Ls[1]**2 + L24**2 - L14**2) / (2 * self.Ls[1] * L24))
        angle2 = angle_324 + angle_421 - (np.pi - angle1)
        p3 = p2 + self.Ls[2] * np.array([cos(angle2), -sin(angle2)])

        # Calculate p3 from angle4
        angle_342 = self.safe_acos((self.Ls[4]**2 + L24**2 - self.Ls[2]**2) / (2 * self.Ls[4] * L24))
        L25 = np.linalg.norm(p2-p5)
        angle_245 = self.safe_acos((L24**2 + self.Ls[5]**2 - L25**2) / (2 * L24 * self.Ls[5]))
        angle4 = np.pi + angle5 - (angle_342 + angle_245)
        p3_alt = p4 + self.Ls[4] * np.array([cos(angle4), -sin(angle4)])

        # Safety check
        if not np.allclose(p3, p3_alt, atol=1e-4):
            self.unsafe_reasons.append("Open Chain P3 not matching")
            return self._ee_point

        # Calculate pe and p3 by averaging the positions
        pe = np.array([(self.Ls[2]+self.Ls[3])*cos(angle2), -(self.Ls[2]+self.Ls[3])*sin(angle2)]) + p2

        # Translate points from 2D to 3D, rotate the current plane to the x-y plane
        transformation_matrix = np.array([
            [1, 0, 0],
            [0, cos(-angle0), -sin(-angle0)],
            [0, sin(-angle0), cos(-angle0)]
        ])
        points = np.array([p1, p2, p3, pe, p4, p5])
        points = np.concatenate([points, np.zeros((points.shape[0], 1))], axis=1)
        points = points @ transformation_matrix

        # Update current joint angles and points
        self._points = points
        self._angles = np.array([angle0, angle1, angle2, None, angle4, angle5])

        ee_point = points[3]
        return ee_point
    
    def calc_pnt2ang(self, ee_point: list[float]) -> list[float]:
        '''
        Given end effector position, calculate motor angles and update states of the leg.
        
        :param ee_point: [x, y, z], cm
        :return motor_angles: [angle0, angle1, angle5], rad
        '''
        x, y, z = ee_point

        # Calculate the angle of the motor 0
        angle0 = -atan2(z, -y)

        # Translate the points from 3D to 2D
        offset_angle = -angle0
        transformation_matrix = np.array([
            [1, 0, 0],
            [0, cos(offset_angle), -sin(offset_angle)],
            [0, sin(offset_angle), cos(offset_angle)]
        ])
        [x, y, z] = transformation_matrix @ np.array([x, y, z])
        pe_2d = np.array([x, y])

        # Angle 1
        L1e = np.linalg.norm(pe_2d)
        angle_e15 = atan2(-y, x)
        if angle_e15 < 0:
            angle_e15 += np.pi
        angle_e12 = self.safe_acos((self.Ls[1]**2 + L1e**2 - (self.Ls[2] + self.Ls[3])**2) / (2 * self.Ls[1] * L1e))
        angle1 = angle_e15 + angle_e12

        # Points based on angle 1
        p1 = np.array([0, 0])  
        p2 = np.array([self.Ls[1]*cos(angle1), -self.Ls[1]*sin(angle1)])
        p3 = (pe_2d * self.Ls[2] + p2 * self.Ls[3]) / (self.Ls[2] + self.Ls[3])
        p5 = np.array([self.Ls[0], 0])

        # Angle 5
        L35 = np.linalg.norm(p3 - p5)
        angle_350 = atan2(-p3[1], self.Ls[0] - p3[0])
        if angle_350 < 0:
            angle_350 += np.pi
        angle_354 = self.safe_acos((L35**2 + self.Ls[5]**2 - self.Ls[4]**2) / (2 * L35 * self.Ls[5]))
        angle5 = np.pi - (angle_350 + angle_354)

        # Point 4
        p4 = np.array([self.Ls[5]*cos(angle5), -self.Ls[5]*sin(angle5)]) + p5

        # Store the states
        angle2 = self.safe_acos((pe_2d-p2)[0] / self.safe_norm(pe_2d-p2))
        angle4 = self.safe_acos((p3-p4)[0] / self.safe_norm(p3-p4))
        if p3[1] > p4[1]:
            angle4 = 2*np.pi - angle4
        angles = np.array([angle0, angle1, angle2, None, angle4, angle5])

        # Translate the points from 2D to 3D
        points = np.array([p1, p2, p3, pe_2d, p4, p5])
        points = np.concatenate([points, np.zeros((points.shape[0], 1))], axis=1)

        transformation_matrix = np.array([
            [1, 0, 0],
            [0, cos(-angle0), -sin(-angle0)],
            [0, sin(-angle0), cos(-angle0)]
        ])
        points = points @ transformation_matrix

        self._points = points
        self._angles = angles

        # Safety Check
        if angle0 > np.pi/2 or angle0 < -np.pi/2:
            self.unsafe_reasons.append("angle0 out of range")
        if angle1 > 1.2 * np.pi or angle1 < 0.25 * np.pi:
            self.unsafe_reasons.append("angle1 out of range")
        if angle5 > 0.75 * np.pi or angle5 < 0:
            self.unsafe_reasons.append("angle5 out of range")

        motor_angles = [angle0, angle1, angle5]
        return motor_angles
    
    def calc_omg2vel(self, motor_omegas: list[float]) -> list:
        '''
        Given angular velocity of the motors, calculate linear velocity of the end effector.

        :param motor_omegas: [omega0, omega1, omega5], rad/s
        :return ee_velocity: [vx, vy, vz], cm/s
        '''
        J = self.numerical_jacobian()
        ee_velocity = J @ motor_omegas
        return ee_velocity
    
    def calc_vel2omg(self, ee_velocity: list[float]) -> list:
        '''
        Given linear velocity of the end effector, calculate angular velocity of the motors.

        :param ee_velocity: [vx, vy, vz], cm/s
        :return motor_omegas: [omega0, omega1, omega5], rad/s
        '''
        J = self.numerical_jacobian()
        J_inv = np.linalg.pinv(J)
        motor_omegas = J_inv @ np.array(ee_velocity)
        return motor_omegas
    
    def numerical_jacobian(self, delta=1e-4) -> list[float]:
        """
        Compute jacobian matrix based on current motor angles and position of the end effector.

        :param delta: the perturbation step size, float
        :return J: shape (3, 3)
        """
        motor_angles = self._motor_angles
        ee_point = self._ee_point
        J = np.zeros((3, 3))
        for i in range(3):
            angles_perturbed = motor_angles.copy()
            angles_perturbed[i] += delta
            pe_perturbed = self.calc_ang2pnt(angles_perturbed)
            J[0:3, i] = (pe_perturbed - ee_point) / delta
        return J
    
    def safe_acos(self, x) -> float:
        """
        Perform safety arccos

        :param x: input value
        :return: arccos value
        """
        if x < -1.0 or x > 1.0:
            self.unsafe_reasons.append("acos input out of range")
        return acos(np.clip(x, -1.0, 1.0))

    def safe_norm(self, vec, eps=1e-6) -> float:
        """
        Perform safety norm

        :param vec: input vector
        :param eps: epsilon value
        :return: norm value
        """
        if np.linalg.norm(vec) < eps:
            self.unsafe_reasons.append("norm input too small")
        return np.linalg.norm(vec) + eps
    