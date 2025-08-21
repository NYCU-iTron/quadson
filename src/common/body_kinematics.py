import numpy as np
from common.config import LegName, OrientationType

class BodyKinematics:
    def __init__(self):
        # Body dimensions, shoulder on the vertices
        body_width = 22.4
        body_length = 30.6
        self.body_z = 17.028

        # Foothold coordinates in world frame
        init_toe_x = -2.1634

        self.foothold_dict = {
            LegName.FRONT_LEFT: np.array([body_length / 2 - init_toe_x, body_width / 2, 0]),
            LegName.FRONT_RIGHT: np.array([body_length / 2 - init_toe_x, -body_width / 2, 0]),
            LegName.REAR_LEFT: np.array([-body_length / 2 + init_toe_x, body_width / 2, 0]),
            LegName.REAR_RIGHT: np.array([-body_length / 2 + init_toe_x, -body_width / 2, 0]),
        }

        # Shoulder coordinates in body frame
        self.shoulder_dict = {
            LegName.FRONT_LEFT: np.array([body_length / 2, body_width / 2, 0]),
            LegName.FRONT_RIGHT: np.array([body_length / 2, -body_width / 2, 0]),
            LegName.REAR_LEFT: np.array([-body_length / 2, body_width / 2, 0]),
            LegName.REAR_RIGHT: np.array([-body_length / 2, -body_width / 2, 0])
        }

        # Transformation matrix
        # Naming convention:
        #   b: body frame
        #   s: shoulder frame

        # Body to World transformation
        self.Tb = self.calc_transform_matrix([0, 0, 0], [0, 0, self.body_z])
        
        # Rotation for shoulder frames to align with the body frame
        Rbs_front = self.calc_rotation_matrix(0, -np.pi, 0) @ self.calc_rotation_matrix(-np.pi/2, 0, 0)
        Rbs_back = self.calc_rotation_matrix(np.pi/2, 0, 0)

        # Shoulder to Body transformation, remain constant all the time
        self.Tbs_dict = {}
        for name in LegName:
            Tbs = np.eye(4)
            Tbs[:3, :3] = Rbs_front if self.shoulder_dict[name][0] > 0 else Rbs_back
            Tbs[:3, 3] = self.shoulder_dict[name]
            self.Tbs_dict[name] = Tbs

        # Initialize the legs and calculate the end effector points
        self.ee_points = {}
        self.body_pose = []
        
        self.update_body_pose({
            OrientationType.ROLL: 0,
            OrientationType.PITCH: 0,
            OrientationType.YAW: 0
        })

    def get_ee_points(self):
        return self.ee_points
    
    def get_body_pose(self):
        return self.body_pose

    def calc_rotation_matrix(self, roll, pitch, yaw):
        """
        Compute 3D rotation matrix from roll, pitch, yaw (XYZ Euler angles).
        """
        Rx = np.array([[1, 0, 0],
                                        [0, np.cos(roll), -np.sin(roll)],
                                        [0, np.sin(roll), np.cos(roll)]])
        
        Ry = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                                        [0, 1, 0],
                                        [-np.sin(pitch), 0, np.cos(pitch)]])
        
        Rz = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                                        [np.sin(yaw), np.cos(yaw), 0],
                                        [0, 0, 1]])

        return Rz @ Ry @ Rx

    def calc_transform_matrix(self, orientation, position):
        """
        Compute 4x4 transformation matrix from roll, pitch, yaw (XYZ Euler angles) and position for 
        current frame to align with the target frame. 

        @param orientation: Roll, pitch, yaw angles for current frame to align with the target frame
        @param position: Position of the current frame's origin in the target frame's coordinates
        """
        [roll, pitch, yaw] = orientation
        R = self.calc_rotation_matrix(roll, pitch, yaw)
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = position
        return T
    
    def calc_inv_T(self, T):
        """
        Compute the inverse of the transformation matrix.
        """
        R = T[:3, :3]
        p = T[:3, 3]

        R_inv = R.T
        p_inv = -R_inv @ p

        T_inv = np.eye(4)
        T_inv[:3, :3] = R_inv
        T_inv[:3, 3] = p_inv

        return T_inv

    def calc_projected_point(self, transform_matrix, point):
        """
        Project a point to the new frame using the transformation matrix.

        @param transform_matrix: 4x4 transformation matrix
        @param point: Point to be projected
        """
        point = np.append(point, 1)
        projected_point = transform_matrix @ point
        return projected_point[:3]
    
    def update_body_pose(self, orientation_dict: dict[OrientationType, float]): 
        """
        Update the orientation and position of the body
        """
        roll = orientation_dict.get(OrientationType.ROLL, 0)
        pitch = orientation_dict.get(OrientationType.PITCH, 0)
        yaw = orientation_dict.get(OrientationType.YAW, 0)
        # Body to World transformation
        self.Tb = self.calc_transform_matrix([roll, pitch, yaw], [0, 0, self.body_z])
        
        # Shoulder to World transformation
        for name in LegName:
            Ts = self.Tb @ self.Tbs_dict[name]
            Ts_inv = self.calc_inv_T(Ts)
            pe = self.calc_projected_point(Ts_inv, self.foothold_dict[name])
            self.ee_points[name] = pe
            