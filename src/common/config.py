class Config:
    """
    A class to store the joint names for the legs.
    """
    def __init__(self):
        ## Front left leg
        self._fl = "fl"
        self._fl_joint0 = "fl_joint0"
        self._fl_joint1 = "fl_joint1"
        self._fl_joint2 = "fl_joint2"
        self._fl_joint4 = "fl_joint4"
        self._fl_joint5 = "fl_joint5"
        self._fl_joints = [self._fl_joint0, self._fl_joint1, self._fl_joint2, self._fl_joint4, self._fl_joint5]
        self._fl_motors = [self._fl_joint0, self._fl_joint1, self._fl_joint5]
        
        ## Front right leg
        self._fr = "fr"
        self._fr_joint0 = "fr_joint0"
        self._fr_joint1 = "fr_joint1"
        self._fr_joint2 = "fr_joint2"
        self._fr_joint4 = "fr_joint4"
        self._fr_joint5 = "fr_joint5"
        self._fr_joints = [self._fr_joint0, self._fr_joint1, self._fr_joint2, self._fr_joint4, self._fr_joint5]
        self._fr_motors = [self._fr_joint0, self._fr_joint1, self._fr_joint5]
        
        ## Rear left leg
        self._rl = "rl"
        self._rl_joint0 = "rl_joint0"
        self._rl_joint1 = "rl_joint1"
        self._rl_joint2 = "rl_joint2"
        self._rl_joint4 = "rl_joint4"
        self._rl_joint5 = "rl_joint5"
        self._rl_joints = [self._rl_joint0, self._rl_joint1, self._rl_joint2, self._rl_joint4, self._rl_joint5]
        self._rl_motors = [self._rl_joint0, self._rl_joint1, self._rl_joint5]
        
        ## Rear right leg
        self._rr = "rr"
        self._rr_joint0 = "rr_joint0"
        self._rr_joint1 = "rr_joint1"
        self._rr_joint2 = "rr_joint2"
        self._rr_joint4 = "rr_joint4"
        self._rr_joint5 = "rr_joint5"
        self._rr_joints = [self._rr_joint0, self._rr_joint1, self._rr_joint2, self._rr_joint4, self._rr_joint5]
        self._rr_motors = [self._rr_joint0, self._rr_joint1, self._rr_joint5]

        # Public
        self.legs = [self._fl, self._fr, self._rl, self._rr]
        self.joint_dict = {self._fl:self._fl_joints, self._fr:self._fr_joints,
                           self._rl:self._rl_joints, self._rr:self._rr_joints}
        self.motor_dict = {self._fl:self._fl_motors, self._fr:self._fr_motors,
                           self._rl:self._rl_motors, self._rr:self._rr_motors}
        