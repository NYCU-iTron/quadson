from dataclasses import dataclass
from enum import Enum, auto
from typing import Optional
import numpy as np

# ----------------------------------- Enum ----------------------------------- #
class LegName(Enum):
    FRONT_LEFT = 0
    FRONT_RIGHT = auto()
    REAR_LEFT = auto()
    REAR_RIGHT = auto()

class GaitType(Enum):
    AUTO = 0
    WALK = auto()
    TROT = auto()
    PACE = auto()
    BOUND = auto()
    GALLOP = auto()

class OrientationType(Enum):
    ROLL = 0
    PITCH = auto()
    YAW = auto()

class CommandType(Enum):
    MOTOR_ANGLES = 0
    EE_POINTS = auto()
    ORIENTATION = auto()
    TWIST = auto()
    TEST_LOCOMOTION = auto()
    TEST_MODEL_CALIBRATION = auto()
    TRAIN_MODEL = auto()

# -------------------------------- Data Class -------------------------------- #
@dataclass
class LegState:
    name: Optional[LegName] = None
    time: Optional[float] = None
    motor_angles: Optional[np.ndarray[float]] = None
    motor_omegas: Optional[np.ndarray[float]] = None
    joint_points: Optional[np.ndarray[float]] = None
    joint_angles: Optional[np.ndarray[float]] = None
    ee_point: Optional[np.ndarray[float]] = None
    ee_velocity: Optional[np.ndarray[float]] = None

@dataclass
class RobotState:
    time: float
    pose: Optional[np.ndarray[float]] = None
    orientation: Optional[np.ndarray[float]] = None
    euler_orientation: Optional[np.ndarray[float]] = None
    linear_velocity: Optional[np.ndarray[float]] = None
    angular_velocity: Optional[np.ndarray[float]] = None
    linear_accleration: Optional[np.ndarray[float]] = None
    leg_states: Optional[list[LegState]] = None

@dataclass
class Command:
    type: CommandType
    data: Optional[dict] = None
    