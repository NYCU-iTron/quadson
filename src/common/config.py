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
    euler_ori: np.ndarray[float]
    pose: np.ndarray[float]
    ori: np.ndarray[float]
    linear_vel: np.ndarray[float]
    angular_vel: np.ndarray[float]
    legs: list[LegState]

@dataclass
class Command:
    type: CommandType
    data: dict
    