import numpy as np
from typing import Dict, List
from common.config import Config

class Locomotion:
    """
    A class to perform locomotion calculations for a quadruped robot.
    """
    PHASE_OFFSETS = {
        'walk': {'fl': 0.0, 'fr': 0.5, 'rl': 0.25, 'rr': 0.75},
        'trot': {'fl': 0.0, 'fr': 0.5, 'rl': 0.5, 'rr': 0.0},
        'pace': {'fl': 0.0, 'fr': 0.0, 'rl': 0.5, 'rr': 0.5},
        'bound': {'fl': 0.0, 'fr': 0.0, 'rl': 0.5, 'rr': 0.5},
        'gallop': {'fl': 0.1, 'fr': 0.25, 'rl': 0.6, 'rr': 0.75},
    }
    VALID_GAIT_TYPES = list(PHASE_OFFSETS.keys())
    
    CYCLE_TIMES = {
        'walk': 1.2,
        'trot': 0.8,
        'pace': 0.7,
        'bound': 0.6,
        'gallop': 0.5
    }
        
    DUTY_FACTORS = {
        'walk': 0.75,
        'trot': 0.5,
        'pace': 0.5,
        'bound': 0.4,
        'gallop': 0.35
    }

    STEP_HEIGHTS = {
        'walk': 3.0,
        'trot': 5.0,
        'pace': 5.0,
        'bound': 8.0,
        'gallop': 7.0
    }
    
    STEP_LENGTHS = {
        'walk': 8.0,
        'trot': 12.0,
        'pace': 12.0,
        'bound': 16.0,
        'gallop': 21.0
    }

    def __init__(self, gait_type: str = 'trot'):
        """
        Initialize the Locomotion class with a specified gait type.

        :param gait_type: The type of gait to be used. Must be one of 'walk', 'trot', 'pace', 'bound', or 'gallop'.
        """
        if gait_type not in self.VALID_GAIT_TYPES:
            print(f"[Warning] Invalid gait type: {gait_type}, using trot as default")
            gait_type = 'trot'

        self.config = Config()
        self.gait_type = gait_type
        self.phase_dict = self.PHASE_OFFSETS[gait_type]
        self.duty_factor = self.DUTY_FACTORS[gait_type]
        self.cycle_time = self.CYCLE_TIMES[gait_type]
        self.step_height = self.STEP_HEIGHTS[gait_type]
        self.step_length = self.STEP_LENGTHS[gait_type]
    
    def get_current_phase(self, time: float) -> Dict[str, float]:
        """
        Get the phase of each leg based on the current time.

        :param time: The current time in seconds.
        :return phase_dict: A dictionary with leg names as keys and phase values as values.
        """
        phase_dict = {}
        cycle_progress = (time % self.cycle_time) / self.cycle_time
        for leg_name in self.config.legs:
            phase_dict[leg_name] = (cycle_progress + self.phase_dict[leg_name]) % 1.0
        return phase_dict
    
    def get_ee_points(self, time: float) -> Dict[str, List[float]]:
        """
        Get the end-effector points for each leg based on the current time.

        :param time: The current time in seconds.
        :return ee_points: A dictionary with leg names as keys and end-effector points as values.
        """
        phase_dict = self.get_current_phase(time)
        ee_points = {}
        base_y = -20.0
        
        for leg_name in self.config.legs:
            direction = 1 if leg_name in ['fl', 'fr'] else -1
            start_x = -4.0 if leg_name in ['fl', 'fr'] else 13.0
            
            # Stance phase
            if phase_dict[leg_name] < self.duty_factor:
                t = phase_dict[leg_name] / self.duty_factor # map to 0-1
                p_start = np.array([start_x, base_y])
                p_end = np.array([start_x + direction * self.step_length, base_y])
                p = p_start + (p_end - p_start) * t
                x, y = p
            
            # Swing phase
            else:
                t = (phase_dict[leg_name] - self.duty_factor) / (1 - self.duty_factor) # map to 0-1
                p0 = (start_x + direction * self.step_length, base_y)  # end
                p1 = (start_x + direction * self.step_length / 2, base_y + self.step_height)  # control point 2
                p2 = (start_x, base_y + self.step_height / 2)  # control point 1
                p3 = (start_x, base_y)  # start
                x, y = self._cubic_bezier(t, p0, p1, p2, p3)
                
            ee_points[leg_name] = [x, y, 0.0]  # z is always 0.0
            
        return ee_points
    
    @staticmethod
    def _cubic_bezier(t: float, p0: tuple[float, float], p1: tuple[float, float], 
                                        p2: tuple[float, float], p3: tuple[float, float]) -> tuple[float, float]:
        """
        Calculate position on a cubic Bezier curve.

        Args:
            t: The interpolation factor (0 <= t <= 1)
            p0, p1, p2, p3: Control points as (x, y) tuples
            
        Returns:
            (x, y) coordinates at position t along the curve
        """
        points = np.array([p0, p1, p2, p3])
        coeffs = np.array([(1-t)**3, 3*(1-t)**2*t, 3*(1-t)*t**2, t**3])
        
        return tuple(coeffs @ points)
    