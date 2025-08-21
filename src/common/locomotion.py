import numpy as np
import logging
from common.config import LegName, GaitType

class Locomotion:
    """
    A class to perform locomotion calculations for a quadruped robot.
    """
    phase_offsets = {
        GaitType.WALK: {LegName.FRONT_LEFT: 0.0, LegName.FRONT_RIGHT: 0.5, LegName.REAR_LEFT: 0.25, LegName.REAR_RIGHT: 0.75},
        GaitType.TROT: {LegName.FRONT_LEFT: 0.0, LegName.FRONT_RIGHT: 0.5, LegName.REAR_LEFT: 0.5, LegName.REAR_RIGHT: 0.0},
        GaitType.PACE: {LegName.FRONT_LEFT: 0.0, LegName.FRONT_RIGHT: 0.0, LegName.REAR_LEFT: 0.5, LegName.REAR_RIGHT: 0.5},
        GaitType.BOUND: {LegName.FRONT_LEFT: 0.0, LegName.FRONT_RIGHT: 0.0, LegName.REAR_LEFT: 0.5, LegName.REAR_RIGHT: 0.5},
        GaitType.GALLOP: {LegName.FRONT_LEFT: 0.1, LegName.FRONT_RIGHT: 0.25, LegName.REAR_LEFT: 0.6, LegName.REAR_RIGHT: 0.75},
    }

    cycle_times = {
        GaitType.WALK: 1.2,
        GaitType.TROT: 0.8,
        GaitType.PACE: 0.7,
        GaitType.BOUND: 0.6,
        GaitType.GALLOP: 0.5
    }
        
    duty_factors = {
        GaitType.WALK: 0.75,
        GaitType.TROT: 0.5,
        GaitType.PACE: 0.5,
        GaitType.BOUND: 0.4,
        GaitType.GALLOP: 0.35
    }

    step_heights = {
        GaitType.WALK: 3.0,
        GaitType.TROT: 5.0,
        GaitType.PACE: 5.0,
        GaitType.BOUND: 8.0,
        GaitType.GALLOP: 7.0
    }
    
    step_lengths = {
        GaitType.WALK: 8.0,
        GaitType.TROT: 12.0,
        GaitType.PACE: 12.0,
        GaitType.BOUND: 16.0,
        GaitType.GALLOP: 21.0
    }

    def __init__(self, gait_type: GaitType = GaitType.TROT):
        """
        Initialize the Locomotion class with a specified gait type.

        :param gait_type: The type of gait to be used. Must be one of 'walk', 'trot', 'pace', 'bound', or 'gallop'.
        """
        self.logger = logging.getLogger(__name__)
        
        if gait_type not in self.phase_offsets:
            self.logger.warning(f"Invalid gait type: {gait_type}, using trot as default")
            gait_type = GaitType.TROT
        
        self.gait_type = gait_type
        self.phase_dict = self.phase_offsets[gait_type]
        self.duty_factor = self.duty_factors[gait_type]
        self.cycle_time = self.cycle_times[gait_type]
        self.step_height = self.step_heights[gait_type]
        self.step_length = self.step_lengths[gait_type]
    
    def get_current_phase(self, time: float) -> dict[LegName, float]:
        """
        Get the phase of each leg based on the current time.

        :param time: The current time in seconds.
        :return phase_dict: A dictionary with leg names as keys and phase values as values.
        """
        phase_dict = {}
        cycle_progress = (time % self.cycle_time) / self.cycle_time
        for name in LegName:
            phase_dict[name] = (cycle_progress + self.phase_dict[name]) % 1.0
        return phase_dict
    
    def get_ee_points(self, time: float) -> dict[int, list[float]]:
        """
        Get the end-effector points for each leg based on the current time.

        :param time: The current time in seconds.
        :return ee_points: A dictionary with leg names as keys and end-effector points as values.
        """
        phase_dict = self.get_current_phase(time)
        ee_points = {}
        base_y = -20.0
        
        for leg_id in LegName:
            direction = 1 if leg_id in [LegName.FRONT_LEFT, LegName.FRONT_RIGHT] else -1
            start_x = -4.0 if leg_id in [LegName.FRONT_LEFT, LegName.FRONT_RIGHT] else 13.0
            
            # Stance phase
            if phase_dict[leg_id] < self.duty_factor:
                t = phase_dict[leg_id] / self.duty_factor # map to 0-1
                p_start = np.array([start_x, base_y])
                p_end = np.array([start_x + direction * self.step_length, base_y])
                p = p_start + (p_end - p_start) * t
                x, y = p
            
            # Swing phase
            else:
                t = (phase_dict[leg_id] - self.duty_factor) / (1 - self.duty_factor) # map to 0-1
                p0 = (start_x + direction * self.step_length, base_y)  # end
                p1 = (start_x + direction * self.step_length / 2, base_y + self.step_height)  # control point 2
                p2 = (start_x, base_y + self.step_height / 2)  # control point 1
                p3 = (start_x, base_y)  # start
                x, y = self.cubic_bezier(t, p0, p1, p2, p3)
                
            ee_points[leg_id] = [x, y, 0.0]  # z is always 0.0
            
        return ee_points
    
    def cubic_bezier(self, t: float, p0: tuple[float, float], p1: tuple[float, float], 
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
    