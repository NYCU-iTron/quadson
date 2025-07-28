from time import sleep
import numpy as np
from real.motor_manager import MotorManager
from real.leg_group import LegGroup

if __name__ == '__main__':
    motor_manager = MotorManager()
    leggroup = LegGroup(0, motor_manager)

    while (1):
        print(f"Angle: {motor_manager.get_motor_angle(1)}")
        print(f"Zero: {motor_manager.get_zero_state(1)}")

    # sleep(1)

    # leggroup.enable_torque(True)
    # leggroup.set_motor_angles([np.pi / 2, np.pi / 2, 0])

    # sleep(1)

    # leggroup.enable_torque(False)
    # motor_manager.disconnect_can_device()