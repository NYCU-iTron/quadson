import sys
import time
import logging
import numpy as np
from common.logging_config import setup_logging
from real.motor_manager import MotorManager
from real.leg import Leg, LegName

if __name__ == '__main__':
    setup_logging()
    logger = logging.getLogger(__name__)

    try:
        motor_manager = MotorManager()
    except Exception as e:
        logger.critical(f"Failed to initialize MotorManager: {e}", exc_info=True)
        exit(1)

    leg = Leg(LegName.FRONT_LEFT, motor_manager)
    leg.set_motor_indices([2, 5, 9])

    try:
        # -------------------------------- User Input -------------------------------- #
        leg.enable_torque(True)
        leg.set_control_mode(0)
        while True:
            curret_angles = leg.get_motor_angles()
            logger.info(f"Motor angles are at : {curret_angles}")

            logger.info("Waiting for user input to continue...")
            user_input = input("Press Enter to continue to motor angles...")

            try:
                a, b, c = map(int, user_input.split())
            except ValueError:
                logger.error("Invalid input format. Please enter three integers separated by spaces.")
                continue

            leg.set_motor_angles([np.radians(a), np.radians(b), np.radians(c)])
            logger.info(f"Set motor angles to: {np.radians(a)}, {np.radians(b)}, {np.radians(c)}")

            time.sleep(0.05)

    except KeyboardInterrupt:
        logger.info("Ctrl+C detected, exiting...")

    except Exception as e:
        logger.error(f"An error occurred: {e}", exc_info=True)
        sys.exit(1)

    finally:
        leg.stop_leg()
        time.sleep(0.05)
        motor_manager.shutdown()
        logger.info("Done")
