import sys
import time
import logging
import numpy as np
from common.logging_config import setup_logging
from real.motor_manager import MotorManager
from real.leg import Leg

if __name__ == '__main__':
    setup_logging()
    logger = logging.getLogger(__name__)

    try:
        motor_manager = MotorManager()
    except Exception as e:
        logger.critical(f"Failed to initialize MotorManager: {e}", exc_info=True)
        exit(1)
    
    lg = Leg(0, motor_manager)

    try:
        # ----------------------------------- Zero ----------------------------------- #
        lg.enable_torque(True)
        lg.set_control_mode(1)
        lg.set_motor_omegas([None, np.pi, np.pi])

        lg_zero_states = [False, False, False]
        done = [None, False, False]
        while True:
            lg_zero_states = lg.get_zero_states()

            if done == [None, True, True]:
                logger.info("All motors are zeroed, exiting loop.")
                break

            if lg_zero_states[1] == True and not done[1]:
                logger.info("Motor 1 is zeroed")
                lg.set_motor_omegas([None, 0, None])
                done[1] = True
            
            if lg_zero_states[2] == True and not done[2]:
                logger.info("Motor 2 is zeroed")
                lg.set_motor_omegas([None, None, 0])
                done[2] = True

            logger.info(f"Zero states: {lg_zero_states}")
            time.sleep(0.05)

        # lg.set_control_mode(0)
        # angle = motor_manager.get_motor_angle(1)
        # logger.info(f"Motor 1 deg: {np.rad2deg(angle)}")

        # -------------------------------- User Input -------------------------------- #
        while True:
            logger.info("Waiting for user input to continue...")
            user_input = input("Press Enter to continue to motor angles...")

            try:
                a, b, c = map(int, user_input.split())
            except ValueError:
                logger.error("Invalid input format. Please enter three integers separated by spaces.")
                continue

            lg.set_motor_angles([0, np.radians(b), np.radians(c)])
            logger.info(f"Set motor angles to: 0, {b}, {c}")


            time.sleep(0.05)

    except KeyboardInterrupt:
        logger.info("Ctrl+C detected, exiting...")

    except Exception as e:
        logger.error(f"An error occurred: {e}", exc_info=True)
        sys.exit(1)

    finally:
        lg.stop_leg()
        time.sleep(0.05)
        motor_manager.shutdown()
        logger.info("Done")