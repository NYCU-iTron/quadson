import sys
import time
import logging
import numpy as np
from common.logging_config import setup_logging
from real.motor_manager import MotorManager

if __name__ == '__main__':
    setup_logging()
    logger = logging.getLogger(__name__)

    try:
        motor_manager = MotorManager()
    except Exception as e:
        logger.critical(f"Failed to initialize MotorManager: {e}", exc_info=True)
        exit(1)

    try:
        motor_manager.enable_motor_torque(1, True)
        motor_manager.enable_motor_torque(2, True)

        # Set to velocity control mode
        motor_manager.set_control_mode(1, 1)
        motor_manager.set_control_mode(2, 1)

        motor_manager.set_motor_omega(1, np.pi)
        motor_manager.set_motor_omega(2, np.pi)

        motor_1_zero_state = False
        motor_2_zero_state = False
        motor_1_done = False
        motor_2_done = False

        while (1):
            if motor_manager.error_flag:
                logger.error("Detected CAN thread error, stopping main loop.")
                break

            motor_1_zero_state = motor_manager.get_zero_state(1)
            motor_2_zero_state = motor_manager.get_zero_state(2)

            if (motor_1_zero_state == 1) and not motor_1_done:
                logger.info("Motor 1 zero done")
                motor_manager.set_motor_omega(1, 0)
                motor_manager.set_control_mode(1, 0)
                motor_manager.enable_motor_torque(1, False)
                motor_1_done = True

            if (motor_2_zero_state == 1) and not motor_2_done:
                logger.info("Motor 2 zero done")
                motor_manager.set_motor_omega(2, 0)
                motor_manager.set_control_mode(2, 0)
                motor_manager.enable_motor_torque(2, False)
                motor_2_done = True
            
            if motor_1_done and motor_2_done:
                logger.info("Both motors are zeroed, exiting loop.")
                break
            
            logger.info(f"Motor 1 state: {motor_1_done}")
            logger.info(f"Motor 2 state: {motor_2_done}")
            
            time.sleep(0.05)

    except KeyboardInterrupt:
        logger.info("Ctrl+C detected, exiting...")

    except Exception as e:
        logger.error(f"An error occurred: {e}", exc_info=True)
        sys.exit(1)

    finally:
        motor_manager.stop_motor(1)
        motor_manager.stop_motor(2)

        time.sleep(0.05)

        motor_manager.shutdown()
        logger.info("Done")
