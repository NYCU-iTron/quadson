from time import sleep
import numpy as np
import logging
from common.logging_config import setup_logging
from real.motor_manager_cando import MotorManager

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

        # Velocity mode
        motor_manager.set_control_mode(1, 1)
        motor_manager.set_control_mode(2, 1)

        # Set initial velocity
        motor_manager.set_motor_omega(1, 0.6)
        motor_manager.set_motor_omega(2, 0.6)

        motor_1_zero_state = False
        motor_2_zero_state = False

        while (1):
            motor_1_zero_state = motor_manager.get_zero_state(1)
            motor_2_zero_state = motor_manager.get_zero_state(2)

            if (motor_1_zero_state == 1):
                logger.info("Motor 1 zero done")
                motor_manager.set_motor_omega(1, 0)
                motor_manager.set_control_mode(1, 0) # Position mode
                motor_manager.enable_motor_torque(1, False)

            if (motor_2_zero_state == 1):
                logger.info("Motor 2 zero done")
                motor_manager.set_motor_omega(2, 0)
                motor_manager.set_control_mode(2, 0) # Position mode
                motor_manager.enable_motor_torque(2, False)
            
            if motor_1_zero_state and motor_2_zero_state:
                logger.info("Both motors are zeroed, exiting loop.")
                break
            
            logger.info(f"Motor 1 state: {motor_manager.get_zero_state(1)}")
            logger.info(f"Motor 2 state: {motor_manager.get_zero_state(2)}")
            
            sleep(0.05)

    except KeyboardInterrupt:
        motor_manager.set_motor_omega(1, 0)
        motor_manager.set_motor_omega(2, 0)
        motor_manager.set_control_mode(1, 0)
        motor_manager.set_control_mode(2, 0)
        motor_manager.enable_motor_torque(1, False)
        motor_manager.enable_motor_torque(2, False)
        logger.info("Ctrl+C detected, exiting...")
    finally:
        motor_manager.shutdown()
        logger.info("Done")