import time
import logging
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
        while (1):
            logger.info(f"Motor 1 state: {motor_manager.get_zero_state(1)}")
            # logger.info(f"Motor 1 angle: {motor_manager.get_motor_angle(1)}")
            time.sleep(0.05)
    except KeyboardInterrupt:
        logger.info("Ctrl+C detected, exiting...")
    finally:
        motor_manager.shutdown()
        logger.info("Done")
