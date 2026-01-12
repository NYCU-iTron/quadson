import sys
import logging
from common.logging_config import setup_logging
from real.motor_manager import MotorManager
from real.can_config import *
from src.real.can_config import CAN_EXT_TYPE

if __name__ == '__main__':
    setup_logging()
    logger = logging.getLogger(__name__)

    try:
        motor_manager = MotorManager()
    except Exception as e:
        logger.critical(f"Failed to initialize MotorManager: {e}", exc_info=True)
        exit(1)

    logger.info("Waiting for user input to continue...")
    user_input = input("Press Enter to continue to STM32 ID...")

    try:
        old_id, new_id = map(int, user_input.split())
    except ValueError:
        logger.error("Invalid input format. Please enter three integers separated by spaces.")

    try:
        motor_manager.send_motor_cmd(old_id, CAN_EXT_TYPE.CAN_EXTID_ID, new_id)

    except KeyboardInterrupt:
        logger.info("Ctrl+C detected, exiting...")

    except Exception as e:
        logger.error(f"An error occurred: {e}", exc_info=True)
        sys.exit(1)
