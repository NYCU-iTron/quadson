import sys
import logging
import time
from common.logging_config import setup_logging
from real.motor_manager import MotorManager
from real.can_config import *
from real.can_config import CAN_EXT_TYPE, CAN_STD_TYPE

if __name__ == '__main__':
    setup_logging()
    logger = logging.getLogger(__name__)

    try:
        motor_manager = MotorManager()
    except Exception as e:
        logger.critical(f"Failed to initialize MotorManager: {e}", exc_info=True)
        exit(1)

    logger.info("=== STM32 Motor ID Changer Tool ===")
    logger.info("WARNING: This operation writes to Flash memory.")

    try:
        input_str = input("Enter old STM32 ID and new STM32 ID separated by space (e.g., '1 2'): ")
        old_id, new_id = map(int, input_str.split())

        logger.info(f"Preparing to change ID from {old_id} to {new_id}...")
        logger.info(f"Step 1: Disabling Torque for ID {old_id} to enter IDLE state...")
        motor_manager.send_motor_cmd(old_id, CAN_EXT_TYPE.CAN_EXTID_TORQUE_ENABLE, 0)
        time.sleep(0.1)


        logger.info(f"Step 2: Changing ID in RAM from {old_id} to {new_id}...")
        motor_manager.send_motor_cmd(old_id, CAN_EXT_TYPE.CAN_EXTID_ID, new_id)
        time.sleep(0.1)

        logger.info(f"Step 3: Sending Save Config command...")
        save_cmd = CAN_EXT_TYPE.CAN_EXTID_SAVE_CONFIG

        # Keep using old_id to send the save command
        motor_manager.send_motor_cmd(old_id, save_cmd, 1)

        logger.info("Save command sent. Waiting for Flash write cycle...")
        time.sleep(1.0)
        logger.info("=== Operation Complete ===")
        logger.info(f"Please restart the script and try communicating with NEW ID: {new_id}")

    except ValueError:
        logger.error("Invalid input format. Please enter two integers separated by spaces.")

    except KeyboardInterrupt:
        logger.info("\nCtrl+C detected, exiting...")

    except Exception as e:
        logger.error(f"An error occurred: {e}", exc_info=True)
        sys.exit(1)

    finally:
        motor_manager.shutdown()
        sys.exit(0)
