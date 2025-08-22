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
        lg.enable_torque(True)
        lg.set_control_mode(1)
        lg.set_motor_omegas([np.pi, np.pi, np.pi])

        lg_zero_states = [False, False, False]
        while True:
            lg_zero_states = lg.get_zero_states()
            if lg_zero_states == [None, True, True]:
                logger.info("All motors are zeroed, exiting loop.")
                break
            logger.info(f"Zero states: {lg_zero_states}")
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