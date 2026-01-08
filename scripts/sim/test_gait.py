import pybullet as p
import pybullet_data
import logging
import time
from common.logging_config import setup_logging
from common.config import Command, CommandType
from sim.quadson import Quadson

def main():
    logger = logging.getLogger(__name__)

    dt = 1 / 240

    p.connect(p.GUI)
    p.resetSimulation()
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.setTimeStep(dt)
    p.loadURDF("plane.urdf")

    quadson = Quadson()
    command = Command(CommandType.TEST_LOCOMOTION)
    
    try:
        for _ in range(10000):
            quadson.process_command(command)
            p.stepSimulation()
            quadson.update_state()

            time.sleep(dt)

    except KeyboardInterrupt:
        logger.info("Simulation interrupted by user.")

    except Exception as e:
        logger.error("An error occurred: %s", e)

    finally:
        p.disconnect()

if __name__ == "__main__":
    setup_logging()
    main()