import pybullet as p
import pybullet_data
import time
import logging
from common.logging_config import setup_logging
from common.config import CommandType
from sim.quadson import Quadson
from sim.slider import Slider

def main():
    logger = logging.getLogger(__name__)

    dt = 1 / 240

    p.connect(p.GUI) # (GUI for visualization, DIRECT for headless)
    p.resetSimulation()
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.setTimeStep(dt)
    p.loadURDF("plane.urdf")
    
    slider = Slider(CommandType.ORIENTATION)
    quadson = Quadson()

    try:
        while True:
            command = slider.get_command()
            quadson.process_command(command)
            p.stepSimulation()
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
