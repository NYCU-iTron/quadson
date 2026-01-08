import pybullet as p
import pybullet_data
import logging
import time
from common.logging_config import setup_logging
from common.config import CommandType
from sim.slider import Slider
from sim.quadson import Quadson as QuadsonSim
from real.quadson import Quadson as QuadsonReal

def main():
    logger = logging.getLogger(__name__)

    dt = 1 / 240
    
    p.connect(p.GUI)
    p.resetSimulation()
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.setTimeStep(dt)
    p.loadURDF("plane.urdf")

    slider = Slider(CommandType.EE_POINTS)
    quadson_sim = QuadsonSim()
    quadson_real = QuadsonReal()

    try:
        while True:
            command = slider.get_command()
            quadson_sim.process_command(command)
            quadson_real.process_command(command)

            p.stepSimulation()
            time.sleep(dt)

    except KeyboardInterrupt:
        logger.info("Ctrl+C detected, exiting...")

    except Exception as e:
        logger.error("An error occurred: %s", e)

    finally:
        quadson_real.shutdown()
        p.disconnect()

if __name__ == "__main__":
    setup_logging()
    main()
    