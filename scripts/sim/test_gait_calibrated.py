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
    current_time = 0.0
    
    p.connect(p.GUI)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
    p.setGravity(0, 0, -9.81)
    p.setTimeStep(dt)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.loadURDF("plane.urdf")

    quadson = Quadson()
    quadson.load_model("trained/quadson_ppo")

    try:
        for _ in range(1000):
            quadson.process_command(Command(CommandType.TEST_MODEL_CALIBRATION))
            p.stepSimulation()
            quadson.update_state()

            # Fixed camera position relative to the robot
            base_pos, _ = p.getBasePositionAndOrientation(quadson.robot_id)
            p.resetDebugVisualizerCamera(cameraDistance=0.8, cameraYaw=50, cameraPitch=-20, cameraTargetPosition=base_pos)

            current_time += dt
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