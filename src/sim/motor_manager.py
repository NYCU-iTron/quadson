import logging
import pybullet as p

class MotorManager:
    def __init__(self, robot_id: int):
        self.logger = logging.getLogger(__name__)

        self.robot_id = robot_id

        motor_dict = self.setup_motor_dict()
        self.logger.debug(f"Motor dictionary: {motor_dict}")

    def setup_motor_dict(self) -> dict:
        motor_dict = {
            p.getJointInfo(self.robot_id, i)[1].decode("utf-8"): i
            for i in range(p.getNumJoints(self.robot_id))
        }
        return motor_dict
    
    def set_motor_angle(self, motor_id: int, angle) -> None:
        if motor_id < 0 or motor_id > 19:
            self.logger.error(f"Invalid motor_id: {motor_id}. Must be between 0 and 19.")
            return
        
        p.setJointMotorControl2(
            bodyUniqueId=self.robot_id,
            jointIndex=motor_id,
            controlMode=p.POSITION_CONTROL,
            targetPosition=angle,
            force=800)
        