import pybullet as p
import time
import numpy as np
from pathlib import Path
from common.leg_kinematics import LegKinematics

urdf_path = Path(__file__).resolve().parent.parent.parent / "assets/single_leg/urdf/single_leg.urdf"

def joint_ctrl(joint_index, target_angle):
  p.setJointMotorControl2(
    bodyUniqueId=robot_id,
    jointIndex=joint_index,
    controlMode=p.POSITION_CONTROL,
    targetPosition=target_angle,
    force=500
  )

def enfore_closure(theory_angles):
  # Theory to env
  j1_env = np.pi - theory_angles[1]
  j5_env = np.pi/2 - theory_angles[5]

  j2_env = 1.2406 - (np.pi + theory_angles[2] - theory_angles[1])
  j4_env =  - 1.6833 + (np.pi + theory_angles[5] - theory_angles[4])

  joint_ctrl(0, j1_env)
  joint_ctrl(1, j2_env)
  joint_ctrl(3, j4_env)
  joint_ctrl(2, j5_env)

p.connect(p.GUI)
p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
p.setGravity(0, 0, -9.81)
robot_id = p.loadURDF(str(urdf_path), useFixedBase=True)
time_step = 1.0 / 240.0
p.setRealTimeSimulation(0)
kinematics = LegKinematics(DEBUG=True)

# Joint info
# for i in range(p.getNumJoints(robot_id)):
#   print(i, p.getJointInfo(robot_id, i)[1].decode("utf-8"))

x_id = p.addUserDebugParameter("x", -5, 20, -2.16399823)
y_id = p.addUserDebugParameter("y", -20, -10, -17.02765643)
# j1_id = p.addUserDebugParameter("j1", 0, np.pi, np.pi)
# j5_id = p.addUserDebugParameter("j5", 0, np.pi, np.pi/2)

for t in range(10000):
  x = p.readUserDebugParameter(x_id)
  y = p.readUserDebugParameter(y_id)
  ee_point = [x, y, 0]
  kinematics.set_ee_point(ee_point)

  # j1 = p.readUserDebugParameter(j1_id)
  # j5 = p.readUserDebugParameter(j5_id)
  # kinematics.set_motor_angles([0, j1, j5])
  # ee_point = kinematics.get_ee_point()

  theory_angles = kinematics.get_angles()
  enfore_closure(theory_angles)

  p.stepSimulation()
  time.sleep(time_step)

while True:
  p.stepSimulation()
  time.sleep(time_step)

# p.disconnect()
