from stable_baselines3 import PPO
import pybullet as p
import pybullet_data
import time
from pathlib import Path
from sim.quadson import Quadson
from sim.interface import Interface
from common.config import Config
from analyze_stability import analyze_stability, plot_stability

model_path = Path(__file__).resolve().parent.parent.parent / "assets/trained/quadson_ppo"
model = PPO.load(model_path, device='cpu')

dt = 1 / 240
current_time = 0.0

# Set up PyBullet and Quadson
p.connect(p.GUI)
p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
p.setGravity(0, 0, -9.81)
p.setTimeStep(dt)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("plane.urdf")

interface = Interface(type="model", target="ee_offset")
robot = Quadson(interface)
config = Config()

# Initial observation
obs = robot.get_observation()
observations = {
  'pos': [],
  'euler_ori': [],
  'linear_vel': [],
}
times = []

# Run the model
steps = 960
for step in range(steps):
  action, _states = model.predict(obs, deterministic=True)
  cmd_dict = {}
  for i, leg_name in enumerate(config.legs):  # Adjust leg names as per your config
    start = i * 3
    end = start + 3
    cmd_dict[leg_name] = action[start:end]
  robot.update(cmd_dict)

  p.stepSimulation()

  obs = robot.get_observation()
  euler_ori = obs[0:3]  # roll, pitch, yaw
  linear_vel = obs[3:6]  # x, y, z velocity
  pos, _ = p.getBasePositionAndOrientation(robot.robot_id)

  # Store reduced data
  if step > 240:
    observations['pos'].append(pos)
    observations['euler_ori'].append(euler_ori)
    observations['linear_vel'].append(linear_vel)
    times.append(step * (1/240))  # Time in seconds

  # Fixed camera position relative to the robot
  base_pos, _ = p.getBasePositionAndOrientation(robot.robot_id)
  p.resetDebugVisualizerCamera(cameraDistance=0.8, cameraYaw=50, cameraPitch=-20, cameraTargetPosition=base_pos)

  current_time += dt
  time.sleep(dt)

metrics = analyze_stability(observations)
plot_stability(times, observations, metrics)