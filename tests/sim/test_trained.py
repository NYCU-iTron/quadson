import pybullet as p
import pybullet_data
import time
from sim.quadson import Quadson
from common.config import Command, CommandType
from analyze_stability import analyze_stability, plot_stability

dt = 1 / 240

# Set up PyBullet and Quadson
p.connect(p.GUI)
p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
p.setGravity(0, 0, -9.81)
p.setTimeStep(dt)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("plane.urdf")

robot = Quadson()
robot.load_model("trained/quadson_ppo")

# Initial observation
robot_state = robot.get_robot_state()
observations = {
    'pos': [],
    'euler_ori': [],
    'linear_vel': [],
}
times = []

# Run the model
steps = 960
for step in range(steps):
    robot.process_command(Command(CommandType.TEST_MODEL_CALIBRATION))
    p.stepSimulation()
    robot_state = robot.get_robot_state()

    # Store reduced data
    if step > 240:
        observations['pos'].append(robot_state.pose)
        observations['euler_ori'].append(robot_state.euler_orientation)
        observations['linear_vel'].append(robot_state.linear_velocity)
        times.append(step * (1/240))  # Time in seconds

    # Fixed camera position relative to the robot
    base_pos, _ = p.getBasePositionAndOrientation(robot.robot_id)
    p.resetDebugVisualizerCamera(cameraDistance=0.8, cameraYaw=50, cameraPitch=-20, cameraTargetPosition=base_pos)

    time.sleep(dt)

metrics = analyze_stability(observations)
plot_stability(times, observations, metrics)