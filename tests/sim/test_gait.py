import pybullet as p
import pybullet_data
import time
from sim.quadson import Quadson
from analyze_stability import analyze_stability, plot_stability

def main():
    dt = 1 / 240
    current_time = 0.0
    
    p.connect(p.GUI) # (GUI for visualization, DIRECT for headless)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
    p.resetSimulation()
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.setTimeStep(dt)
    p.loadURDF("plane.urdf")
    
    quadson = Quadson()

    observations = {
        'pos': [],
        'euler_ori': [],
        'linear_vel': [],
    }
    times = []
    steps = 960
    for step in range(steps):
        quadson.step(current_time)
        p.stepSimulation()

        if step > 240:
            # Get observation
            obs = quadson.get_observation()
            euler_ori = obs[0:3]  # roll, pitch, yaw
            linear_vel = obs[3:6]  # x, y, z velocity
            pos, _ = p.getBasePositionAndOrientation(quadson.robot_id)

            # Store reduced data
            observations['pos'].append(pos)
            observations['euler_ori'].append(euler_ori)
            observations['linear_vel'].append(linear_vel)
            times.append(step * (1/240))  # Time in seconds

        current_time += dt
        time.sleep(dt)

    metrics = analyze_stability(observations)
    plot_stability(times, observations, metrics)

if __name__ == "__main__":
    main()
