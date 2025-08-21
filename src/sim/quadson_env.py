import numpy as np
import pybullet as p
import pybullet_data
from matplotlib import pyplot as plt
import gymnasium as gym
from stable_baselines3.common.callbacks import BaseCallback
import logging
from common.config import LegName, Command, CommandType, RobotState
from sim.quadson import Quadson

class QuadsonEnv(gym.Env):
    def __init__(self):
        super().__init__()
        self.logger = logging.getLogger(__name__)

        p.connect(p.GUI)  # use p.DIRECT when training
        p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
        p.setGravity(0, 0, -9.81)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setTimeStep(1/240)

        planeId = p.loadURDF("plane.urdf")
        p.changeDynamics(planeId, 
                         -1, 
                         lateralFriction = 0.8,       # 側向摩擦力(0-1)
                         spinningFriction = 0.1,      # 自旋摩擦力(通常小於側向摩擦力)
                         rollingFriction = 0.01,      # 滾動摩擦力(通常更小)
                         restitution = 0.2,           # 彈性係數(0-1)
                         contactDamping = 10.0,       # 接觸阻尼
                         contactStiffness = 1000.0)   # 接觸剛度  

        self.robot = Quadson()

        self.rewards = []
        self.step_counter = 0
        self.max_steps = 4500

        # Observation space
        # body_state:
        #     Orientation (roll, pitch, yaw) +
        #     Linear velocity (x, y, z) +
        #     Angular velocity (x, y, z) = 9
        # joint_state:
        #     12 joints × (position) = 12 # Add velocity
        # leg phases:
        #   sin(phase_LF) +
        #   cos(phase_LF) +
        #   sin(phase_RF) +
        #   cos(phase_RF) +
        #   sin(phase_LH) +
        #   cos(phase_LH) +
        #   sin(phase_RH) +
        #   cos(phase_RH) = 8
        # Total = 29

        self.observation_space = gym.spaces.Box(low=-np.inf, high=np.inf, shape=(29,), dtype=np.float32)
        
        # x, y, z offset in four legs
        self.action_space = gym.spaces.Box(low=-5, high=5, shape=(12,), dtype=np.float32)
        self.last_action = np.zeros(12)

    def reset(self, *, seed=None, options=None):
        p.resetSimulation()
        p.setGravity(0, 0, -9.81)
        p.loadURDF("plane.urdf")
        self.robot = Quadson()
        observation = self.robot.get_robot_state()
        return observation, {}
    
    def step(self, action):
        ee_offsets = {}
        for name in LegName:
            start_motor_id = name.value * 3
            end_motor_id = start_motor_id + 3
            ee_offsets[name] = action[start_motor_id:end_motor_id]

        self.robot.process_command(Command(CommandType.TRAIN_MODEL, ee_offsets))
        p.stepSimulation()

        # Fix the camera
        basePos, _ = p.getBasePositionAndOrientation(self.robot.robot_id) # Get model position
        p.resetDebugVisualizerCamera(cameraDistance=0.8, cameraYaw=50, cameraPitch=-20, cameraTargetPosition=basePos) # fix camera onto model

        self.step_counter += 1
        self.current_action = action

        observation = self.robot.get_robot_state()
        reward = self.get_reward(observation)
        done = self.check_done()
        info = {}
        truncated = False

        if done == True:
            self.step_counter = 0

        return observation, reward, done, truncated, info
    
    def get_reward(self, robot_state: RobotState):
        target_x_vel = 0.5
        target_height = 0.2

        roll, pitch, yaw = robot_state.euler_orientation
        linear_velocity = robot_state.linear_velocity
        angular_velocity = robot_state.angular_velocity
        x, y, z = robot_state.pose

        # Forward velocity
        vel_diff = linear_velocity[0] - target_x_vel
        forward_reward = np.exp(-0.5 * vel_diff**2)

        # Lateral motion
        lateral_penalty = 0.5 * linear_velocity[1]**2 + 0.3 * angular_velocity[1]**2

        # Vertical motion
        vertical_penalty = 0.5 * linear_velocity[2]**2 + 0.3 * angular_velocity[2]**2

        # Pose stability
        orientation_penalty = 40 * roll**2 + 30 * pitch**2 + 5 * yaw**2

        # Negative x position
        backward_penalty = 2.0 * np.clip(-linear_velocity[0], 0, None)**2

        # Y axis stability
        y_diff = y
        y_penalty = 1.3 * y_diff**2

        # Height
        height_diff = z - target_height
        height_penalty = 0.5 * height_diff**2

        if hasattr(self, 'prev_orientation'):
            prev_roll, prev_pitch, prev_yaw = self.prev_orientation
            orientation_change_penalty = (+1.2 * (roll - prev_roll)**2
                                          + 0.8 * (pitch - prev_pitch)**2
                                          + 0.3 * (yaw - prev_yaw)**2)
        else:
            orientation_change_penalty = 0
        self.prev_orientation = (roll, pitch, yaw)

        if hasattr(self, 'last_action'):
            action = self.current_action
            energy_penalty = 0.01 * np.sum(np.square(action))
            
            # Smoothness penalty
            action_change = action - self.last_action
            smoothness_penalty = 0.05 * np.sum(np.square(action_change))
        else:
            energy_penalty = 0
            smoothness_penalty = 0
        
        self.last_action = self.current_action.copy() if hasattr(self, 'current_action') else np.zeros_like(self.action_space.sample())

        reward = (+ 1.0 * forward_reward              # 前進獎勵
                  - 0.8 * lateral_penalty             # 側向穩定性懲罰
                  - 0.8 * vertical_penalty            # 垂直穩定性懲罰
                  - 1.0 * orientation_penalty         # 姿態穩定性懲罰
                  - 1.0 * orientation_change_penalty  # 姿態變化率懲罰
                  - 0.5 * backward_penalty            # 後退懲罰
                  - 0.5 * y_penalty                   # Y 軸穩定性懲罰
                  - 0.5 * height_penalty              # 高度穩定性懲罰
                  - energy_penalty                    # 能量效率懲罰
                  - 0.3 * smoothness_penalty)         # 動作平滑度懲罰

        # self.logger.info('\n')
        # self.logger.info('forward_reward            : ', forward_reward            )  
        # self.logger.info('lateral_penalty           : ', lateral_penalty           )  
        # self.logger.info('vertical_penalty          : ', vertical_penalty          )  
        # self.logger.info('orientation_penalty       : ', orientation_penalty       )  
        # self.logger.info('orientation_change_penalty: ', orientation_change_penalty)  
        # self.logger.info('backward_penalty          : ', backward_penalty          )  
        # self.logger.info('y_penalty                 : ', y_penalty                 )  
        # self.logger.info('height_penalty            : ', height_penalty            )  
        # self.logger.info('energy_penalty            : ', energy_penalty            )        
        # self.logger.info('smoothness_penalty        : ', smoothness_penalty        )        
                
        return reward

    def check_done(self, robot_state: RobotState):
        roll, pitch, _ = robot_state.euler_orientation
        z = robot_state.pose[2]

        if abs(roll) > np.pi / 4 or abs(pitch) > np.pi / 4:  # 45 degrees tilt
            return True

        if z < 0.1:  # robot collapsed
            return True

        if self.step_counter >= self.max_steps:
            return True

        return False

class PlottingCallback(BaseCallback):
    def __init__(self, window_size=25, verbose=0, update_freq=1):
        super().__init__(verbose)        
        self.window_size = window_size
        self.update_freq = update_freq
        
        # Recording variables
        self.episode_rewards = []
        self.moving_avg = []
        self.std_devs = []
        self.current_reward = 0
        self.episode_count = 0
        
        self.fig = None
        self.ax = None
                
    def _on_training_start(self) -> None:
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(6, 4))
        
        # Set the initial plot
        self.ax.set_title("Training Progress", fontsize=14)
        self.ax.set_xlabel("Episode", fontsize=12)
        self.ax.set_ylabel("Reward", fontsize=12)
        
        plt.show(block=False)
        plt.pause(0.1)
            
    def _on_step(self) -> bool:
        # Read the reward and done flag from locals
        reward = self.locals["rewards"]
        done = self.locals["dones"]
        
        if isinstance(done, (list, np.ndarray)):
            done = done[0]
            reward = reward[0]
                
        self.current_reward += reward
                
        if done:
            self.episode_count += 1
            self.episode_rewards.append(self.current_reward)
            
            # Compute moving average and std deviation
            if len(self.episode_rewards) >= self.window_size:
                avg = np.mean(self.episode_rewards[-self.window_size:])
                std = np.std(self.episode_rewards[-self.window_size:])
            else:
                avg = np.mean(self.episode_rewards)
                std = np.std(self.episode_rewards) if len(self.episode_rewards) > 1 else 0
                    
            self.moving_avg.append(avg)
            self.std_devs.append(std)
            
            # Update the plot every update_freq episodes
            if self.episode_count % self.update_freq == 0:
                self._update_plot()
                    
            # Debugging output
            if self.verbose and self.episode_count % max(1, self.update_freq) == 0:
                self.logger(f"Episode {self.episode_count} | " 
                            + f"Reward: {self.current_reward:.2f} | "
                            + f"Moving Avg ({self.window_size}): {avg:.2f}")
                
            # Reset
            self.current_reward = 0
                
        return True
            
    def _update_plot(self):
        self.ax.clear()
        episodes = list(range(1, len(self.episode_rewards) + 1))
        
        # Plot the episode rewards
        self.ax.plot(episodes, self.episode_rewards, 'b-', alpha=0.3, label="Episode Reward")
        
        # Plot the moving average
        self.ax.plot(episodes, self.moving_avg, 'r-', linewidth=2, label="Moving Average")
        
        # Plot the std deviation area
        upper_bound = np.array(self.moving_avg) + np.array(self.std_devs)
        lower_bound = np.array(self.moving_avg) - np.array(self.std_devs)
        self.ax.fill_between(episodes, 
                             lower_bound,
                             upper_bound,
                             color = 'red', 
                             alpha = 0.2,
                             label="±1 Std Dev")
        
        # Add grid and legend
        self.ax.legend(loc='upper left')
        self.ax.grid(True, linestyle='--', alpha=0.7)
        
        # Set the title and labels
        self.ax.set_title("Training Progress", fontsize=14)
        self.ax.set_xlabel("Episode", fontsize=12)
        self.ax.set_ylabel("Reward", fontsize=12)
        
        # Improve x-axis ticks for better readability
        if len(episodes) > 10:
            self.ax.set_xticks(np.linspace(1, len(episodes), min(10, len(episodes))).astype(int))
        
        # Update the plot
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        plt.pause(0.001)
            
    def _on_training_end(self) -> None:
        # Save the final plot
        plt.ioff()
        self._update_plot()
        plt.savefig("rl_training_progress.pdf", dpi=150, bbox_inches='tight')
        
        if self.verbose:
            self.logger.info("\nTraining complete. Final plots saved as 'rl_training_progress.png'")
            self.logger.info(f"Final Average Reward (last {self.window_size} episodes): {self.moving_avg[-1]:.2f}")
        
        plt.close(self.fig)
        