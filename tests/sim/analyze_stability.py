import numpy as np
import matplotlib.pyplot as plt

def analyze_stability(observations):
    # Convert lists to numpy arrays
    pos = np.array(observations['pos'])
    euler_ori = np.array(observations['euler_ori'])
    linear_vel = np.array(observations['linear_vel'])

    # Compute statistics for reduced factors
    metrics = {
        'roll_std': np.std(euler_ori[:, 0]),
        'pitch_std': np.std(euler_ori[:, 1]),
        'roll_range': np.max(euler_ori[:, 0]) - np.min(euler_ori[:, 0]),
        'pitch_range': np.max(euler_ori[:, 1]) - np.min(euler_ori[:, 1]),
        'roll_mean': np.mean(euler_ori[:, 0]),
        'pitch_mean': np.mean(euler_ori[:, 1]),

        'linear_vel_x_std': np.std(linear_vel[:, 0]),  # Forward velocity
        'linear_vel_z_std': np.std(linear_vel[:, 2]),  # Vertical velocity
        'linear_vel_x_range': np.max(linear_vel[:, 0]) - np.min(linear_vel[:, 0]),
        'linear_vel_z_range': np.max(linear_vel[:, 2]) - np.min(linear_vel[:, 2]),
        'linear_vel_x_mean': np.mean(linear_vel[:, 0]),
        'linear_vel_z_mean': np.mean(linear_vel[:, 2]),
    }

    # Print stability summary
    print("Stability Analysis (Standard Deviation over 3 seconds):")
    print(f"Roll Std:        {metrics['roll_std']:.4f} rad")
    print(f"Pitch Std:       {metrics['pitch_std']:.4f} rad")
    print(f"Roll Range:      {metrics['roll_range']:.4f} rad")
    print(f"Pitch Range:     {metrics['pitch_range']:.4f} rad")
    print(f"Roll Mean:       {metrics['roll_mean']:.4f} rad")
    print(f"Pitch Mean:      {metrics['pitch_mean']:.4f} rad")
    print(f"Linear Vx Std:   {metrics['linear_vel_x_std']:.4f} m/s")
    print(f"Linear Vz Std:   {metrics['linear_vel_z_std']:.4f} m/s")
    print(f"Linear Vx Range: {metrics['linear_vel_x_range']:.4f} m/s")
    print(f"Linear Vz Range: {metrics['linear_vel_z_range']:.4f} m/s")
    print(f"Linear Vx Mean:  {metrics['linear_vel_x_mean']:.4f} m/s")
    print(f"Linear Vz Mean:  {metrics['linear_vel_z_mean']:.4f} m/s")

    return metrics


def plot_stability(times, observations, metrics):
    pos = np.array(observations['pos'])
    euler_ori = np.array(observations['euler_ori'])
    linear_vel = np.array(observations['linear_vel'])

    # Plot 1: Orientation (Roll and Pitch only)
    plt.figure(figsize=(10, 4))
    plt.plot(times, euler_ori[:, 0], label=f"Roll (σ={metrics['roll_std']:.3f}, μ={metrics['roll_mean']:.3f}, Δ={metrics['roll_range']:.3f})")
    plt.plot(times, euler_ori[:, 1], label=f"Pitch (σ={metrics['pitch_std']:.3f}, μ={metrics['pitch_mean']:.3f}, Δ={metrics['pitch_range']:.3f})")
    plt.xlabel("Time (s)")
    plt.ylabel("Orientation (rad)")
    plt.title("Orientation Over Time")
    plt.ylim((-0.13, 0.13))
    plt.legend(loc = "lower right")
    plt.grid(True)
    plt.tight_layout()
    plt.show()

    # Plot 2: Linear Velocity (X and Z only)
    plt.figure(figsize=(10, 4))
    plt.plot(times, linear_vel[:, 0], label=f"Vx (σ={metrics['linear_vel_x_std']:.3f}, μ={metrics['linear_vel_x_mean']:.3f}, Δ={metrics['linear_vel_x_range']:.3f})")
    plt.plot(times, linear_vel[:, 2], label=f"Vz (σ={metrics['linear_vel_z_std']:.3f}, μ={metrics['linear_vel_z_mean']:.3f}, Δ={metrics['linear_vel_z_range']:.3f})")
    plt.xlabel("Time (s)")
    plt.ylabel("Linear Velocity (m/s)")
    plt.title("Linear Velocity Over Time")
    plt.ylim((-0.22, 0.35))
    plt.legend(loc = "lower right")
    plt.grid(True)
    plt.tight_layout()
    plt.show()
