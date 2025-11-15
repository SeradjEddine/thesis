import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os
import sys

# --- Get input and output directories from command line ---
if len(sys.argv) < 3:
    print("Usage: python plot_results.py <input_dir> <output_dir>")
    sys.exit(1)

input_dir = sys.argv[1]       # e.g., ../Data/sim_results_csv/city/oxts1058
output_dir = sys.argv[2]      # e.g., ../Data/sim_results_plot/city/oxts1058

# Create output directory if it doesn't exist
os.makedirs(output_dir, exist_ok=True)

# Load logs dynamically from input_dir
state_df = pd.read_csv(os.path.join(input_dir, "imu_prop.csv"))
gps_df   = pd.read_csv(os.path.join(input_dir, "gps_updates.csv"))
fuzzy_df = pd.read_csv(os.path.join(input_dir, "fuzzy.csv"))

print(f"Loaded {len(state_df)} state samples, {len(gps_df)} GPS updates, {len(fuzzy_df)} fuzzy entries")

# Helper function to save figures
def save_fig(fig, name):
    path = os.path.join(output_dir, name)
    fig.savefig(path)
    plt.close(fig)

# --- Trajectory (XY ENU) ---
fig = plt.figure(figsize=(8,6))
plt.plot(state_df["px"], state_df["py"], label="EKF trajectory", linewidth=2)

accepted = gps_df[gps_df["accepted_pos"] == 1]
rejected = gps_df[gps_df["accepted_pos"] == 0]

plt.scatter(accepted["zx"], accepted["zy"], c="green", s=20, label="GPS accepted")
plt.scatter(rejected["zx"], rejected["zy"], c="red", s=20, label="GPS rejected")

plt.xlabel("East (m)")
plt.ylabel("North (m)")
plt.title("Trajectory in ENU frame")
plt.legend()
plt.axis("equal")
plt.grid(True)
save_fig(fig, "trajectory.png")

# --- Mahalanobis distance plots in dB ---
# Avoid log of zero or negative values
mahal_pos_db = 10 * np.log10(np.clip(gps_df["mahalanobis_pos"].to_numpy(), 1e-12, None))
mahal_vel_db = 10 * np.log10(np.clip(gps_df["mahalanobis_vel"].to_numpy(), 1e-12, None))

fig = plt.figure(figsize=(10,4))
plt.plot(gps_df["t"], mahal_pos_db, label="Position Mahalanobis (dB)")
plt.axhline(10*np.log10(16.27), color="r", linestyle="--", label="3σ threshold (dB)")
plt.xlabel("Time (s)")
plt.ylabel("dB")
plt.title("GPS Position Mahalanobis Distance (dB)")
plt.legend()
plt.grid(True)
save_fig(fig, "Mahalanobis_pos_dB.png")

fig = plt.figure(figsize=(10,4))
plt.plot(gps_df["t"], mahal_vel_db, label="Velocity Mahalanobis (dB)")
plt.axhline(10*np.log10(16.27), color="r", linestyle="--", label="3σ threshold (dB)")
plt.xlabel("Time (s)")
plt.ylabel("dB")
plt.title("GPS Velocity Mahalanobis Distance (dB)")
plt.legend()
plt.grid(True)
save_fig(fig, "mahalanobis_vel_dB.png")


# --- Fuzzy scaling factors ---
fig = plt.figure(figsize=(10,5))
plt.plot(fuzzy_df["t"], fuzzy_df["scale_R_gps"], label="scale_R_gps", linewidth=2)
plt.plot(fuzzy_df["t"], fuzzy_df["scale_Q"], label="scale_Q", linewidth=2)
plt.plot(fuzzy_df["t"], fuzzy_df["scale_gate"], label="scale_gate", linewidth=2)
plt.xlabel("Time (s)")
plt.ylabel("Scaling factor")
plt.title("Fuzzy Supervisor Adaptive Scaling")
plt.legend()
plt.grid(True)
save_fig(fig, "fuzzy_scaling.png")

# --- Velocity plots (magnitude & components) ---
state_speed = np.sqrt(state_df["vx"]**2 + state_df["vy"]**2 + state_df["vz"]**2)

gps_vx = gps_df.get("vx", gps_df.get("vel_x", pd.Series(np.zeros(len(gps_df))))).to_numpy()
gps_vy = gps_df.get("vy", gps_df.get("vel_y", pd.Series(np.zeros(len(gps_df))))).to_numpy()
gps_vz = gps_df.get("vz", gps_df.get("vel_z", pd.Series(np.zeros(len(gps_df))))).to_numpy()
gps_speed = np.sqrt(gps_vx**2 + gps_vy**2 + gps_vz**2)

# Interpolate GPS speed to EKF times
state_times = state_df["t"].to_numpy()
gps_times = gps_df["t"].to_numpy()
sort_idx = np.argsort(gps_times)
interp_gps_speed = np.interp(state_times, gps_times[sort_idx], gps_speed[sort_idx])
interp_gps_vx = np.interp(state_times, gps_times[sort_idx], gps_vx[sort_idx])
interp_gps_vy = np.interp(state_times, gps_times[sort_idx], gps_vy[sort_idx])
interp_gps_vz = np.interp(state_times, gps_times[sort_idx], gps_vz[sort_idx])

fig = plt.figure(figsize=(10,4))
plt.plot(state_times, state_speed, label="EKF speed (|v|)", linewidth=1)
plt.plot(state_times, interp_gps_speed, label="GPS speed (interp)", linewidth=1, alpha=0.8)
plt.xlabel("Time (s)")
plt.ylabel("Speed (m/s)")
plt.title("EKF vs GPS Speed (magnitude)")
plt.legend()
plt.grid(True)
save_fig(fig, "velocity_speed.png")

fig, axes = plt.subplots(3,1, figsize=(12,8), sharex=True)
axes[0].plot(state_times, state_df["vx"], label="EKF vx (east)")
axes[0].plot(state_times, interp_gps_vx, label="GPS vx", alpha=0.8)
axes[0].set_ylabel("vx (m/s)")
axes[0].legend(); axes[0].grid(True)

axes[1].plot(state_times, state_df["vy"], label="EKF vy (north)")
axes[1].plot(state_times, interp_gps_vy, label="GPS vy", alpha=0.8)
axes[1].set_ylabel("vy (m/s)")
axes[1].legend(); axes[1].grid(True)

axes[2].plot(state_times, state_df["vz"], label="EKF vz (up)")
axes[2].plot(state_times, interp_gps_vz, label="GPS vz", alpha=0.8)
axes[2].set_ylabel("vz (m/s)")
axes[2].set_xlabel("Time (s)")
axes[2].legend(); axes[2].grid(True)

plt.suptitle("Velocity components: EKF vs GPS")
plt.tight_layout(rect=[0,0.03,1,0.97])
save_fig(fig, "velocity_components.png")

print(f"Saved all plots in {output_dir}")

