import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os

# --- Load logs ---
state_df = pd.read_csv("../build/proccessed_csv/imu_prop.csv")
gps_df   = pd.read_csv("../build/proccessed_csv/gps_updates.csv")
fuzzy_df = pd.read_csv("../build/proccessed_csv/fuzzy.csv")  # NEW

print(f"Loaded {len(state_df)} state samples, {len(gps_df)} GPS updates, {len(fuzzy_df)} fuzzy entries")

os.makedirs("../build/plots", exist_ok=True)

# --- Trajectory (XY ENU) ---
plt.figure(figsize=(8,6))
plt.plot(state_df["px"], state_df["py"], label="EKF trajectory", linewidth=2)

# Separate accepted/rejected GPS updates
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
plt.savefig("../build/plots/trajectory.png")
plt.close()

# --- Mahalanobis distances ---
plt.figure(figsize=(10,4))
plt.plot(gps_df["t"], gps_df["mahalanobis_pos"], label="Position Mahalanobis")
plt.axhline(16.27, color="r", linestyle="--", label="3σ threshold")
plt.xlabel("Time (s)")
plt.ylabel("d² value")
plt.title("GPS Position Mahalanobis Distance")
plt.legend()
plt.grid(True)
plt.savefig("../build/plots/Mahalanobis_pos.png")
plt.close()

plt.figure(figsize=(10,4))
plt.plot(gps_df["t"], gps_df["mahalanobis_vel"], label="Velocity Mahalanobis")
plt.axhline(16.27, color="r", linestyle="--", label="3σ threshold")
plt.xlabel("Time (s)")
plt.ylabel("d² value")
plt.title("GPS Velocity Mahalanobis Distance")
plt.legend()
plt.grid(True)
plt.savefig("../build/plots/mahalanobis_vel.png")
plt.close()

# --- Fuzzy scaling factors ---
plt.figure(figsize=(10,5))
plt.plot(fuzzy_df["t"], fuzzy_df["scale_R_gps"], label="scale_R_gps", linewidth=2)
plt.plot(fuzzy_df["t"], fuzzy_df["scale_Q"], label="scale_Q", linewidth=2)
plt.plot(fuzzy_df["t"], fuzzy_df["scale_gate"], label="scale_gate", linewidth=2)
plt.xlabel("Time (s)")
plt.ylabel("Scaling factor")
plt.title("Fuzzy Supervisor Adaptive Scaling")
plt.legend()
plt.grid(True)
plt.savefig("../build/plots/fuzzy_scaling.png")
plt.close()

# ----------------------------
# New: Velocity plots (m/s)
# ----------------------------
# Compute EKF state speed magnitude
state_vx = state_df["vx"].to_numpy()
state_vy = state_df["vy"].to_numpy()
state_vz = state_df["vz"].to_numpy()
state_speed = np.sqrt(state_vx**2 + state_vy**2 + state_vz**2)

# Compute GPS speed magnitude from gps log (vx,vy,vz are logged in gps_updates.csv)
# If gps columns use different names, adapt accordingly.
if {"vx","vy","vz"}.issubset(gps_df.columns):
    gps_vx = gps_df["vx"].to_numpy()
    gps_vy = gps_df["vy"].to_numpy()
    gps_vz = gps_df["vz"].to_numpy()
else:
    # Try alternative column names if present
    gps_vx = gps_df.get("vel_x", pd.Series(np.zeros(len(gps_df)))).to_numpy()
    gps_vy = gps_df.get("vel_y", pd.Series(np.zeros(len(gps_df)))).to_numpy()
    gps_vz = gps_df.get("vel_z", pd.Series(np.zeros(len(gps_df)))).to_numpy()

gps_speed = np.sqrt(gps_vx**2 + gps_vy**2 + gps_vz**2)

# Interpolate GPS speed to EKF state timestamps for direct comparison
state_times = state_df["t"].to_numpy()
gps_times = gps_df["t"].to_numpy()

# Ensure times are sorted (they should be)
sort_idx = np.argsort(gps_times)
gps_times_sorted = gps_times[sort_idx]
gps_speed_sorted = gps_speed[sort_idx]
gps_vx_sorted = gps_vx[sort_idx]
gps_vy_sorted = gps_vy[sort_idx]
gps_vz_sorted = gps_vz[sort_idx]

# Interpolate with np.interp (extrapolate with edge values)
interp_gps_speed = np.interp(state_times, gps_times_sorted, gps_speed_sorted, left=gps_speed_sorted[0], right=gps_speed_sorted[-1])
interp_gps_vx = np.interp(state_times, gps_times_sorted, gps_vx_sorted, left=gps_vx_sorted[0], right=gps_vx_sorted[-1])
interp_gps_vy = np.interp(state_times, gps_times_sorted, gps_vy_sorted, left=gps_vy_sorted[0], right=gps_vy_sorted[-1])
interp_gps_vz = np.interp(state_times, gps_times_sorted, gps_vz_sorted, left=gps_vz_sorted[0], right=gps_vz_sorted[-1])

# Plot speed magnitudes
plt.figure(figsize=(10,4))
plt.plot(state_times, state_speed, label="EKF speed (|v|)", linewidth=1)
plt.plot(state_times, interp_gps_speed, label="GPS speed (interp)", linewidth=1, alpha=0.8)
plt.xlabel("Time (s)")
plt.ylabel("Speed (m/s)")
plt.title("EKF vs GPS Speed (magnitude)")
plt.legend()
plt.grid(True)
plt.savefig("../build/plots/velocity_speed.png")
plt.close()

# Plot velocity components (East, North, Up) — EKF vs interpolated GPS
plt.figure(figsize=(12,8))
ax1 = plt.subplot(3,1,1)
ax1.plot(state_times, state_vx, label="EKF vx (east)", linewidth=1)
ax1.plot(state_times, interp_gps_vx, label="GPS vx (interp)", linewidth=1, alpha=0.8)
ax1.set_ylabel("vx (m/s)")
ax1.legend(); ax1.grid(True)

ax2 = plt.subplot(3,1,2)
ax2.plot(state_times, state_vy, label="EKF vy (north)", linewidth=1)
ax2.plot(state_times, interp_gps_vy, label="GPS vy (interp)", linewidth=1, alpha=0.8)
ax2.set_ylabel("vy (m/s)")
ax2.legend(); ax2.grid(True)

ax3 = plt.subplot(3,1,3)
ax3.plot(state_times, state_vz, label="EKF vz (up)", linewidth=1)
ax3.plot(state_times, interp_gps_vz, label="GPS vz (interp)", linewidth=1, alpha=0.8)
ax3.set_ylabel("vz (m/s)")
ax3.set_xlabel("Time (s)")
ax3.legend(); ax3.grid(True)

plt.suptitle("Velocity components: EKF vs GPS (interpolated to EKF times)")
plt.tight_layout(rect=[0, 0.03, 1, 0.97])
plt.savefig("../build/plots/velocity_components.png")
plt.close()

print("Saved velocity plots: ../build/plots/velocity_speed.png and velocity_components.png")

# Show all figures (optional)
plt.show()
