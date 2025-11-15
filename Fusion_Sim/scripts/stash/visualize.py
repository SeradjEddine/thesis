import pandas as pd
import matplotlib.pyplot as plt

# --- Load logs ---
state_df = pd.read_csv("../build/proccessed_csv/imu_prop.csv")
gps_df = pd.read_csv("../build/proccessed_csv/gps_updates.csv")
fuzzy_df = pd.read_csv("../build/proccessed_csv/fuzzy.csv")  # NEW

print(f"Loaded {len(state_df)} state samples, {len(gps_df)} GPS updates, {len(fuzzy_df)} fuzzy entries")

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

plt.figure(figsize=(10,4))
plt.plot(gps_df["t"], gps_df["mahalanobis_vel"], label="Velocity Mahalanobis")
plt.axhline(16.27, color="r", linestyle="--", label="3σ threshold")
plt.xlabel("Time (s)")
plt.ylabel("d² value")
plt.title("GPS Velocity Mahalanobis Distance")
plt.legend()
plt.grid(True)
plt.savefig("../build/plots/mahalanobis_vel.png")

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

plt.show()

