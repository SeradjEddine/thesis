import os
import pandas as pd
import matplotlib.pyplot as plt

# --- Paths ---
script_dir = os.path.dirname(os.path.abspath(__file__))
state_csv = os.path.join(script_dir, "../build/proccessed_csv/imu_prop.csv")
gps_csv   = os.path.join(script_dir, "../build/proccessed_csv/gps_updates.csv")
fuzzy_csv = os.path.join(script_dir, "../build/proccessed_csv/fuzzy.csv")

# --- Load CSVs ---
state_df = pd.read_csv(state_csv)
gps_df   = pd.read_csv(gps_csv)
fuzzy_df = pd.read_csv(fuzzy_csv)

print(f"Loaded {len(state_df)} state samples, {len(gps_df)} GPS updates, {len(fuzzy_df)} fuzzy entries")

# --- Convert to NumPy arrays ---
px = state_df["px"].to_numpy()
py = state_df["py"].to_numpy()

gps_x = gps_df["zx"].to_numpy()
gps_y = gps_df["zy"].to_numpy()

fuzzy_time  = fuzzy_df["t"].to_numpy()
fuzzy_scale = fuzzy_df["scale_R_gps"].to_numpy()  # adjust if your fuzzy CSV column differs

# --- Plot EKF trajectory vs GPS ---
plt.figure(figsize=(10, 6))
plt.plot(px, py, label="EKF trajectory", linewidth=2)
plt.scatter(gps_x, gps_y, c='r', s=10, label="GPS updates")
plt.xlabel("X [m]")
plt.ylabel("Y [m]")
plt.title("EKF vs GPS")
plt.legend()
plt.grid(True)


# --- Plot fuzzy supervisor scaling ---
plt.figure(figsize=(10, 4))
plt.plot(fuzzy_time, fuzzy_scale, label="Fuzzy R scaling")
plt.xlabel("Time [s]")
plt.ylabel("Scale")
plt.title("Fuzzy Supervisor Output")
plt.grid(True)
plt.legend()


# --- Mahalanobis distances ---
t_pos   = gps_df["t"].to_numpy()
maha_pos = gps_df["mahalanobis_pos"].to_numpy()
maha_vel = gps_df["mahalanobis_vel"].to_numpy()

plt.figure(figsize=(10,4))
plt.plot(t_pos, maha_pos, label="Position Mahalanobis")
plt.axhline(16.27, color="r", linestyle="--", label="3σ threshold")
plt.xlabel("Time [s]")
plt.ylabel("d² value")
plt.title("GPS Position Mahalanobis Distance")
plt.legend()
plt.grid(True)


plt.figure(figsize=(10,4))
plt.plot(t_pos, maha_vel, label="Velocity Mahalanobis")
plt.axhline(16.27, color="r", linestyle="--", label="3σ threshold")
plt.xlabel("Time [s]")
plt.ylabel("d² value")
plt.title("GPS Velocity Mahalanobis Distance")
plt.legend()
plt.grid(True)



plt.show()
