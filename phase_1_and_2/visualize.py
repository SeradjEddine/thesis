import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# Load CSV
df = pd.read_csv("imu_prop.csv")

# Position trajectory (x vs y)
plt.figure()
plt.plot(df['px'], df['py'])
plt.xlabel("x [m]")
plt.ylabel("y [m]")
plt.title("Estimated Trajectory (IMU only)")
plt.axis('equal')
plt.grid()

# Velocity magnitude
vel_mag = np.sqrt(df['vx']**2 + df['vy']**2 + df['vz']**2)
plt.figure()
plt.plot(df['t'], vel_mag)
plt.xlabel("time [s]")
plt.ylabel("Speed [m/s]")
plt.title("Velocity magnitude over time")
plt.grid()

# Quaternion norm check
quat_norm = np.sqrt(df['qw']**2 + df['qx']**2 + df['qy']**2 + df['qz']**2)
plt.figure()
plt.plot(df['t'], quat_norm)
plt.xlabel("time [s]")
plt.ylabel("||q||")
plt.title("Quaternion Norm (should stay ~1)")
plt.grid()

# Trace of covariance
plt.figure()
plt.plot(df['t'], df['traceP'])
plt.xlabel("time [s]")
plt.ylabel("Trace(P)")
plt.title("Covariance growth (uncertainty)")
plt.grid()

plt.show()

