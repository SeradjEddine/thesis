import csv
import numpy as np
import matplotlib.pyplot as plt

csv_file = "visual_samples.csv"

# Load CSV
t, tx, ty, tz = [], [], [], []
with open(csv_file, 'r') as f:
    reader = csv.DictReader(f)
    for row in reader:
        t.append(float(row['t']))
        tx.append(float(row['tx']))
        ty.append(float(row['ty']))
        tz.append(float(row['tz']))

t = np.array(t)
tx = np.array(tx)
ty = np.array(ty)
tz = np.array(tz)

# Plot trajectory (3D)
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot(tx, ty, tz, label='VO trajectory', marker='o')
ax.set_xlabel('X [m]')
ax.set_ylabel('Y [m]')
ax.set_zlabel('Z [m]')
ax.legend()
ax.set_title("Visual Odometry Trajectory")
plt.show()

# Plot each component vs time
plt.figure()
plt.plot(t, tx, label='tx')
plt.plot(t, ty, label='ty')
plt.plot(t, tz, label='tz')
plt.xlabel('Time [s]')
plt.ylabel('Position [m]')
plt.title('VO Translation vs Time')
plt.legend()
plt.show()
