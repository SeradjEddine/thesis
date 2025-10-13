import csv
import matplotlib.pyplot as plt

# Read VO samples
tsv = "visual_samples.csv"
tx, ty = [], []

with open(tsv, "r") as f:
    reader = csv.DictReader(f)
    for row in reader:
        tx.append(float(row['tx']))
        ty.append(float(row['ty']))

# Plot 2D trajectory (top-down view)
plt.figure(figsize=(8,8))
plt.plot(tx, ty, marker='o', markersize=3, linestyle='-', color='blue')
plt.xlabel("X (meters)")
plt.ylabel("Y (meters)")
plt.title("2D Visual Odometry Trajectory (Top-Down View)")
plt.axis('equal')  # Equal scaling for X/Y axes
plt.grid(True)
plt.show()
