import math
from bagpy import bagreader
import matplotlib.pyplot as plt
import numpy as np
import seaborn as sns
import pandas as pd
import csv

plt.rcParams.update({'font.size': 24})
sns.set_style("dark")
sns.color_palette("viridis", as_cmap=True)

bag = bagreader("/home/ravina/catkin_ws/src/imu_gps/Data/imu_gps.bag")
data = bag.message_by_topic('/imu')
readings = pd.read_csv(data)

#This file only contains imu data collected in circles in front of Ruggles T station.
readings = readings[2000:12000]

plt.grid(color='grey', linestyle='--', linewidth=1)
plt.scatter(readings['mag_field.magnetic_field.x'], readings['mag_field.magnetic_field.y'], marker='.', label='Uncalibrated/Raw Data')
doughnut = plt.Circle((0.03, 0.22), 0.07, fill=False, color='black')
plt.gca().add_patch(doughnut)
plt.gca().set_aspect("equal")


#CALIBRATION
min_x = min(readings['mag_field.magnetic_field.x'])
max_x = max(readings['mag_field.magnetic_field.x'])
min_y = min(readings['mag_field.magnetic_field.y'])
max_y = max(readings['mag_field.magnetic_field.y'])

print("min_x = ", min_x)
print("max_x = ", max_x)
print("min_y = ", min_y)
print("max_y = ", max_y)

# HARD-IRON CALIBRATION

#Lets calculate Alpha = X axis offset
x_axis_Offset = (min_x + max_x)/2.0

#Lets calculate Beta = Y axis offset
y_axis_Offset = (min_y + max_y)/2.0

print("hard-iron x_axis_Offset=", x_axis_Offset)
print("hard-iron y_axis_Offset=", y_axis_Offset)

#These offsets are then subtracted from the raw x and y magnetometer data, thus largely eliminating the hard-iron distortion.
hard_iron_x = []
p = hard_iron_x.extend((readings['mag_field.magnetic_field.x']-x_axis_Offset))
hard_iron_y = []
q = hard_iron_y.extend((readings['mag_field.magnetic_field.y']-y_axis_Offset))

plt.grid(color='grey', linestyle='--', linewidth=1)
plt.scatter(hard_iron_x, hard_iron_y, marker='+', label='Hard-Iron Calibrated Data', color='crimson')
doughnut = plt.Circle((0.0, 0.0), 0.07, fill=False, color='black')
plt.gca().add_patch(doughnut)
plt.gca().set_aspect("equal")
plt.title('Hard_Iron_Calibration Plot Of Magnetic Field X vs Y')
plt.xlabel('Hard_Iron_X (Guass)')
plt.ylabel('Hard_Iron_Y (Guass)')
plt.legend()
plt.show()

print(np.shape(hard_iron_x))
print(np.shape(hard_iron_y))

X_major = float(hard_iron_x[2000])
Y_major = float(hard_iron_y[2000])

# SOFT-IRON CALIBRATION
radius = math.sqrt((X_major**2) + (Y_major**2))
print('radius = ', radius)
theta = np.arcsin((Y_major/radius))
print('theta = ', theta)

R = [[np.cos(theta), np.sin(theta)], [np.sin(-theta), np.cos(theta)]]
v = [hard_iron_x, hard_iron_y]

Rv_mat = np.matmul(R, v)
print(np.shape(Rv_mat))

plt.grid(color='grey', linestyle='--', linewidth=1)
plt.scatter(Rv_mat[1], Rv_mat[0], marker='x', label = 'Soft-Iron Calibrated', color='palevioletred')
# doughnut = plt.Circle((0.0, 0.0), 0.075, fill=False, color='black')
# plt.gca().add_patch(doughnut)
# plt.gca().set_aspect("equal")
plt.title('Soft_Iron_Calibration Of Magnetic Field X vs Y')
plt.xlabel('Soft_Iron_X (Guass)')
plt.ylabel('Soft_Iron_Y (Guass)')
plt.legend()
plt.show()

#Find Major and Minor axis using distance formula
r = 0.2
#r = radius
q = 0.15
sigma = q/r
print('sigma = ', sigma)

#Scaling
sigma_mat = [[1, 0], [0, sigma]]
rotate = np.matmul(sigma_mat, Rv_mat)
theta = -theta
R1 = [[np.cos(theta), np.sin(theta)], [np.sin(-theta), np.cos(theta)]]
v1 = np.matmul(R1, rotate)
print(np.shape(v1))
print("v1 = ", v1)
with open("v1_mat.csv", "w", newline="") as f:
    writer = csv.writer(f)
    for row in v1:
        writer.writerow(row)
        
plt.grid(color='grey', linestyle='--', linewidth=1)
plt.scatter(v1[1], v1[0], marker='x', label='Hard and Soft Iron Calibrated Data', color='indianred')
doughnut = plt.Circle((0.0, 0.0), 0.4, fill=False, color='black')
plt.gca().add_patch(doughnut)
plt.gca().set_aspect("equal")
plt.title('Final Calibrated Plot Of Magnetic Field X vs Y')
plt.xlabel('Mx (Guass)')
plt.ylabel('My (Guass)')
plt.rcParams.update({'font.size': 22})
plt.legend()
plt.show()