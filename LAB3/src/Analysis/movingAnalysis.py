import statistics
from bagpy import bagreader
import matplotlib.pyplot as plt
import numpy as np
import seaborn as sns
import pandas as pd
plt.rcParams.update({'font.size': 16})

#Moving Data
path = "/home/ravina/catkin_ws/src/imu_driver/Data/moving/2023-03-03-20-43-52.bag"

bag = bagreader(path)
data = bag.message_by_topic('/imu')
readings = pd.read_csv(data)

print("\n")
time = np.array(readings['Time'])

#LINE_GRAPHS

linear_acc_X = np.array(readings['IMU.linear_acceleration.x'])
linear_acc_Y = np.array(readings['IMU.linear_acceleration.y'])
linear_acc_Z = np.array(readings['IMU.linear_acceleration.z'])

angular_acc_X = np.array(readings['IMU.angular_velocity.x'])
angular_acc_Y = np.array(readings['IMU.angular_velocity.y'])
angular_acc_Z = np.array(readings['IMU.angular_velocity.z'])

#Along Y axis : 10 seconds
motion_1_X = linear_acc_X[:400].astype(int)
motion_1_Y = linear_acc_Y[:400].astype(int)
motion_1_Z = linear_acc_Z[:400].astype(int)

#Along Z axis : 17 to 27 seconds 
motion_2_X = linear_acc_X[680:1080].astype(int)
motion_2_Y = linear_acc_Y[680:1080].astype(int)
motion_2_Z = linear_acc_Z[680:1080].astype(int)

#Along X axis : 40 to 48 seconds
motion_3_X = linear_acc_X[1600:1920]
motion_3_Y = linear_acc_Y[1600:1920]
motion_3_Z = linear_acc_Z[1600:1920]

angle_1_X = angular_acc_X[:400].astype(int)
angle_1_Y = angular_acc_Y[:400].astype(int)
angle_1_Z = angular_acc_Z[:400].astype(int)

angle_2_X = angular_acc_X[680:1080].astype(int)
angle_2_Y = angular_acc_Y[680:1080].astype(int)
angle_2_Z = angular_acc_Z[680:1080].astype(int)

angle_3_X = angular_acc_X[1600:1920].astype(int)
angle_3_Y = angular_acc_Y[1600:1920].astype(int)
angle_3_Z = angular_acc_Z[1600:1920].astype(int)

time1 = np.arange(0, time.size, 1, dtype=int)
time_slot1 = time1[:400]
time_slot2 = time1[680:1080]
time_slot3 = time1[1600:1920]

f, ax = plt.subplots(3, 1, figsize=(30, 18))
f.subplots_adjust(hspace=0.5)
ax[0].plot(time_slot1, motion_1_X)
ax[1].plot(time_slot1, motion_1_Y)
ax[2].plot(time_slot1, motion_1_Z)
ax[0].set_xlabel('Time (Seconds)')
ax[0].set_ylabel('Linear Acceleration_X (m/s\u00b2)')
ax[0].set_title('Time vs Linear Acceleration_X')
ax[1].set_xlabel('Time (Seconds)')
ax[1].set_ylabel('Linear Acceleration_Y (m/s\u00b2)')
ax[1].set_title('Time vs Linear Acceleration_Y')
ax[2].set_xlabel('Time (Seconds)')
ax[2].set_ylabel('Linear Acceleration_Z (m/s\u00b2)')
ax[2].set_title('Time vs Linear Acceleration_Z')


f, ax = plt.subplots(3, 1, figsize=(30, 18))
f.subplots_adjust(hspace=0.5)
ax[0].plot(time_slot1, angle_1_X)
ax[1].plot(time_slot1, angle_1_Y)
ax[2].plot(time_slot1, angle_1_Z)
ax[0].set_xlabel('Time (Seconds)')
ax[0].set_ylabel('Angular Velocity_X(m/s\u00b2)')
ax[0].set_title('Time vs Angular Velocity_X')
ax[1].set_xlabel('Time (Seconds)')
ax[1].set_ylabel('Angular Velocity_Y (m/s\u00b2)')
ax[1].set_title('Time vs Angular Velocity_Y')
ax[2].set_xlabel('Time (Seconds)')
ax[2].set_ylabel('Angular Velocity_Y (m/s\u00b2)')
ax[2].set_title('Time vs Angular Velocity_Z')


f, ax = plt.subplots(3, 1, figsize=(30, 18))
f.subplots_adjust(hspace=0.5)
ax[0].plot(time_slot2, angle_2_X)
ax[1].plot(time_slot2, angle_2_Y)
ax[2].plot(time_slot2, angle_2_Z)
ax[0].set_xlabel('Time (Seconds)')
ax[0].set_ylabel('Angular Velocity_X(m/s\u00b2)')
ax[0].set_title('Time vs Angular Velocity_X')
ax[1].set_xlabel('Time (Seconds)')
ax[1].set_ylabel('Angular Velocity_Y (m/s\u00b2)')
ax[1].set_title('Time vs Angular Velocity_Y')
ax[2].set_xlabel('Time (Seconds)')
ax[2].set_ylabel('Angular Velocity_Y (m/s\u00b2)')
ax[2].set_title('Time vs Angular Velocity_Z')

f, ax = plt.subplots(3, 1, figsize=(30, 18))
f.subplots_adjust(hspace=0.5)
ax[0].plot(time_slot3, angle_3_X)
ax[1].plot(time_slot3, angle_3_Y)
ax[2].plot(time_slot3, angle_3_Z)
ax[0].set_xlabel('Time (Seconds)')
ax[0].set_ylabel('Angular Velocity_X(m/s\u00b2)')
ax[0].set_title('Time vs Angular Velocity_X')
ax[1].set_xlabel('Time (Seconds)')
ax[1].set_ylabel('Angular Velocity_Y (m/s\u00b2)')
ax[1].set_title('Time vs Angular Velocity_Y')
ax[2].set_xlabel('Time (Seconds)')
ax[2].set_ylabel('Angular Velocity_Y (m/s\u00b2)')
ax[2].set_title('Time vs Angular Velocity_Z')

plt.rcParams.update({'font.size': 20})
plt.show()