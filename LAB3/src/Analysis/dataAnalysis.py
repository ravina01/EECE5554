import statistics
from bagpy import bagreader
import matplotlib.pyplot as plt
import numpy as np
import seaborn as sns
import pandas as pd
from scipy.stats import norm

plt.rcParams.update({'font.size': 16})

#Stationary Data
#path = "/home/ravina/catkin_ws/src/imu_driver/Data/stationary/2023-03-03-20-30-09.bag"
#path = "/home/ravina/Desktop/LAB3_data/stationary/March10/2023-03-10-20-05-46.bag"
path = "/home/ravina/catkin_ws/src/imu_driver/Data/stationary/March10/2023-03-10-20-05-46.bag"

bag = bagreader(path)
data = bag.message_by_topic('/imu')
readings = pd.read_csv(data)

w = np.array(readings['IMU.orientation.w']) * (np.pi/180)
x = np.array(readings['IMU.orientation.x']) * (np.pi/180)
y = np.array(readings['IMU.orientation.y']) * (np.pi/180)
z = np.array(readings['IMU.orientation.z']) * (np.pi/180)
#print(w, readings['IMU.orientation.w'])

time = np.array(readings['Time'])
#LINE_GRAPHS
f, ax = plt.subplots(3, 1, figsize=(30, 18))
f.subplots_adjust(hspace=0.4)
ax[0].plot(time, np.array(readings['IMU.angular_velocity.x']))
ax[1].plot(time, np.array(readings['IMU.angular_velocity.y']))
ax[2].plot(time, np.array(readings['IMU.angular_velocity.z']))
ax[0].set_xlabel('Time (Seconds)')
ax[0].set_ylabel('Angular Velocity_X (rad/sec)')
ax[0].set_title('Time vs Angular Velocity_X')
ax[1].set_xlabel('Time (Seconds)')
ax[1].set_ylabel('Angular Velocity_Y (rad/sec)')
ax[1].set_title('Time vs Angular Velocity_Y')
ax[2].set_xlabel('Time (Seconds)')
ax[2].set_ylabel('Angular Velocity_Z (rad/sec)')
ax[2].set_title('Time vs Angular Velocity_Z')

# print("Linear acc = ")
# print(readings['IMU.linear_acceleration.z'])

f, ax = plt.subplots(3, 1, figsize=(30, 18))
f.subplots_adjust(hspace=0.4)
ax[0].plot(time, np.array(readings['IMU.linear_acceleration.x']))
ax[1].plot(time, np.array(readings['IMU.linear_acceleration.y']))
ax[2].plot(time, np.array(readings['IMU.linear_acceleration.z']))
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
f.subplots_adjust(hspace=0.4)
ax[0].plot(time, roll_x, label = 'Time VS roll_x')
ax[1].plot(time, pitch_y, label = 'Time VS pitch_y')
ax[2].plot(time, yaw_z, label = 'Time VS yaw_z')
ax[0].set_xlabel('Time (Seconds)')
ax[0].set_ylabel('roll_x (degrees)')
ax[0].set_title('Time vs roll_x')
ax[1].set_xlabel('Time (Seconds)')
ax[1].set_ylabel('pitch_y (degrees)')
ax[1].set_title('Time vs pitch_y')
ax[2].set_xlabel('Time (Seconds)')
ax[2].set_ylabel('yaw_z (degrees)')
ax[2].set_title('Time vs yaw_z', fontsize=20)


#HISTOGRAMS
f, ax = plt.subplots(3, 1, figsize=(30, 18))
f.subplots_adjust(hspace=0.4)
ax[0].hist(roll_x, bins= 40)
ax[1].hist(pitch_y, bins= 40)
ax[2].hist(yaw_z, bins= 40)
ax[0].set_xlabel('roll_x (degrees)')
ax[0].set_ylabel('Frequency')
ax[0].set_title('roll_x vs Frequency')
ax[1].set_xlabel('pitch_y (degrees)')
ax[1].set_ylabel('Frequency')
ax[1].set_title('pitch_y vs Frequency')
ax[2].set_xlabel('yaw_z')
ax[2].set_ylabel('Frequency')
ax[2].set_title('yaw_z vs Frequency')

f, ax = plt.subplots(3, 1, figsize=(30, 18))
f.subplots_adjust(hspace=0.4)

n, binsX, patches = ax[0].hist(readings['IMU.angular_velocity.x'], bins= 40, density=True)
n, binsY, patches = ax[1].hist(readings['IMU.angular_velocity.y'], bins= 40, density=True)
n, binsZ, patches = ax[2].hist(readings['IMU.angular_velocity.z'], bins= 40, density=True)

mu_x, std_x = norm.fit(np.array(readings['IMU.angular_velocity.x']))
mu_y, std_y = norm.fit(np.array(readings['IMU.angular_velocity.y']))
mu_z, std_z = norm.fit(np.array(readings['IMU.angular_velocity.z']))

p_x = norm.pdf(binsX, mu_x, std_x)
p_y = norm.pdf(binsY, mu_y, std_y)
p_z = norm.pdf(binsZ, mu_z, std_z)

ax[0].plot(binsX, p_x, 'r--', linewidth=2)
ax[1].plot(binsY, p_y, 'r--', linewidth=2)
ax[2].plot(binsZ, p_z, 'r--', linewidth=2)

ax[0].set_xlabel('Angular Velocity_X (rad/sec)')
ax[0].set_ylabel('Frequency')
ax[0].set_title('Angular Velocity_X (rad/sec) vs Frequency')
ax[1].set_xlabel('Angular Velocity_Y (rad/sec)')
ax[1].set_ylabel('Frequency')
ax[1].set_title('Angular Velocity_Y (rad/sec) vs Frequency')
ax[2].set_xlabel('Angular Velocity_Z (rad/sec)')
ax[2].set_ylabel('Frequency')
ax[2].set_title('Angular Velocity_Z (rad/sec) vs Frequency')

f, ax = plt.subplots(3, 1, figsize=(30, 18))
f.subplots_adjust(hspace=0.4)

n, binsX, patches = ax[0].hist(readings['IMU.linear_acceleration.x'], bins= 40, density=True)
n, binsY, patches = ax[1].hist(readings['IMU.linear_acceleration.y'], bins= 40, density=True)
n, binsZ, patches = ax[2].hist(readings['IMU.linear_acceleration.z'], bins= 40, density=True)

mu_x, std_x = norm.fit(np.array(readings['IMU.linear_acceleration.x']))
mu_y, std_y = norm.fit(np.array(readings['IMU.linear_acceleration.y']))
mu_z, std_z = norm.fit(np.array(readings['IMU.linear_acceleration.z']))

p_x = norm.pdf(binsX, mu_x, std_x)
#p_x = p_x / np.diff(binsX)[0]
p_y = norm.pdf(binsY, mu_y, std_y)
#p_y = p_y / np.diff(binsY)[0]
p_z = norm.pdf(binsZ, mu_z, std_z)
#p_z = p_z / np.diff(binsZ)[0]

ax[0].plot(binsX, p_x, 'r--', linewidth=2)
ax[1].plot(binsY, p_y, 'r--', linewidth=2)
ax[2].plot(binsZ, p_z, 'r--', linewidth=2)

ax[0].set_xlabel('Linear Acceleration_X (m/s\u00b2))')
ax[0].set_ylabel('Frequency')
ax[0].set_title('Linear Acceleration_X (m/s\u00b2) vs Frequency')
ax[1].set_xlabel('Linear Acceleration_Y (m/s\u00b2))')
ax[1].set_ylabel('Frequency')
ax[1].set_title('Linear Acceleration_Y (m/s\u00b2) vs Frequency')
ax[2].set_xlabel('Linear Acceleration_Z (m/s\u00b2)')
ax[2].set_ylabel('Frequency')
ax[2].set_title('Linear Acceleration_Z (m/s\u00b2) vs Frequency')


plt.rcParams.update({'font.size': 22})
plt.show()

#def euler_from_quaternion(x, y, z, w):
t0 = +2.0 * (w * x + y * z)
t1 = +1.0 - 2.0 * (x * x + y *y)
roll_x = np.degrees(np.arctan2(t0, t1))

t2 = +2.0 * (w * y - z * x)
t2 = np.where(t2>+1.0, +1.0,t2)
t2 = np.where(t2<-1.0, -1.0,t2)
pitch_y = np.degrees(np.arcsin(t2))

t3 = +2.0 * (w * z + x * y)
t4 = +1.0 - 2.0 * (y * y+ z * z)
yaw_z = np.degrees(np.arctan2(t3, t4))

#MEAN CALCULATION OF RPY
print('Mean & Standard Deviation of RPY:')
print('mean = ',statistics.mean(roll_x))
print('mean = ',statistics.mean(pitch_y))
print('mean = ',statistics.mean(yaw_z))
print('standard deviation = ',statistics.stdev(roll_x))
print('standard deviation = ',statistics.stdev(pitch_y))
print('standard deviation = ',statistics.stdev(yaw_z))

#MEAN CALCULATION OF ANGULAR VELOCITY
print('Mean & Standard Deviation of Angular Velocity:')
for i in ['IMU.angular_velocity.x', 'IMU.angular_velocity.y', 'IMU.angular_velocity.z']:
    print('mean = ',np.array(readings[i]).mean())
    print('standard deviation = ',np.array(readings[i]).std())


#MEAN CALCULATION OF LINEAR ACCELERATION
print('Mean & Standard Deviation of Linear Acceleration X:')
for i in ['IMU.linear_acceleration.x']:
    print('mean = ',np.array(readings[i]).mean())
    print('standard deviation = ',np.array(readings[i]).std())
    print('Median = ',np.median(np.array(readings[i])))

#MEAN CALCULATION OF MAGNETIC FIELD
print('Mean & Standard Deviation of Magnetic Field:')
for i in ['MagField.magnetic_field.x', 'MagField.magnetic_field.y', 'MagField.magnetic_field.z']:
    print('mean = ',np.array(readings[i]).mean())
    print('standard deviation = ',np.array(readings[i]).std())