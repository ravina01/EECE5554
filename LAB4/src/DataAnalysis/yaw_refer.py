import math
from bagpy import bagreader
import matplotlib.pyplot as plt
import numpy as np
import seaborn as sns
import pandas as pd
import scipy.integrate as integrate
from scipy.signal import butter
from scipy import signal

sns.set_style("dark")
sns.color_palette("viridis", as_cmap=True)
plt.rcParams.update({'font.size': 12})

bag = bagreader("/home/ravina/catkin_ws/src/imu_gps/Data/imu_gps.bag")
data = bag.message_by_topic('/imu')
readings = pd.read_csv(data)

readings = readings[10000:101000]
#readings = readings[52000:82000]
# CALIBRATION
min_x = min(readings['mag_field.magnetic_field.x'])
max_x = max(readings['mag_field.magnetic_field.x'])
min_y = min(readings['mag_field.magnetic_field.y'])
max_y = max(readings['mag_field.magnetic_field.y'])

mag_x = readings['mag_field.magnetic_field.x']
mag_y = readings['mag_field.magnetic_field.y']

# HARD-IRON CALIBRATION
x_axis_Offset = (min_x + max_x)/2.0
y_axis_Offset = (min_y + max_y)/2.0

print("hard-iron x_axis_Offset=", x_axis_Offset)
print("hard-iron y_axis_Offset=", y_axis_Offset)
hard_iron_x = []
p = hard_iron_x.extend((readings['mag_field.magnetic_field.x']-x_axis_Offset))
hard_iron_y = []
q = hard_iron_y.extend((readings['mag_field.magnetic_field.y']-y_axis_Offset))

mag_x_corrected = mag_x - x_axis_Offset
mag_y_corrected = mag_y - y_axis_Offset

yaw_raw = np.arctan2(mag_y_corrected, mag_x_corrected)

# SOFT-IRON CALIBRATION
radius = np.sqrt((mag_x_corrected**2) + (mag_y_corrected**2))

r = max(radius)
q = min(radius)
print('radius = ', radius)
theta = np.arcsin(mag_y_corrected[np.argmax(radius)]/r)
print('theta = ', theta)

R = np.array([[np.cos(theta), np.sin(theta)], [np.sin(-theta), np.cos(theta)]])
#v = [hard_iron_x, hard_iron_y]
v = [mag_x_corrected, mag_y_corrected]

v = R@v


# matrix = np.matmul(R, v)
# print(np.shape(matrix))

# # Find Major and Minor axis using distance formula
# r = max(radius) #0.2
# q = min(radius) #0.15
sigma = q/r
print('sigma = ', sigma)

# Scaling
# matrix2 = [[1, 0], [0, sigma]]

# rotate = np.matmul(matrix2, matrix)
theta = -theta
R1 = [[np.cos(theta), np.sin(theta)], [np.sin(-theta), np.cos(theta)]]

r_sigma = v[0]*sigma
v1 = [r_sigma, v[1]]

corrected_v1 = np.matmul(R1, v1)#R1@v1

mag_x_corr = corrected_v1[0]
mag_y_corr = corrected_v1[1]

corrected_yaw = np.arctan2(mag_y_corr, mag_x_corr)
corrected_yaw[15323:34862] = corrected_yaw[15323:34862] + 5.7
corrected_yaw[48534:61191] = corrected_yaw[48534:61191] + 5.7

time_sec = readings['Header.stamp.secs']
time_nsec = readings['Header.stamp.nsecs']
time = time_sec + (time_nsec/1000000000)

gyro_int=[]
gyro_int[0:22000] = integrate.cumtrapz(readings['imu.angular_velocity.z'][0:22000], time[0:22000], initial=0)*(-1)
gyro_int[22000:44000] =  ( integrate.cumtrapz(readings['imu.angular_velocity.z'][22000:44000], time[22000:44000], initial=gyro_int[21999])*(-1))
gyro_int[44000:66000] = integrate.cumtrapz(readings['imu.angular_velocity.z'][44000:66000], time[44000:66000], initial=gyro_int[43999])*(-1)
gyro_int[66000:] = integrate.cumtrapz(readings['imu.angular_velocity.z'][66000:], time[66000:], initial=gyro_int[65999])*(-1)

gyro_int_wrap = np.unwrap(gyro_int)
gyro_scaled = gyro_int_wrap
plt.plot(time, gyro_int_wrap, label = "yaw from gyro",  c='palevioletred')
plt.plot(time, corrected_yaw, label = "Corrected Yaw", c='lightseagreen')
plt.plot(time, yaw_raw, label = "Raw Yaw")
plt.legend(loc='upper right', fontsize='x-large')
plt.grid(color='grey', linestyle='--', linewidth=1)
plt.title('Estimation of Yaw for Magnetometer')
plt.xlabel('time')
plt.ylabel('Yaw (radians)')
plt.show()
#plt.plot(time, ya)


# YAW CALCULATION
w = readings['imu.orientation.w']
x = readings['imu.orientation.x']
y = readings['imu.orientation.y']
z = readings['imu.orientation.z']


# Euler from Quaternion(x, y, z, w):
t0 = +2.0 * (w * x + y * z)
t1 = +1.0 - 2.0 * (x * x + y * y)
roll_x = np.arctan2(t0, t1)

t2 = +2.0 * (w * y - z * x)
pitch_y = np.arcsin(t2)

t3 = +2.0 * (w * z + x * y)
t4 = +1.0 - 2.0 * (y * y + z * z)
yaw_z = np.arctan2(t3, t4)

roll = roll_x
print('roll', roll)
pitch = pitch_y
yaw = yaw_z

mag_z1 = readings['mag_field.magnetic_field.z']
data_x = readings['mag_field.magnetic_field.x']
data_y = readings['mag_field.magnetic_field.y']
data_z = readings['mag_field.magnetic_field.z']
size = len(data_z)
print("size = ",size)

# Filteration
lpf = signal.filtfilt(*butter(1, 0.05, "lowpass",fs = 40, analog=False), corrected_yaw)
hpf = signal.filtfilt(*butter(1, 0.01, 'highpass', fs = 40, analog=False), gyro_int_wrap)
#print(lpf[0]/10+hpf[0]/10)
plt.figure(figsize = (16,8))
plt.plot(time, lpf, label='LPF Calibrated Yaw')
plt.legend(loc='upper right', fontsize='x-large')
plt.grid(color='grey', linestyle='--', linewidth=1)
plt.plot(time, hpf, label = 'HPF Gyro Yaw', c='seagreen')
plt.legend(loc='upper right', fontsize='x-large')
plt.grid(color='grey', linestyle='--', linewidth=1)
plt.title('LPF for Magnetic Yaw and HPF for Gyro Yaw')
plt.xlabel('Time')
plt.ylabel('Yaw (radians)')
plt.show()

#Original Yaw V/S Calibrated Yaw
alpha = 0.5
#omega = readings['imu.angular_velocity.z']
yaw_filtered = np.zeros(len(corrected_yaw))
#yaw_filtered = np.append(yaw_filtered,0)
for i in range(len(corrected_yaw)):
  yaw_filtered[i] = alpha*(yaw_filtered[i-1] + gyro_int[i]) + ((1-alpha)*lpf[i])
# lpf1 = 1 - hpf1
# yaw_filtered = (hpf1*hpf) + (lpf1*lpf)
plt.figure(figsize=(16, 8))
plt.plot(time, yaw_filtered, label='Complementary Filter')
plt.plot(time, yaw_z, label='Yaw computed by imu')
plt.legend(loc='lower right', fontsize='x-large')
plt.grid(color='grey', linestyle='--', linewidth=1)
plt.xlabel('Time')
plt.ylabel('Yaw (radians)')
plt.title('imu Yaw vs Complementary Filter Yaw')
plt.show()


#LPF for Yaw v/s HPF gor gyro v/x Complementary Yaw
plt.figure(figsize = (16,8))
plt.plot(time, lpf, label='LPF Calibrated Yaw',c= 'teal')
plt.legend(loc = "upper right")
plt.plot(time, hpf, label = 'HPF Gyro Yaw', c = 'black')
plt.plot(time, yaw_filtered, label='Complementary Filter',c= 'crimson')
plt.legend(loc='upper right', fontsize='x-large')
plt.grid(color='grey', linestyle='--', linewidth=1)
plt.legend(loc="upper right")
plt.xlabel('Time')
plt.ylabel('Yaw (radians)')
plt.title('LPF for Magnetic Yaw V/S HPF for Gyro Yaw V/S  Complimentary Yaw')
plt.show()