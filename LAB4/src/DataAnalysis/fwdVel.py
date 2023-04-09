import math
import time
from bagpy import bagreader
import matplotlib.pyplot as plt
import numpy as np
import seaborn as sns
import pandas as pd
from scipy.optimize import fsolve
import scipy.integrate as integrate
from scipy.signal import butter
from scipy import signal

sns.set_style("dark")
sns.color_palette("viridis", as_cmap=True)
plt.rcParams.update({'font.size': 22})

bag = bagreader("/home/ravina/catkin_ws/src/imu_gps/Data/imu_gps.bag")
data = bag.message_by_topic('/imu')
readings = pd.read_csv(data).reset_index()
#readings = readings[12000:100000]
#readings = readings[52000:82000]
readings = readings[52000:82000]

data_gps = bag.message_by_topic('/gps')
gpscsv = pd.read_csv(data_gps)
#gpscsv = gpscsv[300:2500]
#gpscsv = gpscsv[1300:2050]
gpscsv = gpscsv[1300:2050]

secs = readings['Header.stamp.secs']
nsecs = np.double(readings['Header.stamp.nsecs'])
nsecs = nsecs / 1000000000
time_x = np.double(secs) + nsecs

#IMU Velocity
raw_val = readings['imu.linear_acceleration.x']
x = np.mean(raw_val)
linear_acc = raw_val - x

difference = []
for i in range(len(linear_acc)-1):
  difference.append((linear_acc.iloc[i + 1] - linear_acc.iloc[i]) / (0.025))
difference = np.array(difference)

final = linear_acc[1:] 
Forward_velocity_adjusted = integrate.cumtrapz(final, initial=0)
Forward_velocity_adjusted[Forward_velocity_adjusted<0] = 0
Forward_velocity_raw = integrate.cumtrapz(linear_acc, initial=0)

#GPS Velocity
time=gpscsv['Header.stamp.secs']
UTMeast = gpscsv['UTM_easting']
UTMnorth = gpscsv['UTM_northing']
Latitude = gpscsv['Latitude']
Longitude = gpscsv['Longitude']
distance=[]
velocity=[]
for i in range(len(UTMnorth)-1):
  distance = np.append(distance, math.sqrt(((UTMnorth.iloc[i + 1] - UTMnorth.iloc[i]) ** 2) + (UTMeast.iloc[i + 1] - UTMeast.iloc[i]) ** 2))
gps_vel = distance / time.iloc[1:]
print("distance = ", distance)

#plot b/w IMU velocity and gps velocity after adjustment.
time_gp = gpscsv['Time']
plt.figure(figsize = (16,8))
plt.plot(np.array(time_x[1:]), np.array(Forward_velocity_adjusted / 1000), label='IMU Adjusted Velocity', c='palevioletred')
plt.plot(np.array(time_gp[1:]), np.array(gps_vel*2000), label='GPS adjusted Velocity')
plt.legend(loc='upper right', fontsize='x-large')
plt.grid(color='grey', linestyle='--', linewidth=1)
plt.title('Forward velocity from IMU and GPS after adjustment')
plt.xlabel('Time (secs)')
plt.ylabel('Velocity (m/sec)')
plt.show()

#Plot b/w forward velocity from imu to gps velocity before adjustment
plt.figure(figsize = (16,8))
plt.plot(time_x, Forward_velocity_raw, label='IMU Raw Velocity', c='palevioletred')
plt.plot(time_gp[1:], gps_vel*2000000, label='GPS Raw Velocity')
plt.legend(loc='upper right', fontsize='x-large')
plt.grid(color='grey', linestyle='--', linewidth=1)
plt.title('Forward velocity from IMU and GPS before adjustment')
plt.xlabel('Time (secs)')
plt.ylabel('Velocity (m/sec)')
plt.show()



#Displacement
disp_x = integrate.cumtrapz(Forward_velocity_adjusted, initial=0)
int_gps_vel = integrate.cumtrapz(distance, initial=0)

accex = readings['imu.linear_acceleration.x']
timeimu = readings['Header.stamp.secs']+readings['Header.stamp.nsecs']*10e-9
x2dot = accex
x1dot = integrate.cumtrapz(x2dot)
angz = readings['imu.angular_velocity.z']
angzdot = integrate.cumtrapz(angz)
# angzdot = np.concatenate(([0], angzdot))
y2dot = angz[1:] * x1dot
t = readings['Header.stamp.secs']
Y_observed = readings['imu.linear_acceleration.y']
x_c = (Y_observed[1:] - (angz[1:] * x1dot))*10/ angzdot
print("offset x_c = ", np.mean(x_c[10000:20000]))
plt.figure(figsize = (8,8))
plt.plot(Y_observed, label = 'Y observed', c='steelblue')
plt.plot(y2dot/1000, label = 'wX(dot)', c='orangered')
plt.legend(loc='upper right', fontsize='x-large')
plt.grid(color='grey', linestyle='--', linewidth=1)
plt.title('Y_observed V/S wX(dot)')
plt.xlabel('Time')
plt.ylabel('acceleration')
plt.show()

#Trajectory of Vehicle
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
fv = np.unwrap(Forward_velocity_adjusted)
mgx = np.array(readings['mag_field.magnetic_field.x'] - np.mean(readings['mag_field.magnetic_field.x']))
mgy = np.array(readings['mag_field.magnetic_field.y'] - np.mean(readings['mag_field.magnetic_field.y']))
mgh1 = yaw_z
rot = (108*np.pi/180)
theta = np.arctan2(mgy,mgx) * rot
X=[]
Y=[]
x = 0
y = 0
for i in range(len(Forward_velocity_adjusted)):
  y += Forward_velocity_adjusted[i] * np.cos(theta[i])
  x += Forward_velocity_adjusted[i] * np.sin(theta[i])
  X.append(x)
  Y.append(y)
  

# unit1 = np.cos(mgh1[1:]+rot)*fv
# unit2 = -np.sin(mgh1[1:]+rot)*fv
# unit3 = np.cos(mgh1[1:]+rot)*fv
# unit4 = np.sin(mgh1[1:]+rot)*fv
# rads = (180/np.pi)
# ve = unit1+unit2
# vn = unit3+unit4
# xe = integrate.cumtrapz(ve)
# xn = integrate.cumtrapz(vn)

plt.figure(figsize = (8,8))
#plt.plot((xe/(10**6))/2,-xn/(10**5), c='crimson')
plt.plot(X,Y, c='crimson')
plt.grid(color='grey', linestyle='--', linewidth=1)
plt.title('Trajectory of Vehicle')
plt.xlabel('Xe')
plt.ylabel('Xn')
plt.plot()
plt.show()

plt.figure(figsize = (8,8))
plt.plot(UTMeast, UTMnorth, c ='palevioletred')
plt.grid(color='grey', linestyle='--', linewidth=1)
plt.title('UTM Easting V/S UTM Northing')
plt.xlabel('UTM Easting')
plt.ylabel('UTM Northing')
plt.show()