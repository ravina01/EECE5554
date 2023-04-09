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
import rosbag

sns.set_style("dark")
sns.color_palette("viridis", as_cmap=True)
plt.rcParams.update({'font.size': 20})

bag = rosbag.Bag("/home/ravina/catkin_ws/src/imu_gps/Data/imu_gps.bag")
#bagreader("/home/ravina/catkin_ws/src/imu_gps/Data/imu_gps.bag")
msgs = []
for topic, msg, _ in bag.read_messages(topics=['/imu']):
    msgs.append(msg)

msgs_gps = []
for topic, msg, _ in bag.read_messages(topics=['/gps']):
    msgs_gps.append(msg)

msgs_drive = msgs[10000:100000]  # Driving data

# Extract yaw
yaw_data = np.zeros(len(msgs_drive))
for i, msg in enumerate(msgs_drive):
    t3 = +2.0 * (msg.imu.orientation.w * msg.imu.orientation.z + msg.imu.orientation.x * msg.imu.orientation.y)
    t4 = +1.0 - 2.0 * (msg.imu.orientation.y * msg.imu.orientation.y + msg.imu.orientation.z * msg.imu.orientation.z)
    yaw_z = np.arctan2(t3, t4)
    yaw_data[i] = np.deg2rad(yaw_z)
    yaw_data[i] = np.unwrap([yaw_data[i]])[0]

print(yaw_data[0])

# Extract MagneticField vectors
mag_x = np.zeros(len(msgs_drive))
mag_y = np.zeros(len(msgs_drive))
for i, msg in enumerate(msgs_drive):
    mag_x[i] = msg.mag_field.magnetic_field.x * 1000  # Tesla to Gauss conversion
    mag_y[i] = msg.mag_field.magnetic_field.y * 1000

yaw_from_mag_raw = np.arctan2(-mag_y, mag_x)

# Plotting
# plt.figure()
# plt.plot(yaw_data, "b", linewidth=2, label="YAW from IMU")
# plt.plot(yaw_from_mag_raw, "r", linewidth=2, label="Raw YAW from Magnetometer")
# plt.title("YAW from IMU vs Raw YAW from Mag")
# plt.xlabel("Number of readings")
# plt.ylabel("Yaw (radians)")
# plt.grid(True)
# plt.legend()
# plt.show()

# Removing hard-iron distortion
mag_x = mag_x - 24.55
mag_y = mag_y - 220.5

# Removing soft-iron distortion
# Removing tilt
T = np.array([[-0.83746, -0.54649],
              [0.54649, -0.83746]])

for i in range(len(msgs_drive)):
    tilt_correction = T @ np.array([mag_x[i], mag_y[i]])
    mag_x[i] = tilt_correction[0]
    mag_y[i] = tilt_correction[1]

scale_factor = 0.98

#Scale the x-axis by the scaling factor
mag_x_scaled = mag_x / scale_factor
mag_x = mag_x_scaled

yaw_from_mag = np.arctan2(-mag_y, mag_x)
yaw_from_mag = np.unwrap(scale_factor * yaw_from_mag)+3.06

yaw_from_mag[15323:34862] = yaw_from_mag[15323:34862] + 5.7
yaw_from_mag[48534:61191] = yaw_from_mag[48534:61191] + 5.7

plt.figure()
plt.plot(yaw_data, "b", linewidth=2, label="YAW from IMU")
plt.plot(yaw_from_mag, "r", linewidth=2, label="Corrected YAW from Magnetometer")
plt.title("YAW from IMU vs Corrected YAW from Mag")
plt.xlabel("Number of readings")
plt.ylabel("Yaw (radians)")
plt.grid(True)
plt.legend()
plt.show()

# Extract angular velocity
ang_vel = np.zeros(len(msgs_drive))
for i, msg in enumerate(msgs_drive):
    ang_vel[i] = msg.imu.angular_velocity.z

# Extract time
time = np.zeros(len(msgs_drive))
for i, msg in enumerate(msgs_drive):
    time[i] = msg.Header.stamp.secs

yaw_from_gyr = integrate.cumtrapz(time, ang_vel)
yaw_from_gyr = np.append(yaw_from_gyr, 0) # adding zero to match a length
yaw_from_gyr[34862:48534] -= 5.8
yaw_from_gyr[61336:] -= 5.8

plt.figure()
plt.plot(yaw_data, "b", linewidth=2, label="YAW from IMU")
plt.plot(yaw_from_gyr, "r", linewidth=2, label="YAW from Gyroscope")
plt.title("YAW from IMU vs YAW from Gyro")
plt.xlabel("Number of readings")
plt.ylabel("Yaw (radians)")
plt.grid(True)
plt.legend()
plt.show()


# plt.figure()
# plt.plot(yaw_from_mag, "r", linewidth=2, label="YAW from Magnetometer")
# plt.plot(yaw_from_gyr, "b", linewidth=2, label="YAW from Gyroscope")
# plt.title("YAW from Mag and Gyro")
# plt.xlabel("Number of readings")
# plt.ylabel("Yaw (radians)")
# plt.grid(True)
# plt.legend()
# plt.show()


import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import butter, filtfilt

def butter_highpass(cutoff, fs, order=5):
    nyq = 0.5 * fs
    normal_cutoff = cutoff / nyq
    b, a = butter(order, normal_cutoff, btype='high', analog=False)
    return b, a

def butter_lowpass(cutoff, fs, order=5):
    nyq = 0.5 * fs
    normal_cutoff = cutoff / nyq
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    return b, a

def apply_filter(data, cutoff, fs, order, filter_type='high'):
    if filter_type == 'high':
        b, a = butter_highpass(cutoff, fs, order=order)
    elif filter_type == 'low':
        b, a = butter_lowpass(cutoff, fs, order=order)
    y = filtfilt(b, a, data)
    return y

fs = 40  # sampling frequency
# yaw_from_gyr = np.array([...])  # your gyroscope yaw data
# yaw_from_mag = np.array([...])  # your magnetometer yaw data

yaw_hp_gyr = apply_filter(yaw_from_gyr, 1, fs, order=5, filter_type='high')
yaw_lp_mag = apply_filter(yaw_from_mag, 1e-16, fs, order=5, filter_type='low')
yaw_combined = yaw_hp_gyr + yaw_lp_mag
yaw_lp_mag = yaw_lp_mag + 0.1

plt.plot(yaw_lp_mag, "r", linewidth=2)
plt.plot(yaw_from_gyr, "g", linewidth=2)
plt.xlabel('Number of readings')
plt.ylabel('Yaw (radians)')
plt.grid(True)
plt.plot(yaw_combined, "b", linewidth=2)
plt.legend(["Mag yaw from high pass", "Gyro yaw from low pass", "Yaw from complimentary filter"])
plt.show()

# Extract acceleration
accel_x = np.zeros(len(msgs_drive))
for i, msg in enumerate(msgs_drive):
    accel_x[i] = msg.imu.linear_acceleration.x

time = np.arange(0, len(accel_x) * 0.025, 0.025)
forward_velocity = integrate.cumtrapz(time, accel_x)
forward_velocity = np.concatenate(([0], forward_velocity))

# Uncomment the following lines if you want to plot forward_velocity
# plt.figure()
# plt.plot(time, forward_velocity, "b", linewidth=2)
# plt.title("Forward Velocity IMU before adjustment")
# plt.xlabel("time")
# plt.ylabel("Velocity")
# plt.grid(True)
# plt.show()

# After adjustment
difference = np.zeros(len(msgs_drive))

for i in range(len(msgs_drive) - 1):
    difference[i] = (accel_x[i + 1] - accel_x[i]) * 40
difference[len(msgs_drive) - 1] = 0

accel_x_corrected = accel_x - difference
forward_velocity_corrected = integrate.cumtrapz(time, accel_x_corrected)
forward_velocity_corrected = np.concatenate(([0], forward_velocity_corrected))

for i in range(len(forward_velocity_corrected)):
    if forward_velocity_corrected[i] <= 0:
        forward_velocity_corrected[i] = 0

# plt.figure()
# plt.plot(time, forward_velocity, "r", linewidth=2, label="Actual forward velocity")
# plt.title("Forward velocity vs Forward velocity corrected")
# plt.xlabel("Time (s)")
# plt.ylabel("Velocity (m/s)")
# plt.grid(True)
# plt.plot(time, forward_velocity_corrected, "b", linewidth=2, label="Forward Velocity Corrected")
# plt.legend()
# plt.show()


msgs_gps_drive = msgs_gps[29:2500]  # Driving gps data

time_gps = np.arange(0, len(msgs_gps_drive), 1)

# Extract Easting and Northing data
UTM_easting = np.zeros(len(msgs_gps_drive))
UTM_northing = np.zeros(len(msgs_gps_drive))

for i, msg in enumerate(msgs_gps_drive):
    UTM_easting[i] = msg.UTM_easting
    UTM_northing[i] = msg.UTM_northing

min_easting = np.min(UTM_easting)
min_northing = np.min(UTM_northing)

UTM_easting -= min_easting
UTM_northing -= min_northing

UTM_combine = np.column_stack((UTM_easting, UTM_northing))

forward_velocity_gps = np.zeros(len(time_gps))
for i in range(len(time_gps) - 1):
    forward_velocity_gps[i] = np.linalg.norm(UTM_combine[i+1] - UTM_combine[i]) / (time_gps[i+1] - time_gps[i])
forward_velocity_gps[-1] = 0
forward_velocity_gps *= 2.5

forward_velocity_gps_i = np.interp(np.arange(0, len(time_gps), 1/40), time_gps, forward_velocity_gps)

# print(forward_velocity_gps[66])
# print("gps velocity", len(forward_velocity_gps_i))
# print("time", len(time))

print("Length of time array:", len(time))
print("Length of forward_velocity_gps_i array:", len(forward_velocity_gps_i))

# Displacement from imu
# Slicing forward_velocity_gps_i to match the length of time array
forward_velocity_gps_i = forward_velocity_gps_i[:90000]

# Create the time array with the same length as forward_velocity_gps_i
time = np.linspace(0, (len(forward_velocity_gps_i) - 1) * 0.025, len(forward_velocity_gps_i))

# Displacement from imu
imu_displacement = integrate.cumtrapz(time, forward_velocity_corrected)
imu_displacement = np.concatenate(([0], imu_displacement))
gps_displacement = integrate.cumtrapz(time, forward_velocity_gps_i)
gps_displacement = np.concatenate(([0], gps_displacement))

# Plotting
# plt.figure()
# plt.plot(time, imu_displacement, "b", linewidth=2, label="IMU Displacement")
# plt.plot(time, gps_displacement, "r", linewidth=2, label="GPS Displacement")
# plt.title("IMU vs GPS Displacement")
# plt.xlabel("time (s)")
# plt.ylabel("Displacement (m)")
# plt.grid(True)
# plt.legend()
# plt.show()

# Observed x-acceleration and y-acceleration
# Extract z-angular velocity
ang_vel_z = np.zeros(len(msgs_drive))
for i, msg in enumerate(msgs_drive):
    ang_vel_z[i] = msg.imu.angular_velocity.z

# Product of z-angular velocity and corrected velocity
p = np.zeros(len(msgs_drive))
for i in range(len(msgs_drive)):
    p[i] = ang_vel_z[i] * forward_velocity_corrected[i]

# Extract y acceleration
accel_y = np.zeros(len(msgs_drive))
for i, msg in enumerate(msgs_drive):
    accel_y[i] = msg.imu.linear_acceleration.y

# Plotting
# plt.figure()
# plt.plot(time[:90001], p[:90001], "b", linewidth=2, label="Acceleration-y")
# plt.plot(time[:90001], accel_y[:90001], "r", linewidth=2, label="w.Velocity-x")
# plt.title("Acceleration-y vs w.Velocity-x")
# plt.xlabel("time (s)")
# plt.ylabel("Acceleration (m/s^2)")
# plt.grid(True)
# plt.legend()
# plt.show()


# Dead Reckoning

# Yaw corrections
yaw_correction = yaw_data.copy()

yaw_correction[34860:34863] = 3.098
yaw_correction[34863:48533] = yaw_data[34863:48533] + 6.28
yaw_correction[48532:48536] = 3.12

yaw_correction[61189:61290] = 3.12
yaw_correction[61290:61312] = 3.14
yaw_correction[61306:84980] = yaw_data[61306:84980] + 6.24

yaw_correction[84980:85601] = 3.4
yaw_correction[85602:90001] = yaw_correction[85602:90001] + 6.23

# Finding path
heading_mag = yaw_correction

imu_x = np.zeros(len(imu_displacement))
imu_y = np.zeros(len(imu_displacement))

for i in range(1, len(imu_displacement)):
    imu_x[i] = imu_x[i-1] + np.linalg.norm(imu_displacement[i] - imu_displacement[i-1]) * np.cos(heading_mag[i-1])
    imu_y[i] = imu_y[i-1] + np.linalg.norm(imu_displacement[i] - imu_displacement[i-1]) * np.sin(heading_mag[i-1])

imu_x -= imu_x[0]
imu_y -= imu_y[0]

imu_x_scaled = imu_x.copy()
imu_y_scaled = imu_y.copy()


from scipy.interpolate import interp1d

# Interpolate UTM_easting and UTM_northing
x = np.arange(len(UTM_easting))
interp_fn = interp1d(x, UTM_easting, kind='linear')
#UTM_easting_i = interp_fn(np.arange(0, len(UTM_easting), 1/40))

x = np.arange(len(UTM_northing))
interp_fn = interp1d(x, UTM_northing, kind='linear')
#UTM_northing_i = interp_fn(np.arange(0, len(UTM_northing), 1/40))

UTM_easting_i = interp_fn(np.linspace(0, len(UTM_easting) - 1, len(UTM_easting) * 40))
UTM_northing_i = interp_fn(np.linspace(0, len(UTM_northing) - 1, len(UTM_northing) * 40))

plt.figure()
plt.plot(UTM_easting_i[29609:59102], UTM_northing_i[29609:59102], "b", linewidth=2)

imu_x_man = imu_x_scaled[29609:48800]
imu_y_man = imu_y_scaled[29609:48800]

imu_x_man -= 2700
imu_y_man -= 447
ang = np.pi / 2.5

imu_x_man_rot = imu_x_man * np.cos(ang) + imu_y_man * np.sin(ang)
imu_y_man_rot = -imu_x_man * np.sin(ang) + imu_y_man * np.cos(ang)

imu_x_man_rot -= 700
imu_y_man_rot += 1700

imu_f_x = imu_y_man_rot
imu_f_y = imu_x_man_rot

# Rotate
ang = np.pi / 3.5
imu_x_final = imu_f_x * np.cos(ang) + imu_f_y * np.sin(ang)
imu_y_final = -imu_f_x * np.sin(ang) + imu_f_y * np.cos(ang)

# Shift
imu_y_final += 1200

# plt.figure()
# plt.plot(imu_x_final, imu_y_final, "r", linewidth=2)

# plt.plot([590, 1602.6], [1340, 1701.71], "r", linewidth=2)
# plt.grid(True)
# plt.legend(["GPS Path", "Scaled IMU Path"])
# plt.title("Dead reckoning")
# plt.xlabel("Easting (m)")
# plt.ylabel("Northing (m)")
# plt.show()
