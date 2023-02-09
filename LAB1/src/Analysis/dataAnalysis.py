import bagpy
from bagpy import bagreader
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np

#Uncomment below line to see : Scatter plot of Open space walking
#path = "/home/ravina/catkin_ws/src/gps_driver/Data/walking/open/open_walking.bag"
#path = "/home/ravina/catkin_ws/src/gps_driver/Data/walking/open/cent_walk_2.bag"
#bag = bagreader(path)

#Uncomment below line to see : Scatter plot of Occluded walking
#path = "/home/ravina/catkin_ws/src/gps_driver/Data/walking/occluded/ravina/occluded_walking.bag"
#bag = bagreader(path)

#Uncomment below line to see : Scatter plot of Open space stationary
#path = "/home/ravina/catkin_ws/src/gps_driver/Data/stationary/open/open_stationary.bag"
#bag = bagreader(path)

#Uncomment below line to see : Scatter plot of Occluded stationary
path = "/home/ravina/catkin_ws/src/gps_driver/Data/stationary/occluded/occluded_stationary.bag"
bag = bagreader(path)

bag.topic_table
data = bag.message_by_topic('/gps')
readings = pd.read_csv(data)
x = readings['UTM_easting']
y = readings['UTM_northing']

easting = readings['UTM_easting']
northing =  readings['UTM_northing']

readings['UTM_easting'] = readings['UTM_easting'] - readings['UTM_easting'].min()
readings['UTM_northing'] = readings['UTM_northing'] - readings['UTM_northing'].min()
print(readings[['UTM_easting', 'UTM_northing']])
print(readings)

plt.rcParams.update({'font.size': 20})
#readings[['UTM_easting','UTM_northing']].plot()
fig, ax = bagpy.create_fig(1)
ax[0].scatter(x = 'UTM_easting', y = 'UTM_northing', data = readings, s = 20, label = 'UTM_easting VS UTM_northing')
for axis in ax:
    #axis.legend()
    axis.set_xlabel('UTM_easting in meters', fontsize = 20)
    axis.set_ylabel('UTM_northing in meters', fontsize = 20)
    plt.savefig(path.split('.')[0] + "_eastNorth" + ".png", bbox_inches = "tight")
plt.show()

#plt.rcParams.update({'font.size': 40})
#readings[['Altitude','Time']].plot()
fig, bx = bagpy.create_fig(1)
bx[0].scatter(x = 'Time', y = 'Altitude', data = readings, s = 20, label = 'Altitude VS Time')
for axis in bx:
    #axis.legend()
    axis.set_xlabel('Time in seconds', fontsize = 20)
    axis.set_ylabel('Altitude in meters', fontsize = 20)
    plt.savefig(path.split('.')[0] + "_alti" + ".png", bbox_inches = "tight")
plt.show()

#Known Locations
open1 = [327748.86,4689330.21] # exact location of open space
occluded = [327970.56,4689515.05] # exact location of partially occluded space

error_easting = [item - occluded[0] for item in x]
error_northing = [item - occluded[1] for item in y]

mean_easting = np.mean(error_easting)
mean_northing = np.mean(error_northing)

median_easting = np.median(error_easting)
median_northing = np.median(error_northing)

fig3,ax3 = plt.subplots(2)
ax3[0].hist(error_easting, bins = 20)
ax3[0].set_title('Easting error(m)')
ax3[0].set_xlabel('Error from known(m) Measured Easting(m)')
ax3[0].set_ylabel('Frequency of error in Easting(m)')

ax3[1].hist(error_northing, bins = 20)
ax3[1].set_title('Northing error(m)')
ax3[1].set_xlabel('Error from known(m) Measured Northing(m)')
ax3[1].set_ylabel('Frequency of error in Northing(m)')

plt.show()
fig3.savefig(path.split('.')[0] + "error_hist" + ".png", bbox_inches = 'tight')

print(mean_easting)
print(mean_northing)
print(median_easting)
print(median_northing)
