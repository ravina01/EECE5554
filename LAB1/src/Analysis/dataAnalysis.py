import bagpy
from bagpy import bagreader
import matplotlib.pyplot as plt
import pandas as pd

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
