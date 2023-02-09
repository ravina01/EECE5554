from bagpy import bagreader
from matplotlib import pyplot as plt
import pandas as pd
import numpy as np

#Uncomment below line to see : Scatter plot of Open space walking
path = "/home/ravina/catkin_ws/src/gps_driver/Data/walking/open/open_walking.bag"
#path = "/home/ravina/catkin_ws/src/gps_driver/Data/walking/open/cent_walk_2.bag"
bag = bagreader(path)

#Uncomment below line to see : Scatter plot of Occluded walking
path = "/home/ravina/catkin_ws/src/gps_driver/Data/walking/occluded/ravina/occluded_walking.bag"
bag = bagreader(path)

#Uncomment below line to see : Scatter plot of Open space stationary
#path = "/home/ravina/catkin_ws/src/gps_driver/Data/stationary/open/open_stationary.bag"
#bag = bagreader(path)

#Uncomment below line to see : Scatter plot of Occluded stationary
#path = "/home/ravina/catkin_ws/src/gps_driver/Data/stationary/occluded/occluded_stationary.bag"
#bag = bagreader(path)


bag.topic_table
data = bag.message_by_topic('/gps')
readings = pd.read_csv(data)

easting = np.array(readings['UTM_easting'])
northing = np.array(readings['UTM_northing'])

fig=plt.figure()
myaxes=fig.add_axes([0,0,1,1])
myaxes.plot(easting,northing,'r', lw='3')
myaxes.set_title('straight line data')
myaxes.set_xlabel('UTM_Easting (Meter)')
myaxes.set_ylabel('UTM_Northing (Meter)')

#Slope(m) and Y intercept(c) calculation
m, c = np.polyfit(easting, northing,1)
plt.plot(easting, m*easting + c)
plt.show()

new_northing=[]
#updating Y (northing) values
for i in readings['UTM_easting']:
    new= m*i+c
    new_northing.append(new)

#finding values of error terms
error_par = new_northing - northing
#square of the error terms
error_square = pow(error_par,2)
#sum of error_sqaure's
error_sum = sum(error_square)
#dividing the error value by number of total observations 
error=error_sum/119
print("Mean Squared Error in square meter is = ",error)
Root_mean_error_square= pow(error,0.5)
print("Root Mean Square Error in meter is = ",Root_mean_error_square)