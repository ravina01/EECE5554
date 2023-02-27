from bagpy import bagreader
from matplotlib import pyplot as plt
import pandas as pd
import numpy as np
import math
from statistics import stdev
from matplotlib import style

#Uncomment below line to see : Scatter plot of Occluded walking
path = "/home/ravina/catkin_ws/src/gps_driver/Data/occluded/walking/occluded_walking.bag"

bag = bagreader(path)


bag.topic_table
data = bag.message_by_topic('/gps')
readings = pd.read_csv(data)

easting = np.array(readings['UTM_easting'])
northing = np.array(readings['UTM_northing'])


mean_east=np.mean(easting)
print("Mean Easting Value", mean_east)
mean_north=np.mean(northing)
print("Mean Northing Value", mean_north)

def mean_square_error(east,nort):
    #Slope(m) and Y intercept(c) calculation
    m, c = np.polyfit(east, nort,1)
    plt.plot(east, m*east + c, color='black')
    new_nort=[]
    #updating Y (northing) values
    for i in east:
        new= m*i+c
        new_nort.append(new)
    #finding values of error terms
    error_par = new_nort - nort
    #square of the error terms
    error_square= pow(error_par,2)
    #sum of error_sqaure's
    error_sum=sum(error_square)
    #dividing the error value by number of total observations 
    error=error_sum/east.size

    return error

fig=plt.figure()
myaxes=fig.add_axes([0,0,1.0,1.0])

easting1=easting[:55]
northing1=northing[:55]
mse1= mean_square_error(easting1,northing1)
rmse1=math.sqrt(mse1)
print(rmse1)
easting2=easting[55:70]
northing2=northing[55:70]
mse2= mean_square_error(easting2,northing2)
rmse2=math.sqrt(mse2)
print(rmse2)
easting3=easting[73:129]
northing3=northing[73:129]
mse3= mean_square_error(easting3,northing3)
rmse3=math.sqrt(mse3)
print(rmse3)
easting4=easting[129:157]
northing4=northing[129:157]
mse4= mean_square_error(easting4,northing4)
rmse4=math.sqrt(mse4)
print(rmse4)

rmse=(rmse1+rmse2+rmse3+rmse4)/4

print('Standard Deviation of UTM_Easting:',stdev(easting))
print('Standard Deviation of UTM_Northing',stdev(northing))
print('Root Mean Square Error',rmse)

# myaxes.plot(easting,northing,'o', lw='3')
plt.scatter(easting1,northing1, lw='2', s = 50)
plt.scatter(easting2,northing2, lw='2', s = 50)
plt.scatter(easting3,northing3, lw='2', s = 50)
plt.scatter(easting4,northing4, lw='2', s = 50)


#Labeling the plot
myaxes.set_title('Moving data - partially occluded (Easting-Northing)',fontweight='bold')
myaxes.set_xlabel('UTM_Easting(Meter)',fontweight='bold')
myaxes.set_ylabel('UTM_Northing(Meter)',fontweight='bold')

plt.show()
