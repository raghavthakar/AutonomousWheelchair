from cmath import pi
from turtle import distance
import cv2
import pyrealsense2
from realsense_depth import *
import math
import numpy as np
from scipy import signal


#Defining depth cam

depthcam=DepthCamera()


while True:
    ret,depth_frame,clr_frame=depthcam.get_frame()

    #5 coordinates 50mm pixels apart
    coordinate=(300,250)
    coordinate_2=(300,300)
    coordinate_3=(300,350)
    coordinate_4=(300,400)
    coordinate_5=(300,450)

    #Display
    cv2.circle(depth_frame,coordinate,4,(0,255,0))
    cv2.circle(depth_frame,coordinate_2,4,(0,255,0))
    cv2.circle(depth_frame,coordinate_3,4,(0,255,0))
    cv2.circle(depth_frame,coordinate_4,4,(0,255,0))
    cv2.circle(depth_frame,coordinate_5,4,(0,255,0))



    dist1=depth_frame[coordinate[1],coordinate[0]]
    dist2=depth_frame[coordinate_2[1],coordinate_2[0]]
    dist3=depth_frame[coordinate_3[1],coordinate_3[0]]
    dist4=depth_frame[coordinate_4[1],coordinate_4[0]]
    # dist5=depth_frame[coordinate_5[1],coordinate_5[0]]

    #distance array
    x=np.array([dist1,dist2,dist3,dist4], dtype=np.float64)
    

   #Moving Average Filter
    # def moving_average(x,N):
    #     np.cumsum=np.cumsum(np.insert(x,0,0))
    #     return (np.cumsum[N:]-np.cumsum[:-N]/float(N))

    # print(moving_average(x,4))

    

    slope_1=math.degrees(math.atan((x[0]-x[1])/50.0))
    slope_2=math.degrees(math.atan((x[0]-x[2])/100.0))
    slope_3=math.degrees(math.atan((x[0]-x[3])/150.0))
    # slope_4=math.degrees(math.atan(dist1-dist5/200.0))

    
    slope_mean=((slope_1+slope_2+slope_3)/3.0)
    print(90.0-slope_mean)

    cv2.imshow("Depth Frame", depth_frame)
    # cv2.imshow("Coloured Frame", clr_frame)
    key=cv2.waitKey(1)

    if key==27:
        break  #Press F to break

