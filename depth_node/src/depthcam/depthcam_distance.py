import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from cmath import pi
from turtle import distance
import cv2
import pyrealsense2
from realsense_depth import *
import math
import numpy as np


#Defining depth cam

depthcam=DepthCamera()

bridge=CvBridge()
key=cv2.waitKey(1)

def imagecallback(msg):
    try:
        #ros to cv2 img
        cv2_img=bridge.imgmsg_to_cv2(msg,"depth") 
    except CvBridgeError as e:
        print(e)
    else:
        cv2.imwrite('depthcam_image.jpeg',cv2_img)
       

def main():
    rospy.init_node('image_subscriber')

    topic="/camera/depth/image_rect_raw"

    #image subscriber
    rospy.Subscriber(topic,Image,imagecallback)
    rospy.spin()

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


        #Distances
        dist1=depth_frame[coordinate[1],coordinate[0]]
        dist2=depth_frame[coordinate_2[1],coordinate_2[0]]
        dist3=depth_frame[coordinate_3[1],coordinate_3[0]]
        dist4=depth_frame[coordinate_4[1],coordinate_4[0]]

        x=np.array([dist1,dist2,dist3,dist4], dtype=np.float64)

        #angle wrt slope
        slope_1=math.degrees(math.atan((x[0]-x[1])/50.0))
        slope_2=math.degrees(math.atan((x[0]-x[2])/100.0))
        slope_3=math.degrees(math.atan((x[0]-x[3])/150.0))

        #slopes array
        slopes=np.array([slope_1,slope_2,slope_3], dtype=np.float64)

        slope_mean=((slope_1+slope_2+slope_3)/np.sqrt(3.0))
        print(90.0-slope_mean)

        cv2.imshow("Depth Frame", depth_frame)
       

if __name__=='__main__':
    main()
