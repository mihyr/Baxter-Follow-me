#!/usr/bin/env python3

import rospy
from collections import deque
from imutils.video import VideoStream
import pyrealsense2 as rs
import numpy as np
import cv2
import argparse
import time
import imutils
from geometry_msgs.msg import Point
import time


def go_pointCallback(msg_Point):
    global x_wpoint, y_wpoint, z_wpoint
    x_wpoint = msg_Point.x
    y_wpoint = msg_Point.y
    z_wpoint = msg_Point.z
   

def cv_to_waypoint():
    global x_wpoint, y_wpoint, z_wpoint
    
    x_waypoints = []
    y_waypoints = []
    z_waypoints = []

    
    i = 0
    while True:
        x_waypoints.append(x_wpoint)
        y_waypoints.append(y_wpoint)
        z_waypoints.append(z_wpoint)

        time.sleep(0.5) #delay callback by 0.5s so we don't collect all waypoints being published from CV node

        if (len(y_waypoints) > 1):
            delta = y_waypoints[i] - y_waypoints[i-1]
        if (delta <= 0.001):
            print("target object is stationary\r\n")
            print("store waypoint into ros params\r\n")

            rospy.set_param('x_waypoints', x_waypoints)
            rospy.set_param('y_waypoints', y_waypoints)
            rospy.set_param('z_waypoints', z_waypoints)
            
            break
    




if __name__ == '__main__':
    rospy.init_node('cv_to_waypoint', anonymous=True)
    sub = rospy.Subscriber('world_coord', Point, go_pointCallback)

    cv_to_waypoint()
