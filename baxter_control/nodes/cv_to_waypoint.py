#!/usr/bin/env python3
from collections import deque
from imutils.video import VideoStream
import pyrealsense2 as rs
import numpy as np
import cv2
import argparse
import time
import imutils
import rospy 
from geometry_msgs.msg import Point


rospy.init_node('tracking')
pub= rospy.Publisher('world_coord',Point, queue_size = 50)

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
profile = pipeline.start(config)

ap = argparse.ArgumentParser()
ap.add_argument("-b", "--buffer", default=64, type=int, help="max buffer size")
ap.add_argument("-c", "--color",help="input tracking color")
args = vars(ap.parse_args())

rate=rospy.Rate(1)

pts = deque(maxlen=args["buffer"])

if args["color"]=='green':
    print('colour is green')
    Upper = np.array([102,255,255],np.uint8)
    Lower = np.array([25,52,72],np.uint8)
    
elif args["color"]=='red':
    print('colour is red')
    Upper = np.array([180,255,255],np.uint8)
    Lower = np.array([136,87,111],np.uint8)

else:
    print('colour is blue')
    Upper = np.array([120,255,255],np.uint8)
    Lower = np.array([94,80,2],np.uint8)

x=0
y=0
z=0

while True:
    frames = pipeline.wait_for_frames()
    depth_frame = frames.get_depth_frame()
    colour_frame = frames.get_color_frame()

    color_image = np.asanyarray(colour_frame.get_data())
    depth_image = np.asanyarray(depth_frame.get_data())

    depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics


    frame = imutils.resize(color_image, width=600)
    blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    mask = cv2.inRange(hsv,Lower, Upper)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    cnts = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    center = None

    if len(cnts) > 0:
        c = max(cnts, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        center = (int(M['m10']/M['m00']), int(M['m01']/M['m00']))

        if radius > 10:
            cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
            cv2.circle(frame, center, 5, (0, 0, 255), -1)

    pts.append(center)

    p=Point()
    x_waypoints = []
    y_waypoints = []
    z_waypoints = []

    for i in range(1, len(pts)):
        if pts[i-1] is None or pts[i] is None:
            continue

    
        thickness = int(np.sqrt(args["buffer"] / float(i+1)) * 2.5)
        cv2.line(frame, pts[i-1], pts[i], (0, 0, 255), thickness)
        
        if i%10==0 and i!=0 :
            p.x=x/10
            p.y=y/10
            p.z=z/10
            pub.publish(p)
            x_waypoints.append(p.x)
            y_waypoints.append(p.y)
            z_waypoints.append(p.z)

            #  reset the numbers 
            x=0
            y=0
            z=0
        else:
            depth= depth_frame.get_distance(pts[i][0],pts[i][1])
            depth_point = rs.rs2_deproject_pixel_to_point(depth_intrin, [pts[i][0],pts[i][1]], depth)
            x=x+depth_point[0]
            y=y+depth_point[1]
            z=z+depth_point[2]
        
    rospy.set_param('x_waypoints', x_waypoints)
    rospy.set_param('y_waypoints', y_waypoints)
    rospy.set_param('z_waypoints', z_waypoints)
    
        

        # print(x,y,z)   
        

    rate.sleep()
          
    cv2.imshow('RealSense', frame)
    # cv2.imshow("Depth", depth_image)

    if cv2.waitKey(25) == ord('q'):
        break


pipeline.stop()
cv2.destroyAllWindows()
