#!/usr/bin/env python

import tf
import rospy
import numpy as np
from storm32_gimbal.msg import GimbalOrientation
from geometry_msgs.msg import QuaternionStamped, Quaternion
import time
import matplotlib.pyplot as plt
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, VisionInfo
from cv_bridge import CvBridge
import cv2
import math as m
import csv


print(cv2.__version__)
(major_ver, minor_ver, subminor_ver) = (cv2.__version__).split(".")


## deection callback 
detections = Detection2DArray()
def get_detections(data):
    global detections
    detections = data
    return detections

overlayed_img = Image()
def get_overlayed_img(data):
    global overlayed_img
    overlayed_img = data
    return overlayed_img

raw_img = Image()
def get_raw_img(data):
    global raw_img
    raw_img = data
    return raw_img



def main():
    rospy.init_node("object_tracker")
    bridge = CvBridge()
    rate = rospy.Rate(30)

    # tracker topic
    

    # some variable initialization 
    seq_prev = 0
    obj_centre_prev = (-1,-1)
    obj_centre_curr = (-1,-1)

    # tracker declaration and definition #########################
    tracker_types = ['BOOSTING', 'MIL','KCF', 'TLD', 'MEDIANFLOW', 'GOTURN', 'CSRT']
    tracker_type = tracker_types[2]
    #tracker = cv2.Tracker_create(tracker_type)
    if int(major_ver) <= 3:
        tracker = cv2.Tracker_create(tracker_type)
        print('i am here')
    else:
        if tracker_type == 'BOOSTING':
            tracker = cv2.TrackerBoosting_create()
        if tracker_type == 'MIL':
            tracker = cv2.TrackerMIL_create()
        if tracker_type == 'KCF':
            tracker = cv2.TrackerKCF_create()
        if tracker_type == 'TLD':
            tracker = cv2.TrackerTLD_create()
        if tracker_type == 'MEDIANFLOW':
            tracker = cv2.TrackerMedianFlow_create()
        if tracker_type == 'GOTURN':
            tracker = cv2.TrackerGOTURN_create()
        if tracker_type == "CSRT":
            tracker = cv2.TrackerCSRT_create()

    #####################################


    while not rospy.is_shutdown():
        # subscribe to raw video source
        rospy.Subscriber("/video_source/raw", Image, get_raw_img, queue_size=1)
        # subscribe to detections from detector node for initialization
        rospy.Subscriber("/detectnet/detections", Detection2DArray,
                            get_detections, queue_size=1)

        if detections.header.seq != 0 and detections.header.seq != seq_prev:
            if len(detections.detections) > 1:
                for i in range(0, len(detections.detections)):
                    if detections.detections[i].results[0].score > detections.detections[index].results[0].score:
                        index = i
            obj_centre_curr = (int(detections.detections[index].bbox.center.x), int(detections.detections[index].bbox.center.y))
    
        else:
            continue
        
        if obj_centre_curr != (-1,-1):
            # Initialize tracker
            init_bbox = (obj_centre_curr[0] - 50, obj_centre_curr[1] - 80, obj_centre_curr[0]+50, obj_centre_curr[1]+80)
            #rosimg = raw_img.data
            cv_img  = bridge.imgmsg_to_cv2(raw_img, desired_encoding="passthrough")
            print(cv_img.shape)

        rate.sleep()

if __name__ == "__main__":
    main()
        



