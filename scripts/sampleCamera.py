#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge, CvBridgeError

videoPath = "/sampleData/sample.mp4"

cam = cv2.VideoCapture(videoPath)

rospy.init_node('VideoPublisher', anonymous=True)
VideoRaw = rospy.Publisher('VideoRaw', Image, queue_size=10)
VideoRawMono = rospy.Publisher('VideoRawMono', Image, queue_size=10)

rate = rospy.Rate(30) 

bridge = CvBridge()

while not rospy.is_shutdown():

    if not cam.isOpened():
        rospy.logwarning("Capture was closed so reopen it!")
        cam = cv2.VideoCapture(videoPath)

    meta, frame = cam.read()

    frameMono = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    msg_frame = bridge.cv2_to_imgmsg(frame, encoding="passthrough")
    msg_frame_mono = bridge.cv2_to_imgmsg(frameMono, encoding="passthrough")

    VideoRaw.publish(msg_frame, "bgr8")
    VideoRawMono.publish(msg_frame_mono, "mono8")

    rate.sleep()

rospy.logwarning("Closing capture!")
cam.close()