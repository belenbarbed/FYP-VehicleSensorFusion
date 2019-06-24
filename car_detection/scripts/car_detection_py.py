#!/usr/bin/env python3

import cv2
from cv_bridge import CvBridge
import numpy as np
import rospy
import time

from sensor_msgs.msg import CompressedImage, Image
from car_detection.msg import Detected_Img, Bbox

# Scale frame down for speed
scale = 1

def callback_1(data):
    global br
    global car_cascade
    global pub_1
    global scale
    rospy.loginfo("in callback for pixel 1")

    # Reduce frame size by half for speed purposes
    frame = br.compressed_imgmsg_to_cv2(data, desired_encoding="bgr8")
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    small_frame = cv2.resize(gray, (0, 0), fx=1/scale, fy=1/scale)

    # Detect faces in frame
    car_locations = car_cascade.detectMultiScale(small_frame, 1.1, 1)

    labels = []
    bboxes = []

    # Display the results
    for (top, right, bottom, left) in car_locations:
        # Scale back up face locations since the frame we detected in was scaled to 1/2 size
        top *= scale
        right *= scale
        bottom *= scale
        left *= scale

        bbox = Bbox()
        bbox.top = top
        bbox.right = right
        bbox.bottom = bottom
        bbox.left = left
        bboxes.append(bbox)

    # Publish results for visualisation node
    msg = Detected_Img()
    msg.header.stamp = data.header.stamp
    msg.img = br.cv2_to_compressed_imgmsg(frame)
    msg.bboxes = bboxes
    msg.labels = labels
    pub_1.publish(msg)

def callback_2(data):
    global br
    global car_cascade
    global pub_2
    global scale
    rospy.loginfo("in callback for pixel 2")

    # Reduce frame size by half for speed purposes
    frame = br.compressed_imgmsg_to_cv2(data, desired_encoding="bgr8")
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    small_frame = cv2.resize(gray, (0, 0), fx=1/scale, fy=1/scale)

    # Detect faces in frame
    car_locations = car_cascade.detectMultiScale(small_frame, 1.1, 1)

    labels = []
    bboxes = []

    # Display the results
    for (top, right, bottom, left) in car_locations:
        # Scale back up face locations since the frame we detected in was scaled to 1/2 size
        top *= scale
        right *= scale
        bottom *= scale
        left *= scale

        bbox = Bbox()
        bbox.top = top
        bbox.right = right
        bbox.bottom = bottom
        bbox.left = left
        bboxes.append(bbox)

    # Publish results for visualisation node
    msg = Detected_Img()
    msg.header.stamp = data.header.stamp
    msg.img = br.cv2_to_compressed_imgmsg(frame)
    msg.bboxes = bboxes
    msg.labels = labels
    pub_2.publish(msg)

def callback_3(data):
    global br
    global car_cascade
    global pub_3
    global scale
    rospy.loginfo("in callback for pixel 3")

    # Reduce frame size by half for speed purposes
    frame = br.compressed_imgmsg_to_cv2(data, desired_encoding="bgr8")
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    small_frame = cv2.resize(gray, (0, 0), fx=1/scale, fy=1/scale)

    # Detect faces in frame
    car_locations = car_cascade.detectMultiScale(small_frame, 1.1, 1)

    labels = []
    bboxes = []

    # Display the results
    for (top, right, bottom, left) in car_locations:
        # Scale back up face locations since the frame we detected in was scaled to 1/2 size
        top *= scale
        right *= scale
        bottom *= scale
        left *= scale

        bbox = Bbox()
        bbox.top = top
        bbox.right = right
        bbox.bottom = bottom
        bbox.left = left
        bboxes.append(bbox)

    # Publish results for visualisation node
    msg = Detected_Img()
    msg.header.stamp = data.header.stamp
    msg.img = br.cv2_to_compressed_imgmsg(frame)
    msg.bboxes = bboxes
    msg.labels = labels
    pub_3.publish(msg)

def callback_4(data):
    global br
    global car_cascade
    global pub_4
    global scale
    rospy.loginfo("in callback for pixel 4")

    # Reduce frame size by half for speed purposes
    frame = br.compressed_imgmsg_to_cv2(data, desired_encoding="bgr8")
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    small_gray = cv2.resize(gray, (0, 0), fx=1/scale, fy=1/scale)

    # Detect faces in frame
    car_locations = car_cascade.detectMultiScale(small_gray, 1.1, 1)

    labels = []
    bboxes = []

    # Display the results
    for (top, right, bottom, left) in car_locations:
        # Scale back up face locations since the frame we detected in was scaled to 1/2 size
        top *= scale
        right *= scale
        bottom *= scale
        left *= scale

        bbox = Bbox()
        bbox.top = top
        bbox.right = right
        bbox.bottom = bottom
        bbox.left = left
        bboxes.append(bbox)

    # Publish results for visualisation node
    msg = Detected_Img()
    msg.header.stamp = data.header.stamp
    msg.img = br.cv2_to_compressed_imgmsg(frame)
    msg.bboxes = bboxes
    msg.labels = labels
    pub_4.publish(msg)

def main():
    rospy.loginfo("Initialising node...")
    rospy.init_node("car_detection_py")
    rospy.loginfo("Initialising node...")

    global br
    br = CvBridge()

    global car_cascade
    car_cascade = cv2.CascadeClassifier('/home/soteris-group/bb2115/catkin_ws/src/car_detection/scripts/cars.xml')

    global pub_1
    pub_1  = rospy.Publisher("/pixel_1/camera0/image/detected", Detected_Img, queue_size=1)
    global pub_2
    pub_2  = rospy.Publisher("/pixel_2/camera0/image/detected", Detected_Img, queue_size=1)
    global pub_3
    pub_3  = rospy.Publisher("/pixel_3/camera0/image/detected", Detected_Img, queue_size=1)
    global pub_4
    pub_4  = rospy.Publisher("/pixel_4/camera0/image/detected", Detected_Img, queue_size=1)

    rospy.Subscriber("/pixel_1/camera0/image/compressed", CompressedImage, callback_1, queue_size=1)
    rospy.Subscriber("/pixel_2/camera0/image/compressed", CompressedImage, callback_2, queue_size=1)
    rospy.Subscriber("/pixel_3/camera0/image/compressed", CompressedImage, callback_3, queue_size=1)
    rospy.Subscriber("/pixel_4/camera0/image/compressed", CompressedImage, callback_4, queue_size=1)

    rospy.spin()

    rospy.loginfo("Finished car_detection_py")


if __name__ == "__main__":
    main()
