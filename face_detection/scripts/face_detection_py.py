#!/usr/bin/env python3

import cv2
from cv_bridge import CvBridge
import face_recognition
import numpy as np
import rospy
import time

from sensor_msgs.msg import CompressedImage, Image
from face_detection.msg import Detected_Img, Bbox

# Detect faces on 1-in-freq frames
# freq = 2
# counter_1 = freq
# counter_2 = freq
# counter_3 = freq
# counter_4 = freq

# Scale frame down for speed
scale = 2
'''
def callback_1(data):
    global br
    global counter_1
    global freq
    global pub_1
    global scale
    rospy.loginfo("in callback for pixel 1")

    counter_1 -= 1

    if(counter_1 <= 0):
        counter_1 = freq

        # Timing
        # start =  time.clock()

        # Reduce frame size by half for speed purposes
        frame = br.compressed_imgmsg_to_cv2(data, desired_encoding="bgr8")
        small_frame = cv2.resize(frame, (0, 0), fx=1/scale, fy=1/scale)
        rgb_small_frame = small_frame[:, :, ::-1]

        # Detect faces in frame
        face_locations = face_recognition.face_locations(rgb_small_frame)

        labels = []
        bboxes = []
        for face_location in face_locations:
            # rospy.loginfo("found a face at " + str(face_location))
            labels.append("A face")

        # Display the results
        for (top, right, bottom, left), name in zip(face_locations, labels):
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

            # Draw a box around the face
            # cv2.rectangle(frame, (left, top), (right, bottom), (0, 0, 255), 2)

            # Draw a label with a name below the face
            # cv2.rectangle(frame, (left, bottom - 35), (right, bottom), (0, 0, 255), cv2.FILLED)
            # font = cv2.FONT_HERSHEY_DUPLEX
            # cv2.putText(frame, name, (left + 6, bottom - 6), font, 1.0, (255, 255, 255), 1)

        # Display the resulting image
        # cv2.imshow('Video' + str(pixel_no+1), frame)
        # cv2.waitKey(1)

        # Publish results for visualisation node
        msg = Detected_Img()
        msg.header.stamp = data.header.stamp
        msg.img = br.cv2_to_compressed_imgmsg(frame)
        msg.bboxes = bboxes
        msg.labels = labels
        pub_1.publish(msg)

        # Timing
        # total =  time.clock() - start
        # rospy.loginfo("face detection took " + str(total) + " seconds")
'''
def callback_2(data):
    global br
    # global counter_2
    # global freq
    global pub_2
    global scale
    rospy.loginfo("in callback for pixel 2")

    # counter_2 -= 1
    # if(counter_2 <= 0):
        # counter_2 = freq

    # Reduce frame size by half for speed purposes
    frame = br.compressed_imgmsg_to_cv2(data, desired_encoding="bgr8")
    small_frame = cv2.resize(frame, (0, 0), fx=1/scale, fy=1/scale)
    rgb_small_frame = small_frame[:, :, ::-1]

    # Detect faces in frame
    face_locations = face_recognition.face_locations(rgb_small_frame)

    labels = []
    bboxes = []
    for face_location in face_locations:
        labels.append("A face")

    # Display the results
    for (top, right, bottom, left) in zip(face_locations):
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
'''
def callback_3(data):
    global br
    global counter_3
    global freq
    global pub_3
    global scale
    rospy.loginfo("in callback for pixel 3")

    # counter_3 -= 1
    # if(counter_3 <= 0):
    #     counter_3 = freq

    # Reduce frame size by half for speed purposes
    frame = br.compressed_imgmsg_to_cv2(data, desired_encoding="bgr8")
    small_frame = cv2.resize(frame, (0, 0), fx=1/scale, fy=1/scale)
    rgb_small_frame = small_frame[:, :, ::-1]

    # Detect faces in frame
    face_locations = face_recognition.face_locations(rgb_small_frame)

    labels = []
    bboxes = []
    for face_location in face_locations:
        labels.append("A face")

    # Display the results
    for (top, right, bottom, left) in zip(face_locations):
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
    global counter_4
    global freq
    global pub_4
    global scale
    rospy.loginfo("in callback for pixel 4")

    counter_4 -= 1

    if(counter_4 <= 0):
        counter_4 = freq

        # Reduce frame size by half for speed purposes
        frame = br.compressed_imgmsg_to_cv2(data, desired_encoding="bgr8")
        small_frame = cv2.resize(frame, (0, 0), fx=1/scale, fy=1/scale)
        rgb_small_frame = small_frame[:, :, ::-1]

        # Detect faces in frame
        face_locations = face_recognition.face_locations(rgb_small_frame)

        labels = []
        bboxes = []
        for face_location in face_locations:
            labels.append("A face")

        # Display the results
        for (top, right, bottom, left) in zip(face_locations):
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
'''
def main():
    rospy.loginfo("Initialising node...")
    rospy.init_node("face_detection_py")
    rospy.loginfo("Initialising node...")

    global br
    br = CvBridge()

    global pub_1
    pub_1  = rospy.Publisher("/pixel_1/camera0/image/detected", Detected_Img, queue_size=1)
    global pub_2
    pub_2  = rospy.Publisher("/pixel_2/camera0/image/detected", Detected_Img, queue_size=1)
    global pub_3
    pub_3  = rospy.Publisher("/pixel_3/camera0/image/detected", Detected_Img, queue_size=1)
    global pub_4
    pub_4  = rospy.Publisher("/pixel_4/camera0/image/detected", Detected_Img, queue_size=1)

    # rospy.Subscriber("/pixel_1/camera0/image/compressed", CompressedImage, callback_1, queue_size=1)
    rospy.Subscriber("/pixel_2/camera0/image/compressed", CompressedImage, callback_2, queue_size=1)
    # rospy.Subscriber("/pixel_3/camera0/image/compressed", CompressedImage, callback_3, queue_size=1)
    # rospy.Subscriber("/pixel_4/camera0/image/compressed", CompressedImage, callback_4, queue_size=1)

    rospy.spin()

    rospy.loginfo("Finished face_detection_py")


if __name__ == "__main__":
    main()
