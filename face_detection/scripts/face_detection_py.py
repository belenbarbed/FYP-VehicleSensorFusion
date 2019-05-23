#!/usr/bin/env python3

import cv2
from cv_bridge import CvBridge
import face_recognition
import numpy as np
import rospy
import time

from sensor_msgs.msg import Image
from face_detection.msg import Detected_Img

# Detect faces on 1-in-freq frames
freq = 2
counters = [freq, freq, freq, freq]

# Scale frame down for speed
scale = 2

def callback(data, args):
    global br
    global counters
    global freq
    global scale
    pixel_no = int(args[0])
    pub = args[1]
    rospy.loginfo("in callback for pixel " + str(pixel_no+1))

    counters[pixel_no] -= 1

    if(counters[pixel_no] <= 0):
        counters[pixel_no] = freq

        # Timing
        start =  time.clock()

        # Reduce frame size by half for speed purposes
        frame = br.imgmsg_to_cv2(data, desired_encoding="bgr8")
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

            # Draw a box around the face
            cv2.rectangle(frame, (left, top), (right, bottom), (0, 0, 255), 2)

            # Draw a label with a name below the face
            cv2.rectangle(frame, (left, bottom - 35), (right, bottom), (0, 0, 255), cv2.FILLED)
            font = cv2.FONT_HERSHEY_DUPLEX
            cv2.putText(frame, name, (left + 6, bottom - 6), font, 1.0, (255, 255, 255), 1)

        # Display the resulting image
        cv2.imshow('Video' + str(pixel_no+1), frame)
        cv2.waitKey(1)

        # Publish results for visualisation node
        msg = Detected_Img()
        msg.header  = data.header
        msg.img = frame
        # msg.bboxes = ?
        msg.labels = labels
        pub.publish(msg)

        # Timing
        total =  time.clock() - start
        rospy.loginfo("face detection took " + str(total) + " seconds")

def main():
    rospy.loginfo("Initialising node...")
    rospy.init_node("face_detection_py")
    rospy.loginfo("Initialising node...")

    global br
    br = CvBridge()

    pub1  = rospy.Publisher("/pixel_1/camera0/image/detected", Detected_Img, queue_size=1)
    pub2  = rospy.Publisher("/pixel_2/camera0/image/detected", Detected_Img, queue_size=1)
    pub3  = rospy.Publisher("/pixel_3/camera0/image/detected", Detected_Img, queue_size=1)
    pub4  = rospy.Publisher("/pixel_4/camera0/image/detected", Detected_Img, queue_size=1)

    rospy.Subscriber("/pixel_1/camera0/image/not_compressed", Image, callback, ("0", pub1))
    rospy.Subscriber("/pixel_2/camera0/image/not_compressed", Image, callback, ("1", pub2))
    rospy.Subscriber("/pixel_3/camera0/image/not_compressed", Image, callback, ("2", pub3))
    rospy.Subscriber("/pixel_4/camera0/image/not_compressed", Image, callback, ("3", pub4))

    rospy.spin()

    rospy.loginfo("Finished face_detection_py")


if __name__ == "__main__":
    main()
