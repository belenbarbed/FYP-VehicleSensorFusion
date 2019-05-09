#!/usr/bin/env python

import rospy
from sensor_msgs.msg import CompressedImage, Image
import cv2
from cv_bridge import CvBridge
import numpy as np

def pixel_1_callback(data):
    global br
    global pub1
    if(data.data != []):
        rospy.loginfo("Image received from pixel_1")
        img = br.compressed_imgmsg_to_cv2(data, desired_encoding="bgr8")
        # time = str(data.header.stamp.secs) + ("00000000" if data.header.stamp.nsecs == 0 else str(data.header.stamp.nsecs))
        # cv2.imwrite("/home/soteris-group/phone_test/pixel_1/" + time + ".jpeg", img)
        msg = br.cv2_to_imgmsg(img)
        msg.header.stamp.secs = data.header.stamp.secs
        msg.header.stamp.nsecs = data.header.stamp.nsecs
        msg.encoding = "bgr8"
        pub1.publish(msg)
    else:
        rospy.loginfo("Something went wrong")

def phone_5_callback(data):
    global br
    global pub5
    if(data.data != []):
        rospy.loginfo("Image received from phone_5")
        img = br.compressed_imgmsg_to_cv2(data, desired_encoding="bgr8")
        # time = str(data.header.stamp.secs) + str(data.header.stamp.nsecs)
        # cv2.imwrite("/home/soteris-group/phone_test/phone_5/" + time + ".jpeg", img)
        msg = br.cv2_to_imgmsg(img)
        msg.header.stamp.secs = data.header.stamp.secs
        msg.header.stamp.nsecs = data.header.stamp.nsecs
        msg.encoding = "bgr8"
        pub5.publish(msg)
    else:
        rospy.loginfo("Something went wrong")

def main():
    rospy.loginfo("Initialising node...")
    rospy.init_node("camera_test")
    rospy.loginfo("Initialising node...")

    global br
    br = CvBridge()

    global pub1
    pub1 = rospy.Publisher("/pixel_1/camera0/image/not_compressed", Image, queue_size=10)
    global pub5
    pub5 = rospy.Publisher("/phone_5/camera0/image/not_compressed", Image, queue_size=10)

    rospy.Subscriber("/pixel_1/camera0/image/compressed", CompressedImage, pixel_1_callback)
    rospy.Subscriber("/phone_5/camera0/image/compressed", CompressedImage, phone_5_callback)
    rospy.spin()

    rospy.loginfo("Finished camera_test")


if __name__ == "__main__":
    main()
