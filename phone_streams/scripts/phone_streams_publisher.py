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

def pixel_2_callback(data):
    global br
    global pub2
    if(data.data != []):
        rospy.loginfo("Image received from pixel_2")
        img = br.compressed_imgmsg_to_cv2(data, desired_encoding="bgr8")
        # time = str(data.header.stamp.secs) + ("00000000" if data.header.stamp.nsecs == 0 else str(data.header.stamp.nsecs))
        # cv2.imwrite("/home/soteris-group/phone_test/pixel_2/" + time + ".jpeg", img)
        msg = br.cv2_to_imgmsg(img)
        msg.header.stamp.secs = data.header.stamp.secs
        msg.header.stamp.nsecs = data.header.stamp.nsecs
        msg.encoding = "bgr8"
        pub1.publish(msg)
    else:
        rospy.loginfo("Something went wrong")

def pixel_3_callback(data):
    global br
    global pub3
    if(data.data != []):
        rospy.loginfo("Image received from pixel_3")
        img = br.compressed_imgmsg_to_cv2(data, desired_encoding="bgr8")
        # time = str(data.header.stamp.secs) + ("00000000" if data.header.stamp.nsecs == 0 else str(data.header.stamp.nsecs))
        # cv2.imwrite("/home/soteris-group/phone_test/pixel_3/" + time + ".jpeg", img)
        msg = br.cv2_to_imgmsg(img)
        msg.header.stamp.secs = data.header.stamp.secs
        msg.header.stamp.nsecs = data.header.stamp.nsecs
        msg.encoding = "bgr8"
        pub1.publish(msg)
    else:
        rospy.loginfo("Something went wrong")

def pixel_4_callback(data):
    global br
    global pub4
    if(data.data != []):
        rospy.loginfo("Image received from pixel_4")
        img = br.compressed_imgmsg_to_cv2(data, desired_encoding="bgr8")
        # time = str(data.header.stamp.secs) + ("00000000" if data.header.stamp.nsecs == 0 else str(data.header.stamp.nsecs))
        # cv2.imwrite("/home/soteris-group/phone_test/pixel_4/" + time + ".jpeg", img)
        msg = br.cv2_to_imgmsg(img)
        msg.header.stamp.secs = data.header.stamp.secs
        msg.header.stamp.nsecs = data.header.stamp.nsecs
        msg.encoding = "bgr8"
        pub1.publish(msg)
    else:
        rospy.loginfo("Something went wrong")

# DEBUG: remove this test phone
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
    rospy.init_node("phone_streams_publisher")
    rospy.loginfo("Initialising node...")

    global br
    br = CvBridge()

    global pub1
    pub1 = rospy.Publisher("/pixel_1/camera0/image/not_compressed", Image, queue_size=10)
    global pub2
    pub1 = rospy.Publisher("/pixel_2/camera0/image/not_compressed", Image, queue_size=10)
    global pub3
    pub1 = rospy.Publisher("/pixel_3/camera0/image/not_compressed", Image, queue_size=10)
    global pub4
    pub1 = rospy.Publisher("/pixel_4/camera0/image/not_compressed", Image, queue_size=10)

    # DEBUG: remove this test phone
    global pub5
    pub5 = rospy.Publisher("/phone_5/camera0/image/not_compressed", Image, queue_size=10)

    rospy.Subscriber("/pixel_1/camera0/image/compressed", CompressedImage, pixel_1_callback)
    rospy.Subscriber("/pixel_2/camera0/image/compressed", CompressedImage, pixel_2_callback)
    rospy.Subscriber("/pixel_3/camera0/image/compressed", CompressedImage, pixel_3_callback)
    rospy.Subscriber("/pixel_4/camera0/image/compressed", CompressedImage, pixel_4_callback)

    # DEBUG: remove this test phone
    rospy.Subscriber("/phone_5/camera0/image/compressed", CompressedImage, phone_5_callback)
    
    rospy.spin()

    rospy.loginfo("Finished phone_streams_publisher")


if __name__ == "__main__":
    main()
