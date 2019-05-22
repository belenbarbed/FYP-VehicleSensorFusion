#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import CompressedImage, Image, CameraInfo
import cv2
from cv_bridge import CvBridge
import numpy as np

def image_callback(data, args):
    global br
    pub_raw  = args[0]
    pub_info = args[1]
    pixel_no = args[2]
    if(data.data != []):
        # From compressed to raw
        rospy.loginfo("Image received from pixel-" + pixel_no)
        img = br.compressed_imgmsg_to_cv2(data, desired_encoding="bgr8")
        # time = str(data.header.stamp.secs) + ("00000000" if data.header.stamp.nsecs == 0 else str(data.header.stamp.nsecs))
        # cv2.imwrite("/home/soteris-group/phone_test/pixel_1/" + time + ".jpeg", img)
        msg = br.cv2_to_imgmsg(img)
        msg.header.stamp = data.header.stamp
        msg.encoding = "bgr8"
        pub_raw.publish(msg)

        # Camera info - projection matrix
        pixel_info = CameraInfo()
        pixel_info.P = [515.4,   0.0, 323.0, 0.0,
                          0.0, 518.7, 233.9, 0.0,
                          0.0,   0.0,   1.0, 0.0]
        pixel_info.header.stamp = data.header.stamp
        pub_info.publish(pixel_info)
    else:
        rospy.loginfo("Something went wrong")

def main():
    rospy.loginfo("Initialising node...")
    rospy.init_node("phone_streams_publisher")
    rospy.loginfo("Initialising node...")

    global br
    br = CvBridge()

    pub1_raw  = rospy.Publisher("/pixel_1/camera0/image/not_compressed", Image, queue_size=1)
    pub1_info = rospy.Publisher("/pixel_1/camera0/camera_info", CameraInfo, queue_size=1)
    pub2_raw  = rospy.Publisher("/pixel_2/camera0/image/not_compressed", Image, queue_size=1)
    pub2_info = rospy.Publisher("/pixel_2/camera0/camera_info", CameraInfo, queue_size=1)
    pub3_raw  = rospy.Publisher("/pixel_3/camera0/image/not_compressed", Image, queue_size=1)
    pub3_info = rospy.Publisher("/pixel_3/camera0/camera_info", CameraInfo, queue_size=1)
    pub4_raw  = rospy.Publisher("/pixel_4/camera0/image/not_compressed", Image, queue_size=1)
    pub4_info = rospy.Publisher("/pixel_4/camera0/camera_info", CameraInfo, queue_size=1)

    rospy.Subscriber("/pixel_1/camera0/image/compressed", CompressedImage, image_callback, (pub1_raw, pub1_info, "1"))
    rospy.Subscriber("/pixel_2/camera0/image/compressed", CompressedImage, image_callback, (pub2_raw, pub2_info, "2"))
    rospy.Subscriber("/pixel_3/camera0/image/compressed", CompressedImage, image_callback, (pub3_raw, pub3_info, "3"))
    rospy.Subscriber("/pixel_4/camera0/image/compressed", CompressedImage, image_callback, (pub4_raw, pub4_info, "4"))

    rospy.spin()

    rospy.loginfo("Finished phone_streams_publisher")


if __name__ == "__main__":
    main()
