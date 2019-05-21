#!/usr/bin/env python

import rospy
from sensor_msgs.msg import CompressedImage
from phone_streams.msg import Framerates
import numpy as np

def counter_callback(data, args):
    global time
    global pub
    global counters
    pixel_no = int(args[0])

    counters[pixel_no] += 1

    if(time < data.header.stamp.secs):
        time = data.header.stamp.secs
        msg = Framerates()
        msg.header.stamp.secs = time
        msg.pixel_1 = counters[0]
        msg.pixel_2 = counters[1]
        msg.pixel_3 = counters[2]
        msg.pixel_4 = counters[3]

        rospy.loginfo("")
        rospy.loginfo("Framerates for second %d: ", time)
        rospy.loginfo("  pixel_1: %d fps", counters[0])
        rospy.loginfo("  pixel_2: %d fps", counters[1])
        rospy.loginfo("  pixel_3: %d fps", counters[2])
        rospy.loginfo("  pixel_4: %d fps", counters[3])
        rospy.loginfo("")

        counters = [0, 0, 0, 0]
        pub.publish(msg)
        

def main():
    rospy.loginfo("Initialising node...")
    rospy.init_node("phone_frames_counter")
    rospy.loginfo("Initialising node...")

    global counters
    counters = [0, 0, 0, 0]

    global pub
    pub  = rospy.Publisher("/pixels/framerates", Framerates, queue_size=1)

    global time
    time = rospy.get_rostime().secs

    rospy.Subscriber("/pixel_1/camera0/image/compressed", CompressedImage, counter_callback, ("0"))
    rospy.Subscriber("/pixel_2/camera0/image/compressed", CompressedImage, counter_callback, ("1"))
    rospy.Subscriber("/pixel_3/camera0/image/compressed", CompressedImage, counter_callback, ("2"))
    rospy.Subscriber("/pixel_4/camera0/image/compressed", CompressedImage, counter_callback, ("3"))

    rospy.spin()

    rospy.loginfo("Finished phone_frames_counter")


if __name__ == "__main__":
    main()
