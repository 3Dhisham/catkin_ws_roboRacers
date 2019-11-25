# !/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import UInt8
from std_msgs.msg import Int16
from autominy_msgs.msg import SpeedCommand, NormalizedSteeringCommand


def drive_at_speed(speed, steering):
    pub_steer.publish(value=steering)
    pub_speed.publish(value=speed)
    rospy.sleep(acceleration_time)
    pub_speed.publish(value=0.0)
    pub_steer.publish(value=0.0)


rospy.init_node("drive", anonymous=True)

# create subscribers and publishers
pub_speed = rospy.Publisher("/actuators/speed", SpeedCommand, queue_size=10)
pub_steer = rospy.Publisher("/actuators/steering_normalized", NormalizedSteeringCommand, queue_size=10)
rate = rospy.Rate(10)  # 10hz

iter_per_speed = 2
acceleration_time = 3  # in seconds
speed_list = [0.2]
res = {}
steering = 1.0
while not rospy.is_shutdown():
    for speed in speed_list:
        for i in range(iter_per_speed):
            rospy.loginfo("driving at speed of %f - steering with %f - run %d" % (speed, steering, i))
            drive_at_speed(speed, steering)
            rospy.loginfo(50 * "-")

    input("Driving done. press any key to exit...")