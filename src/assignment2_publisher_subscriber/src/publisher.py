#!/usr/bin/env python

import rospy
from autominy_msgs.msg import SpeedCommand, NormalizedSteeringCommand


def pubSpeed():

    #initialize node
    rospy.init_node('pubSpeed', anonymous=True)
    # steering publisher
    steeringPub = rospy.Publisher('/actuators/steering_normalized', NormalizedSteeringCommand, queue_size=10)
    # speed publisher
    speedPub = rospy.Publisher('/actuators/speed', SpeedCommand, queue_size=10)
    # frequency
    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        rospy.loginfo("Publishing...")
        #print("test")
        steeringPub.publish(value=1.0)
        speedPub.publish(value=0.1)
        rate.sleep()


if __name__ == '__main__':
    try:
        pubSpeed()
    except rospy.ROSInterruptException:
        pass
