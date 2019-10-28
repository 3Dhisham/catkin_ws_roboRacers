#!/usr/bin/env python
import rospy
from autominy_msgs.msg import SpeedCommand, NormalizedSteeringCommand

# Callback function that prints speed value
def speed_value(msg):
    print(msg.value)
    print(" ----------------------------- ")


def subscriber():
    rospy.init_node('subscriber', anonymous=True)
    rospy.Subscriber('/sensors/speed', SpeedCommand, speed_value)
    rospy.spin()


if __name__ == '__main__':
    try:
        subscriber()
    except rospy.ROSInterruptException:
        pass
