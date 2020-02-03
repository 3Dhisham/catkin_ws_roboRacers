#!/usr/bin/env python

import rospy
from std_msgs.msg import UInt8

def publisher():
    chosen_lane = 0

    while (chosen_lane < 1 or chosen_lane > 2):
        print ("please enter only 1 or 2 for changing lane")
        chosen_lane = int(input("input lane: "))
    print ("\nactual lane was chosen: " + str(chosen_lane))
    print ("\nplease restart node to choose again!")
    
    lane_pub = rospy.Publisher('/chosen_lane', UInt8, queue_size=10)
    rospy.init_node('chosen_lane_publisher', anonymous=True)
    rate = rospy.Rate(1)
    
    while not rospy.is_shutdown():
        lane_msg = UInt8(chosen_lane)        
        lane_pub.publish(lane_msg)
        rate.sleep()

if __name__ == '__main__':
    publisher()

