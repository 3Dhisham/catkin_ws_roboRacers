# !/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import UInt8
from std_msgs.msg import Int16
from autominy_msgs.msg import SpeedCommand, NormalizedSteeringCommand, SteeringAngle, Tick
from nav_msgs.msg import Odometry


def read_points(odometry):
    global x_point
    global y_point
    global z_point

    print("Recording Points...")
    x_point = odometry.pose.pose.position.x
    y_point = odometry.pose.pose.position.y
    z_point = odometry.pose.pose.position.z

    print("=======================================================")
    print("x: ", x_point)
    print("y: ", y_point)
    print("z: ", z_point)
    print("=======================================================")


class WheelSensorCalibration:
    # Init Coordinate points
    x_point = 0.0
    y_point = 0.0
    z_point = 0.0

    '''def tick_callback(self):
        rospy.loginfo("TICK")
'''
    def drive_at_speed(self):
        print("Car Is Moving...")
        self.pub_steer.publish(value=1.0)
        #   rospy.sleep(execution_time)
        self.pub_speed.publish(value=0.1)
        rospy.sleep(2)
        print("Car Is Stopping...")
        self.pub_speed.publish(value=0.0)
        self.pub_steer.publish(value=0.0)
        print("Car Has Stopped...")

    def __init__(self):
        rospy.init_node("drive", anonymous=True)
        # create subscribers and publishers

        # This subscribes to the ceiling camera topic which prints out the points based on car location
        self.localization_subscriber = rospy.Subscriber("/communication/gps/7", Odometry, read_points, queue_size=10)
        # self.ticks = rospy.Subscriber("/sensors/arduino/ticks", Tick, self.tick_callback, queue_size=10)
        # self.angle_subscriber = rospy.Subscriber("/sensors/arduino/steering_angle", SteeringFeedback, steering_ang
        # le, queue_size=10)

        # The spin() function runs the ros node in loops
        # rospy.spin()

        self.pub_speed = rospy.Publisher("/actuators/speed", SpeedCommand, queue_size=10)
        self.pub_steer = rospy.Publisher("/actuators/steering_normalized", NormalizedSteeringCommand, queue_size=10)

        self.rate = rospy.Rate(10)  # 10hz


if __name__ == '__main__':
    try:
        rospy.loginfo("Starting Node")
        rospy.loginfo(50 * "-")
        calibration = WheelSensorCalibration()
        calibration.drive_at_speed()
        read_points(odometry=read_points())
        rospy.loginfo(50 * "-")
        rospy.loginfo("Finished Executing")
        rospy.loginfo(50 * "-")

        # speed = 0.0
        # steering = 1.0
        # rospy.loginfo("driving at speed of %f - steering with %f" % (speed, steering))
        # drive_at_speed(speed, steering)
        # rospy.loginfo(50 * "-")
    except rospy.ROSInterruptException:
        pass
