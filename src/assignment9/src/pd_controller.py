# !/usr/bin/env python2

import rospy
from autominy_msgs.msg import Speed, SpeedCommand, NormalizedSteeringCommand, SteeringAngle, NormalizedSpeedCommand
from nav_msgs.msg import Odometry
import math
import tf


class PDController:

    def __init__(self):
        # Subscribers
        self.sub_speed = rospy.Subscriber("/sensors/speed", Speed, self.callback_speed, queue_size=100)
        self.sub_steering = rospy.Subscriber("/sensors/steering", SteeringAngle, self.callback_steering, queue_size=100)
        self.sub_localize = rospy.Subscriber("/communication/gps/15", Odometry, self.callback_localize, queue_size=100)
        # self.sub_odometry = rospy.Subscriber("/sensors/localization/filtered_map", Odometry, self.callback_odometry, queue_size=100)

        # Publishers
        self.pub_speed = rospy.Publisher("/actuators/speed", SpeedCommand, queue_size=10)
        self.pub_steer = rospy.Publisher("/actuators/steering_normalized", NormalizedSteeringCommand, queue_size=10)
        # initial parameters
        self.x_orientation = 0.0
        self.y_orientation = 0.0
        self.z_orientation = 0.0
        self.loc_array = []
        self.theta = 0.0
        self.speed = 0.0
        self.steering_angle = 0.0

    '''--------------- Callbacks ---------------'''

    def callback_steering(self, data):
        self.steering_angle = data.value

    def callback_speed(self, data):
        self.speed = data.value

    def callback_localize(self, data):
        quaternion = data.pose.pose.orientation
        self.x_orientation = quaternion.x
        self.y_orientation = quaternion.y
        self.z_orientation = quaternion.z
        self.loc_array = [quaternion.w, self.x_orientation, self.y_orientation, self.z_orientation]
        self.theta = tf.transformations.euler_from_quaternion(self.loc_array)[0]

    '''--------------- Execution ---------------'''

    def drive(self, desired_orientation):

        theta = self.theta
        print("Theta is: ", theta)
        print("-" * 50)

        if desired_orientation == 0:
            # Will return a value between 0 and 1 for a steering angle
            self.steering_angle = (theta / math.pi)
        elif desired_orientation == 180:
            if theta < 0:
                self.steering_angle = (math.pi + theta) / math.pi
            else:
                self.steering_angle = (math.pi - theta) / -math.pi
        # To make sure that a steering input is being published
        else:
            self.steering_angle = theta / math.pi

        return self.steering_angle


    def execute(self, desired_orientation, speed):
        # Sleep a few seconds to initialize node
        rospy.sleep(2.0)
        rate = rospy.Rate(100)

        while not rospy.is_shutdown():

            steering_angle = self.drive(desired_orientation)
            self.pub_steer.publish(NormalizedSteeringCommand(value=steering_angle))
            self.pub_speed.publish(value=speed)

            rate.sleep()

        print("STOPPED")

        rospy.spin()


def main(desired_orientation, speed):
    try:
        rospy.init_node("pd_controller")
        pdc = PDController()
        pdc.execute(desired_orientation, speed)
    except rospy.ROSInterruptException:
        print("=== Failed ===")
        pdc.pub_speed.publish(value=0.0)


if __name__ == '__main__':
    print(40 * "-")
    print("   Starting...   ")
    pdc = PDController()
    print(" ")
    desired_angle=input("Enter desired orientation. For zero type 0, for Pi type 180: ")
    main(desired_angle, 0.2)
    pdc.pub_speed.publish(value=0.0)
    pdc.pub_steer.publish(value=0.0)    
    print("   Done   ")
    print(40 * "-")
