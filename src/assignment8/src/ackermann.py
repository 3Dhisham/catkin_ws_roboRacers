# !/usr/bin/env python2

import rospy
from autominy_msgs.msg import Speed, SpeedCommand, NormalizedSteeringCommand, SteeringAngle, NormalizedSpeedCommand
from nav_msgs.msg import Odometry
import math
from std_msgs.msg import Float32
from std_msgs.msg import String



class AckermannOdo:

    def __init__(self):
        # Subscribers
        self.sub_speed = rospy.Subscriber("/sensors/speed", Speed, self.callback_speed, queue_size=100)
        self.sub_steering = rospy.Subscriber("/sensors/steering", SteeringAngle, self.callback_steering , queue_size=100)
        self.sub_odometry = rospy.Subscriber("/sensors/localization/filtered_map", Odometry, self.callback_odometry, queue_size=100)

        # initial parameters
        self.is_loc_initialized = True
        self.speed = 0.0
        self.init_x_pose = 0.0
        self.init_y_pose = 0.0
        self.init_theta = 0.0
        
    def callback_steering(self, data):
        self.steering = data.value

    def callback_speed (self, data):
        self.speed = data.value

    def callback_odometry(self, data):
        if self.is_loc_initialized:
            self.init_x_pose = data.pose.pose.position.x
            self.init_y_pose = data.pose.pose.position.y
            self.init_theta = math.acos(data.pose.pose.orientation.w)

        self.is_loc_initialized = False

    def execute(self):
        # Sleep a few seconds to initialize node
        rospy.sleep(2.0)
        pub_odometry= rospy.Publisher("/roboRacers_odo", Odometry, queue_size=100) 
        ackermann_odo = AckermannOdo()
        rospy.sleep(2.0)

        # create local values from initial values
        old_x = ackermann_odo.init_x_pose
        old_y = ackermann_odo.init_y_pose
        old_theta = ackermann_odo.init_theta
        wheelbase = 0.27
        
        rate = rospy.Rate(100)

        while not rospy.is_shutdown():

            velocity = ackermann_odo.speed
            steering_angle = ackermann_odo.steering
            time_delta = 0.01

            # Equations
            x_dot = velocity * math.cos(old_theta)
            y_dot = velocity * math.sin(old_theta)
            theta_dot = (velocity / wheelbase) * math.tan(steering_angle)

            current_x = old_x + time_delta * x_dot
            current_y = old_y + time_delta * y_dot
            current_theta = old_theta + time_delta * theta_dot

            # create publisher and pub the new odometry
            odo_msg = Odometry()
            odo_msg.header.frame_id = "map"
            odo_msg.child_frame_id = "base_link"

            # -3.0 to create a relocation on the rviz plane (can be removed)
            odo_msg.pose.pose.position.x = current_x - 3.0
            odo_msg.pose.pose.position.y = current_y - 3.0
            odo_msg.pose.pose.position.z = 1.0

            odo_msg.pose.pose.orientation.w = math.cos(current_theta/2)
            odo_msg.pose.pose.orientation.z = math.sin(current_theta/2)

            pub_odometry.publish(odo_msg)

            # overwrite values for next run
            old_x = current_x
            old_y = current_y
            old_theta = current_theta

            rate.sleep()

        rospy.spin()



def main():
    try:
        rospy.init_node("ackermann_odo")
        ackermann = AckermannOdo()
        ackermann.execute()
    except rospy.ROSInterruptException:
        print("=== Failed ===")


if __name__ == '__main__':
    print(40*"-")
    print("   Publishing...   ")
    main()
    print("   Done   ")
    print(40*"-")
