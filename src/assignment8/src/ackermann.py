# !/usr/bin/env python2

import rospy
from autominy_msgs.msg import Speed, SpeedCommand, NormalizedSteeringCommand, SteeringAngle, NormalizedSpeedCommand
from nav_msgs.msg import Odometry
import math
from std_msgs.msg import Float32
from std_msgs.msg import String



class AckermannOdo:

    def __init__(self):
        ''' Publishers and Subscribers '''
        # Subscribers
        self.sub_speed = rospy.Subscriber("/sensors/speed", Speed, self.callback_speed, queue_size=10)
        self.sub_steering = rospy.Subscriber("/sensors/steering", SteeringAngle, self.callback_steering , queue_size=10)
        self.sub_odometry = rospy.Subscriber("/sensors/localization/filtered_map", Odometry, self.callback_odometry, queue_size=10)
        # Publishers
        self.pub_speed = rospy.Publisher("/actuators/speed", SpeedCommand, queue_size=10)
        self.pub_steer = rospy.Publisher("/actuators/steering_normalized", NormalizedSteeringCommand, queue_size=10)
        
        # initial parameters
        self.is_loc_initialized = True
        self.init_velocity = 0
        self.init_x_velo = 0
        self.init_y_velo = 0
        self.init_theta = 0
        
        self.rate = rospy.Rate(100)  # 100hz

    def callback_steering(self, data):
        self.steering = data.value

    def callback_speed (self, data):
        self.speed = data.value

    def callback_odometry(self, data):
        if self.is_loc_initialized:
            self.init_x_velo = data.pose.pose.position.x
            self.init_y_velo = data.pose.pose.position.y
            self.init_theta = math.acos(data.pose.pose.orientaion.w)

        self.is_loc_initialized = False

        print("initial x: ", self.init_x_velo)
        print("initial y: ", self.init_y_velo)
        print("initial theta: ", self.init_theta)

    def execute(self):
    	# Sleep a few seconds to initialize ndoe
        rospy.sleep(2.0)

        self.wheelbase = 0.27

        ackermann_odo = AckermannOdo()
        rospy.sleep(1.0)

        pub_odometry= rospy.Publisher("/roboRacers_odo", Odometry, queue_size=100) 

        # create local values from initial values
        old_y = ackermann_odo.init_y_velo
        old_x = ackermann_odo.init_x_velo
        old_theta = ackermann_odo.init_theta

        self.rate = rospy.Rate(100)

        while not rospy.is_shutdown:

            velocity = ackermann_odo.speed
            steering_angle = ackermann_odo.steering.value
            time_delta = 0.01

            # Equations
            new_x = old_x + (time_delta * (velocity * math.cos(old_theta)))
            new_y = old_y + (time_delta * (velocity * math.sin(old_theta)))
            new_theta = old_theta + (time_delta * (velocity / self.wheelbase * math.tan(steering_angle)))

            # create publsiher and pub the new odometry
            msg = Odometry()
            msg.header.frame_id = "map"
            msg.child_frame_id = "base_link"

            msg.pose.pose.position.x = new_x
            msg.pose.pose.position.y = new_y
            msg.pose.pose.position.z = 1

            msg.pose.pose.orientation.w = new_theta

            # publish to our new odometry topic
            pub_odometry.publish(msg)

            # overwrite values for next run
            old_x = new_x
            old_y = new_y
            old_theta = new_theta

            self.rate.sleep()

        rospy.spin()


def main():
    try:
        rospy.init_node("ackermann_odo")
        ackermann = AckermannOdo()
        ackermann.execute()
    except rospy.ROSInterruptException:
        print("=== Failed ===")


if __name__ == '__main__':
    print(40*"#") 
    main()
    print(40*"#")
