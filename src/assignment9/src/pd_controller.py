# !/usr/bin/env python2

import rospy
from autominy_msgs.msg import Speed, SpeedCommand, NormalizedSteeringCommand, SteeringAngle, NormalizedSpeedCommand
from nav_msgs.msg import Odometry
import math
from std_msgs.msg import Float32
from std_msgs.msg import String
import tf


class PDController:

    def __init__(self):
        # Subscribers
        self.sub_speed = rospy.Subscriber("/sensors/speed", Speed, self.callback_speed, queue_size=100)
        self.sub_steering = rospy.Subscriber("/sensors/steering", SteeringAngle, self.callback_steering , queue_size=100)
        #self.sub_odometry = rospy.Subscriber("/sensors/localization/filtered_map", Odometry, self.callback_odometry, queue_size=100)
        self.sub_localize = rospy.Subscriber("/communication/gps/7", Odometry, self.callback_localize, queue_size=100)

        # Publishers
        self.pub_speed = rospy.Publisher("/actuators/speed", SpeedCommand, queue_size=10)
        self.pub_steer = rospy.Publisher("/actuators/steering_normalized", NormalizedSteeringCommand, queue_size=10)

        # initial parameters
        self.x_orientation = 0.0
        self.y_orientation = 0.0
        self.z_orientation = 0.0
        self.theta = 0.0
        self.steering_angle = 0.0
        self.speed = 0.0

        
    def callback_steering(self, data):
        self.steering_angle = data.value

    def callback_speed (self, data):
        self.speed = data.value


    def callback_localize(self, data):
            quaternion = data.pose.pose.orientation
            self.x_orientation = quaternion.x
            self.y_orientation = quaternion.y
            self.z_orientation = quaternion.z
            self.loc_array = [quaternion.w, self.x_orientation, self.y_orientation, self.z_orientation]
            self.theta = tf.transformations.euler_from_quaternion(self.loc_array)



    def execute(self, steering, speed, duration):
        # Sleep a few seconds to initialize node
        rospy.sleep(2.0)
        
        ''' Drive the car '''
        self.pub_steer.publish(NormalizedSpeedCommand(value=steering))
        self.pub_speed.publish(SpeedCommand(value=speed))
        #self.pub_speed.publish(SpeedCommand(value=0.0))
        pdc = PDController()

        rospy.sleep(duration)

        rate = rospy.Rate(100)

        while not rospy.is_shutdown():
            speed = 0.2
            theta = pdc.theta
            self.steering_angle = (theta/math.pi)

            # Drive
            self.pub_steer.publish(NormalizedSpeedCommand(value=self.steering_angle))
            self.pub_speed.publish(NormalizedSpeedCommand(value=speed))
            

            rate.sleep()

        
        self.pub_speed.publish(SpeedCommand(value=0.0))

        rospy.spin()



def main(steering, speed, duration):
    try:
        rospy.init_node("pd_controller")
        pdc = PDController()
        pdc.execute(steering, speed, duration)
    except rospy.ROSInterruptException:
        print("=== Failed ===")


if __name__ == '__main__':
    print(40*"-")
    print("   Publishing...   ")
    main(0.0, 0.3, 30)
    print("   Done   ")
    print(40*"-")
