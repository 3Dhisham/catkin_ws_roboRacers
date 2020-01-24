#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from autominy_msgs.msg import Speed, SteeringAngle, SpeedCommand, NormalizedSteeringCommand
from nav_msgs.msg import Odometry
import numpy
import math
import tf
import sys

pi = math.pi

class OdoMeasurer():
    def __init__(self):
        self.initial_pose = True
        self.initial_steering = True
        self.theta = 0 
        ceiling_cam_sub = rospy.Subscriber('/communication/gps/11', Odometry, self.ceiling_callback)
        steering_angle_sub = rospy.Subscriber('/sensors/steering', SteeringAngle, self.steering_callback)

    def ceiling_callback(self,data):
        self.theta = tf.transformations.euler_from_quaternion([data.pose.pose.orientation.w,data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z])[0]

    def steering_callback(self,data):
        self.steering= data.value


def error_signed(wanted_angle,theta):
    # let the angles wrap around 2pi and calculate smaller distance, left or right
    # TO DO: would be better computationally to wrap around pi as the angles
    # are given wrapped around pi
    # like this it was easier to think of for me

    wanted_angle  %= 2*pi 
    theta  %= 2*pi 

    right_dist = (2*pi - theta + wanted_angle) % (2 * pi)
    left_dist =  (2*pi - wanted_angle + theta) % (2 * pi)

    if right_dist < left_dist:
        return -right_dist #negative value for steering to right
    else:
        return left_dist  #positive for steering to left


# ---main--- 

if len(sys.argv) < 2:
    print("please enter desired angle in the coordinate system used by communication/gps")
    sys.exit()

rospy.init_node('my_odometry', anonymous=True)

# ---Publishers---
speed_pub = rospy.Publisher('/actuators/speed', SpeedCommand, queue_size=10)
steering_pub = rospy.Publisher('/actuators/speed', SpeedCommand, queue_size=10)
pub_steering = rospy.Publisher("/actuators/steering_normalized", NormalizedSteeringCommand, queue_size=100)

odo = OdoMeasurer()
rospy.sleep(1)

# constants
Kp = 3 
Kd = 0.5
wanted_angle = float(sys.argv[1])

# initialize loop variables
theta = odo.theta
old_error = abs(error_signed(wanted_angle, theta))
t = 0
error_diff = 0

# main loop
r = rospy.Rate(100)
while not rospy.is_shutdown():

    theta = odo.theta
    speed_pub.publish(value=0.2)
    
    # get steering with direction
    steering = error_signed(wanted_angle, theta) / math.pi
    # get absolute error
    error = abs(steering)
    
    # dont measure error too often, only every 16th loop
    # so the noise is not too big
    if (t == 0):
        error_diff = error - old_error
        old_error = error
    t = (t+1) % 16

    # control formula 
    steering_angle = Kp * steering + (Kd * error_diff) / math.pi

    pub_steering.publish(value=steering_angle)
    
    r.sleep()

rospy.spin()


