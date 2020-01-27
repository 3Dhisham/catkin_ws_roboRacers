#!/usr/bin/env python

import rospy
import tf
import numpy as np
import sys
import os
import scipy.interpolate
from timeit import default_timer as timer
from std_msgs.msg import UInt8
from autominy_msgs.msg import SpeedCommand
from autominy_msgs.msg import Speed
from autominy_msgs.msg import NormalizedSteeringCommand
from autominy_msgs.msg import SteeringCommand
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from visualization_msgs.msg import Marker

global wanted_angle, steering, old_error, time, pos, spline1_x, spline1_y, wanted_point
global pMarker, old_seq, chosen_lane, spline2_x, spline2_y, offset, kd, kp
global curve_speed, straight_speed
curve_speed = 0.2
straight_speed = 0.4
steering = 0.0
wanted_angle = 0.0
old_error = 0.0
kd = 0.5
kp = 2.0
wanted_point = np.array([5.94,4.26])
pos = [0,0]
pMarker = np.array([0,0])
old_seq = 0
chosen_lane = 1 #default

#--------------------------------------------
#-------------- 1st Lane --------------------
#--------------------------------------------
lane1 = np.load("lane1.npy")
offset = int(len(lane1) / 20) 
points = lane1[[0, 50, 209, 259, 309, 350, 409, 509, 639, 750, 848, 948, 1028, 1148, 1200, 1276], :]
points_np = np.array(points)
arc1_array = points_np[:,0]
array1_x = points_np[:,1]
array1_y = points_np[:,2]
spline1_x = scipy.interpolate.CubicSpline(arc1_array, array1_x, bc_type='periodic')
spline1_y = scipy.interpolate.CubicSpline(arc1_array, array1_y, bc_type='periodic')

#--------------------------------------------
#-------------- 2nd Lane --------------------
#--------------------------------------------
lane2 = np.load("lane2.npy")
points2 = lane2[[0, 50, 100, 150, 209, 400, 600, 738, 800, 850, 900, 949, 1150, 1300, 1476], :]
points_np2 = np.array(points2)
arc2_array = points_np2[:,0]
array2_x = points_np2[:,1]
array2_y = points_np2[:,2]

spline2_x = scipy.interpolate.CubicSpline(arc2_array, array2_x, bc_type='periodic')
spline2_y = scipy.interpolate.CubicSpline(arc2_array, array2_y, bc_type='periodic')



def points_distance(point1, point2):
    return(np.linalg.norm(point1 - point2))

def lookahead_point(spline_x, spline_y, range,  point, offset):
    rangeMin = 0
    rangeMax = range
    d = 0
    i = 13
    while(i > 0):
        rangeMid = (rangeMin + rangeMax) / 2
        half_dist = np.abs(rangeMid - rangeMin) / 2
        point_low = np.array([spline_x(rangeMid - half_dist), spline_y(rangeMid - half_dist)])
        point_high = np.array([spline_x(rangeMid + half_dist), spline_y(rangeMid + half_dist)])

        if(points_distance(point_low, point) < points_distance(point_high, point)):
            rangeMax = rangeMid
            result = [point_low[0], point_low[1]]
            d = points_distance(result, point)
        else:
            rangeMin = rangeMid
            result = [point_high[0], point_high[1]]
            d = points_distance(result, point)
        i = i-1
    return np.array([spline_x(rangeMid + offset), spline_y(rangeMid + offset)])

#--------------------------------------------
#-------------- draw a point ----------------
#--------------------------------------------
def point_marker_properties(Point, RGBColor):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.color.r = RGBColor[0]
        marker.color.g = RGBColor[1]
        marker.color.b = RGBColor[2]
        marker.pose.position.x = Point[0]
        marker.pose.position.y = Point[1]
        marker.pose.position.z = 0.0
        marker.points = []
        return marker

#--------------------------------------------
#----------- odometry controller ------------
#--------------------------------------------
def callback_controller(msg):
    global wanted_angle, steering, old_error, time, pos, spline1_x, spline1_y, wanted_point, pMarker, old_seq, chosen_lane, spline2_x, spline2_y, offset, kd, kp
    global curve_speed, straight_speed
    new_seq = msg.header.seq
    print("curennt lane: " + str(chosen_lane))

    if(new_seq > old_seq):        
        pos[0] = msg.pose.pose.position.x
        pos[1] = msg.pose.pose.position.y
        ori = msg.pose.pose.orientation
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])    
        angle = yaw

        if(chosen_lane == 1): # 1st lane
            wanted_point = lookahead_point(spline1_x, spline1_y, 12.80, np.array(pos), 0.5)
        else: # 2nd lane
            wanted_point = lookahead_point(spline2_x, spline2_y, 14.76, np.array(pos), 0.5)

        pMarker = wanted_point
        wanted_point = wanted_point - pos
        rotation_matrix = np.array(( (np.cos(-angle), -np.sin(-angle)), (np.sin(-angle), np.cos(-angle)) ))
        wanted_point_carview = rotation_matrix.dot(np.array(wanted_point)) 
        wanted_angle = np.arctan(wanted_point_carview[1] / wanted_point_carview[0])

        new_error = wanted_angle
        error_diff = new_error - old_error 
        control_value = kp*new_error + kd*error_diff       
        steering = control_value
        old_error = new_error
    old_seq = new_seq

#--------------------------------------------
#-------- lane change controller ------------
#--------------------------------------------
def callback_lane_change(msg):
    global chosen_lane
    chosen_lane = msg.data

#--------------------------------------------
#-------------- main function ---------------
#--------------------------------------------
def publisher(arg):
    global wanted_angle, steering, old_error, time, pos, spline1_x, spline1_y, wanted_point, pMarker, old_seq, chosen_lane, spline2_x, spline2_y
    speed_msg = SpeedCommand()
    steering_msg = NormalizedSteeringCommand()
    rospy.init_node('publisher', anonymous=True)

    rospy.Subscriber("/sensors/localization/filtered_map", Odometry, callback_controller)
    rospy.Subscriber("/chosen_lane", UInt8, callback_lane_change)
    #speed_sub = rospy.Subscriber("/sensors/speed")

    steering_pub = rospy.Publisher("/actuators/steering_normalized", NormalizedSteeringCommand, queue_size=10)
    speed_pub = rospy.Publisher("/actuators/speed", SpeedCommand, queue_size=10)
    closest_point_pub = rospy.Publisher('my_closest_point', Marker, queue_size = 1)
    spline_pub = rospy.Publisher('my_spline', Marker, queue_size=1)
    
    rate = rospy.Rate(100)
    rospy.sleep(2.0)
    while not rospy.is_shutdown():
        #print("wanted_angle " + str(wanted_angle))
        #print("Steering: " + str(steering))
        #print("type of Steering: " + str(type (steering)))
        steering_msg.value = float(steering)
        speed_msg.value = 0.5
        steering_pub.publish(steering_msg)
        speed_pub.publish(speed_msg)
        point_marker = point_marker_properties(pMarker, [0.0, 1.0, 0.0])
        closest_point_pub.publish(point_marker)

        rate.sleep()

if __name__ == '__main__':
    publisher(rospy.myargv(argv=sys.argv))

