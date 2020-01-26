#! /bin/env python

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PointStamped
import scipy.interpolate
import numpy as np
from scipy.spatial.distance import cdist
import math
import tf
import sys

from autominy_msgs.msg import Tick
from std_msgs.msg import String
from autominy_msgs.msg import Speed
from autominy_msgs.msg import SteeringAngle
from autominy_msgs.msg import SteeringCommand
from autominy_msgs.msg import SpeedCommand
from autominy_msgs.msg import NormalizedSteeringCommand
from nav_msgs.msg import Odometry

pi = math.pi




# -----------------------------------------------------------------
# -------------- from map.py --------------------------------------
# -----------------------------------------------------------------
class Lane:

    def __init__(self, support_points):
        self.support_points = support_points
        self.spline_x = scipy.interpolate.CubicSpline(self.support_points[:, 0], self.support_points[:, [1]], bc_type='periodic')
        self.spline_y = scipy.interpolate.CubicSpline(self.support_points[:, 0], self.support_points[:, [2]], bc_type='periodic')

    def length(self):
        return self.support_points[:, 0][-1]

    def interpolate(self, param):
        return np.column_stack((self.spline_x(param), self.spline_y(param)))

    def closest_point(self, point, precision=0.001, min_param=0.0, max_param=-1.0):
        step_size = 0.2
        if max_param < 0:
            max_param = self.length()
        closest_param = -1.0

        while step_size > precision:
            params = np.arange(min_param, max_param, step_size)
            points = self.interpolate(params)

            closest_index = cdist([point], points, 'sqeuclidean').argmin()
            closest_param = params[closest_index]
            min_param = max(min_param, closest_param - step_size)
            max_param = min(max_param, closest_param + step_size)
            step_size *= 0.5

        return self.interpolate(closest_param), closest_param

    def lookahead_point(self, point, lookahead_distance):
        closest_point, closest_param = self.closest_point(point)
        return self.interpolate(closest_param + lookahead_distance), closest_param + lookahead_distance


class Map:

    def __init__(self):
        self.lane_1 = np.load("lane1.npy")
        # Lane 2 is the outer lane
        self.lane_2 = np.load("lane2.npy")
        self.lanes = [
            Lane(self.lane_1[[0, 50, 209, 259, 309, 350, 409, 509, 639, 750, 848, 948, 1028, 1148, 1200, 1276], :]),
            Lane(self.lane_2[[0, 50, 100, 150, 209, 400, 600, 738, 800, 850, 900, 949, 1150, 1300, 1476], :])]


class MapVisualization:
    def __init__(self):
        self.map = Map()
        rospy.init_node("map_visualization")
        self.lane_pub = rospy.Publisher("lane", Marker, queue_size=10)
        self.clicked_point_subscriber = rospy.Subscriber("/clicked_point", PointStamped, self.on_click, queue_size=1)

        self.rate = rospy.Rate(1)

        while not rospy.is_shutdown():
            i = 0
            for lane in self.map.lanes:
                msg = Marker(type=Marker.LINE_STRIP, action=Marker.ADD)
                msg.header.frame_id = "map"
                msg.scale.x = 0.01
                msg.scale.y = 0.01
                msg.color.r = 1.0
                msg.color.a = 1.0
                msg.id = i

                for i in range(int(lane.length() * 100.0)):
                    inter = lane.interpolate(i / 100.0)
                    msg.points.append(Point(inter[0][0], inter[0][1], 0.0))
                i += 1

                self.lane_pub.publish(msg)

            self.rate.sleep()

    def on_click(self, point_msg):
        i = 2
        point = np.array([point_msg.point.x, point_msg.point.y])
        for lane in self.map.lanes:
            msg = Marker(type=Marker.SPHERE, action=Marker.ADD)
            msg.header.frame_id = "map"
            msg.scale.x = 0.1
            msg.scale.y = 0.1
            msg.scale.z = 0.1
            msg.color.b = 1.0
            msg.color.a = 1.0
            msg.id = i

            p, param = lane.closest_point(point)
            msg.pose.position.x = p[0][0]
            msg.pose.position.y = p[0][1]

            i += 1

            self.lane_pub.publish(msg)

            msg.color.b = 0.0
            msg.color.g = 1.0
            p, param = lane.lookahead_point(point, 0.5)
            msg.pose.position.x = p[0][0]
            msg.pose.position.y = p[0][1]
            msg.id = i

            i += 1

            self.lane_pub.publish(msg)



# ------------------------------------------------------------------------------
# ----------------- from U9 ----------------------------------------------------
# ------------------------------------------------------------------------------

class OdoMeasurer:
    
    def __init__(self):
        # Subscribers
        self.sub_speed = rospy.Subscriber("/sensors/speed", Speed, self.callback_speed, queue_size=100)
        self.sub_steering = rospy.Subscriber("/sensors/steering", SteeringAngle, self.callback_steering, queue_size=100)
        self.sub_localize = rospy.Subscriber("/communication/gps/15", Odometry, self.callback_localize, queue_size=100)
        self.sub_filteredMap = rospy.Subscriber("/sensors/localization/filtered_map", Odometry, self.callback_localize, queue_size=100)

        # Publishers
        self.pub_speed = rospy.Publisher("/actuators/speed", SpeedCommand, queue_size=10)
        self.pub_steer = rospy.Publisher("/actuators/steering_normalized", NormalizedSteeringCommand, queue_size=10)
        # initial parameters
        self.x_orientation = 0.0
        self.y_orientation = 0.0
        self.z_orientation = 0.0
        self.position = Point(x=0, y=0, z=0)
        self.loc_array = []
        self.theta = 0.0
        self.speed = 0.0
        self.steering_angle = 0.0
    
    '''--------------- Callbacks ---------------'''

    def callback_speed(self, data):
        self.speed = data.value


    def callback_steering(self, data):
        self.steering_angle = data.value

    def callback_localize(self, data):
        quaternion = data.pose.pose.orientation
        self.x_orientation = quaternion.x
        self.y_orientation = quaternion.y
        self.z_orientation = quaternion.z
        self.loc_array = [quaternion.w, self.x_orientation, self.y_orientation, self.z_orientation]
        self.theta = tf.transformations.euler_from_quaternion(self.loc_array)[0]
        self.position = data.pose.pose.position


    def error_signed(self, wanted_angle, theta):
        # let the angles wrap around 2pi and calculate smaller distance, left or right
        # TO DO: would be better computationally to wrap around pi as the angles
        # are given wrapped around pi
        # like this it was easier to think of for me

        wanted_angle  %= 2*pi 
        theta  %= 2*pi 

        right_steer_val = (2*pi - theta + wanted_angle) % (2 * pi)
        left_steer_val =  (2*pi - wanted_angle + theta) % (2 * pi)

        if right_steer_val < left_steer_val:
            return -right_steer_val #negative value for steering to right
        else:
            return left_steer_val  #positive for steering to left


# -----------------------------------------------------------------
# ----------------- main ------------------------------------------
# -----------------------------------------------------------------

if __name__ == "__main__":
    mapVisual = MapVisualization()
    odo = OdoMeasurer()
    rospy.init_node('my_odometry', anonymous=True)
    rospy.sleep(1)

    # constants
    Kp = 3 
    Kd = 0.5
    wanted_angle = float(sys.argv[1])

    # initialize loop variables
    theta = odo.theta
    old_error = abs(odo.error_signed(wanted_angle, theta))
    t = 0
    error_diff = 0

    # main loop
    r = rospy.Rate(100)
    while not rospy.is_shutdown():

        theta = odo.theta
        odo.pub_speed.publish(value=0.2)
    
        # get steering with direction
        steering = odo.error_signed(wanted_angle, theta) / math.pi
        # get absolute error
        error = abs(steering)
    
        # dont measure error too often, only every 16th loop
        # so the noise is not too big
        if t == 0:
            error_diff = error - old_error
            old_error = error
            t = (t+1) % 16

        # control formula 
        steering_angle = Kp * steering + (Kd * error_diff) / math.pi

        odo.pub_steer.publish(value=steering_angle)
    
        r.sleep()

    rospy.spin()






