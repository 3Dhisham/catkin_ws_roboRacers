import rospy
import numpy as np
import math
import tf.transformations

from autominy_msgs.msg import SpeedCommand, SteeringCommand
from visualization_msgs.msg import Marker
from std_msgs.msg import UInt8
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from map import Map


class ObstacleAvoidance:

    def __init__(self):
        rospy.init_node("obstacle_avoidance")
        
        self.pub_speed = rospy.Publisher("/control/speed", SpeedCommand, queue_size=100)
        self.pub_steering = rospy.Publisher("/control/steering", SteeringCommand, queue_size=100)
        self.pub_lookahead = rospy.Publisher("/lookahead", Marker, queue_size=100)
        
        self.sub_localization = rospy.Subscriber("/sensors/localization/filtered_map", Odometry, self.callback_localization, queue_size=100)
        self.sub_lane = rospy.Subscriber("/lane", UInt8, self.callback_lane, queue_size=100)
        self.sub_scan = rospy.Subscriber("/sensors/rplidar/scan", LaserScan, self.callback_scanner, queue_size=100)

        
        self.rate = rospy.Rate(10)
        self.timer = rospy.Timer(rospy.Duration.from_sec(0.3), self.callback_control)
        
        self.map = Map()
        # lane flag
        self.lane = 1

        self.pose = Odometry()
        self.wanted_angle = 0.0

        rospy.on_shutdown(self.callback_shutdown)

        while not rospy.is_shutdown():
            self.rate.sleep()
            
    def on_steering(self, msg):
        self.wanted_angle = msg.value

    def callback_localization(self, msg):
        self.pose = msg

    def callback_lane(self, msg):
        self.lane = msg.data

    def is_obstacle_on_lane(self, lane, point_cloud):
        obstacles = 0
        position_x = self.pose.pose.pose.position.x
        position_y = self.pose.pose.pose.position.y
        position = np.array([position_x, position_y])
        min_param = lane.closest_point(position)
        max_param = min_param + 1.0

        for point in point_cloud.T:
            point = point.flatten()
                                            # point, precision, min, max
            closest_pnt = lane.closest_point(point, 0.1, min_param, max_param)
            distance = np.sqrt(np.sum(np.square(closest_pnt - point)))
            # 25cm threshold
            if distance < 0.25:
                obstacles += 1
                if obstacles > 10:
                    return True
        return False

    def callback_scanner(self, msg):
        points_ahead = int(0.25 / msg.angle_increment)
        alpha = msg.angle_min
        point_cloud = np.empty([4, 0])
        i = 0
        for range_ in msg.ranges:
            alpha += msg.angle_increment

            if range_ < 1.2 and (i < points_ahead or i > len(msg.ranges) - points_ahead):
                p = np.array([range_ * np.cos(alpha) - 0.18, range_ * np.sin(alpha), 0.0, 1.0])
                #stack into 2D array
                point_cloud = np.column_stack([point_cloud, p])

            i += 1

        orientation_x = self.pose.pose.pose.orientation.x
        orientation_y = self.pose.pose.pose.orientation.y
        orientation_z = self.pose.pose.pose.orientation.z
        orientation_w = self.pose.pose.pose.orientation.w
        
        quaternion = [orientation_x, orientation_y, orientation_z, orientation_w]
        yaw = tf.transformations.euler_from_quaternion(quaternion)

        # rotate
        yaw += np.pi

        position_x = self.pose.pose.pose.position.x
        position_y = self.pose.pose.pose.position.y
        
        transform = np.array([
            [np.cos(yaw), -np.sin(yaw), 0.0, position_x],
            [np.sin(yaw), np.cos(yaw), 0.0, position_y],
            [0.0        , 0.0        , 1.0, 0.0],
            [0.0        , 0.0        , 0.0, 1.0]])

        point_cloud = np.matmul(transform, point_cloud)
        point_cloud = point_cloud[[0, 1], :]

        # obstacle-on-lane flags
        is_lane_0_occupied = self.is_obstacle_on_lane(self.map.lanes[0], point_cloud)
        is_lane_1_occupied = self.is_obstacle_on_lane(self.map.lanes[1], point_cloud)

        if self.lane == -1:
            if not is_lane_0_occupied:
                self.lane = 0
            if not is_lane_1_occupied:
                self.lane = 1

        if self.lane == 1 and is_lane_1_occupied:
            self.lane = 0

        if self.lane == 0 and is_lane_0_occupied:
            self.lane = 1

        if is_lane_0_occupied and is_lane_1_occupied:
            print("stopping, obstacles blocking path.")
            self.lane = -1

    def callback_control(self, tmr):
        # stop of lanes are blocked
        if self.lane == -1:
            speed_msg = SpeedCommand()
            speed_msg.value = 0.0
            self.pub_speed.publish(speed_msg)
            return

        lane = self.map.lanes[self.lane]
        
        position_x = self.pose.pose.pose.position.x
        position_y = self.pose.pose.pose.position.y

        orientation_x = self.pose.pose.pose.orientation.x
        orientation_y = self.pose.pose.pose.orientation.y
        orientation_z = self.pose.pose.pose.orientation.z
        orientation_w = self.pose.pose.pose.orientation.w
        
        position = np.array([position_x, position_y])
        quaternion = [orientation_x, orientation_y, orientation_z, orientation_w]
        lookahead = lane.lookahead_point(position, 0.9)
        yaw = tf.transformations.euler_from_quaternion(quaternion)

        rotation = np.array([[np.cos(-yaw), -np.sin(-yaw)], [np.sin(-yaw), np.cos(-yaw)]])
        vector = lookahead - position
        vector = np.matmul(rotation, vector)

        steering_angle = math.atan2(vector[1], vector[0]) + yaw
        steering_msg = SteeringCommand()
        steering_msg.value = steering_angle
        self.pub_steering.publish(steering_msg)

        speed_msg = SpeedCommand()
        speed_msg.value = 0.2
        self.pub_speed.publish(speed_msg)


        visualize_msg = Marker(type=Marker.SPHERE, action=Marker.ADD)
        visualize_msg.header.frame_id = "map"
        visualize_msg.scale.x = 0.1
        visualize_msg.scale.y = 0.1
        visualize_msg.scale.z = 0.1
        visualize_msg.color.b = 1.0
        visualize_msg.color.a = 1.0
        visualize_msg.id = 1
        visualize_msg.color.b = 0.0
        visualize_msg.color.g = 1.0
        visualize_msg.pose.position.x = lookahead[0]
        visualize_msg.pose.position.y = lookahead[1]
        self.pub_lookahead.publish(visualize_msg)

    def callback_shutdown(self):
        speed_msg = SpeedCommand()
        speed_msg.value = 0.0
        self.pub_speed.publish(speed_msg)


if __name__ == "__main__":
    ObstacleAvoidance()
