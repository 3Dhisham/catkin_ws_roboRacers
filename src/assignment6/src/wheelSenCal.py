# !/usr/bin/env python2
import sys

import rospy
from autominy_msgs.msg import SpeedCommand, NormalizedSteeringCommand, SteeringAngle, Tick, NormalizedSpeedCommand
from nav_msgs.msg import Odometry
import math
from std_msgs.msg import Float32
from std_msgs.msg import String


class WheelSensorCalibration:

    def __init__(self):
        self.sub_localize = rospy.Subscriber("/communication/gps/7", Odometry, self.localize, queue_size=10)
        self.sub_ticks = rospy.Subscriber("/sensors/arduino/ticks", Tick, self.tick_counter, queue_size=10)
        self.pub_speed = rospy.Publisher("/actuators/speed", SpeedCommand, queue_size=10)
        self.pub_steer = rospy.Publisher("/actuators/steering_normalized", NormalizedSteeringCommand, queue_size=10)

        self.rate = rospy.Rate(10)  # 10hz

        # Variables
        self.tick = 0
        self.points = []
        self.x = 0.0
        self.y = 0.0

    def tick_counter(self, msg):
        self.tick += msg.value

    def localize(self, raw_msgs):
        self.x = round(raw_msgs.pose.pose.position.x, 3)
        self.y = round(raw_msgs.pose.pose.position.y, 3)
        self.points.append((self.x, self.y))


    def execute(self, steering, speed, duration):
    	# Sleep a few seconds to prevent Devision By Zero error
        rospy.sleep(2.0)

        self.tick = 0
        self.points = []

        self.pub_steer.publish(NormalizedSpeedCommand(value=steering))
        self.pub_speed.publish(SpeedCommand(value=speed))
        rospy.sleep(duration)
        self.pub_speed.publish(SpeedCommand(value=0.0))
        
        points = self.points[:]
        tick = self.tick
        dist = distance(points)
        
        print(30*"-")
        print(30*"-")
        print("Distance: ", dist)
        print("Ratio is: ", dist / tick, "Distance per Tick")
        print(30*"-")
        print(30*"-")



def distance(points):
    dist = 0
    points = [point for x, point in enumerate(points) if x % 2 == 0]
    # use zip to iterate
    for ((x1, y1), (x2, y2)) in zip(points, points[1:]):
        dist += math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)
    return dist


def main(steering, speed, duration):
    try:
        rospy.init_node("WheelSenCal")
        calibration = WheelSensorCalibration()
        calibration.execute(steering, speed, duration)
        print(30*"#")

    except rospy.ROSInterruptException:
        print("=== Failed ===")


if __name__ == '__main__':
    main(0.0, 0.2, 4.0)
