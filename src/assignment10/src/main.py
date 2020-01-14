#!/usr/bin/env python

import sys
import rospy
import numpy as np
import math
from timer import Timer
from nav_msgs.msg import Odometry
from trajectory import trj, closest_point, get_distance
import matplotlib.pyplot as plt
from threading import Lock

ax = plt.subplots(1, 1, figsize=(4.3, 6), dpi=80, facecolor='w')[1]
ax.set_xlim([0,430])
ax.set_ylim([0,600])

car_position = [0,0]
mutex = Lock()

total_error = 0
total_sq_error = 0
readings = 0


total_timer = Timer()

def plot_point(canvas, point, color_='r'):
	canvas.scatter(point[0], point[1], color=color_)

def odom_cb(data):
	global car_position, total_error, readings, total_sq_error

	m_to_pix_mul = 100.0
	pos = [data.pose.pose.position.y*m_to_pix_mul, data.pose.pose.position.x*m_to_pix_mul]

	mutex.acquire()
	car_position = pos
	mutex.release()

	s_pos = closest_point(trj, pos)
	error = get_distance(s_pos, pos)
	total_error += error
	total_sq_error += error*error
	readings += 1

	#plot car position


def main(args):
	global car_position, total_error, readings, total_sq_error

	rospy.init_node('odom_follow')
	print("Node started")
	total_timer.start()

	odom_sub = rospy.Subscriber("//odom/", Odometry, odom_cb, queue_size=None)
	rospy.sleep(1)


	while(total_timer.elapsed_time()<60):
		mutex.acquire()
		tmp_cp = car_position
		mutex.release()

		plot_point(ax, tmp_cp, 'g')
		plot_point(ax, closest_point(trj, tmp_cp), 'r')

		plt.pause(0.01)
		plt.show(block=False)

	odom_sub.unregister()
	print("Avg. Error: " + str(0.01*total_error/readings) + "m")
	print("Avg. Sq. Error: " + str(0.01*total_sq_error/readings) + "m")
	#rospy.spin()

if __name__ == '__main__':
	main(sys.argv)
