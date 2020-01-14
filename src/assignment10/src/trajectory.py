#!/usr/bin/env python

import numpy as np
import math
import matplotlib.pyplot as plt

class Line:
	p1 = np.array([0.0, 0.0])
	p2 = np.array([0.0, 0.0])

	def __init__(self, p1_, p2_):
		self.p1 = np.array(p1_)
		self.p2 = np.array(p2_)

	def get_point(self, n):
		assert(n>=0.0 and n<=1.0)
		return (self.p1 + n*(self.p2-self.p1)).tolist()


class Bezier:
	p1 = np.array([0.0, 0.0])
	p2 = np.array([0.0, 0.0])
	p3 = np.array([0.0, 0.0])
	p4 = np.array([0.0, 0.0])

	def __init__(self, p1_, p2_, p3_, p4_):
		self.p1 = np.array(p1_)
		self.p2 = np.array(p2_)	
		self.p3 = np.array(p3_)	
		self.p4 = np.array(p4_)	

	def get_point(self, n):
		assert(n>=0.0 and n<=1.0)
		point = (1.0-n)**3.0*self.p1 + 3.0*(1.0-n)**2.0*n*self.p2 + 3.0*(1.0-n)*n**2.0*self.p3 + n**3.0*self.p4
		return point.tolist()

class Trajectory:
	shapes = []
	
	def __init__(self, shapes_):
		self.shapes = shapes_

	def get_point(self, n):
		n = clamp(0.0, 1.0, n)
		shape_index = clamp(0, len(self.shapes)-1, int(n * len(self.shapes)))
		pn = (n* len(self.shapes)) - shape_index
		return self.shapes[shape_index].get_point(pn)

def clamp(min_val, max_val, val):
	return min(max_val, max(min_val, val))

def plot_point(canvas, point, color_='r'):
	canvas.scatter(point[0], point[1], color=color_)
	plt.show(block=False)

def get_distance(p1, p2):
  return math.sqrt((p2[0]-p1[0])*(p2[0]-p1[0]) + (p2[1]-p1[1])*(p2[1]-p1[1]))

def closest_point(trj, point):
	rul = 1000.0 #resulution until linear interpolation
	it = 1.0 / rul

	#get closest point
	min_dist = 999999999.0
	closest_point = [-1, -1]
	closest_i = 0.0
	i = 0.0
	while(i<=1.0):
		p = trj.get_point(i)
		dist = get_distance(p, point)
		if dist<min_dist:
			min_dist = dist
			closest_point = p
			closest_i = i
		i = i + it

	#get closest neighbouring point
	p_previous = trj.get_point(closest_i - it)
	p_next = trj.get_point(closest_i + it)

	if get_distance(p_previous, point)<get_distance(p_next, point): closest_neigh = p_previous
	else: closest_neigh = p_next

	#get closest point to point on line segment AB
	AB = [closest_neigh[0] - closest_point[0], closest_neigh[1] - closest_point[1]]
	AP = [point[0] - closest_point[0], point[1] - closest_point[1]]
	lengthSqrAB = AB[0]**2.0 + AB[1]**2.0
	if(lengthSqrAB == 0.0): lengthSqrAB = 0.00001
	t = (AP[0]*AB[0] + AP[1]*AB[1]) / lengthSqrAB
	return [closest_point[0]+t*AB[0], closest_point[1]+t*AB[1]]


#singular line in the middle
s_line = Line([64.0, 208.0], [64.0, 372.0])

s_trj = Trajectory([s_line])


#outer loop
o_bez1 = Bezier([31.0, 395.0], [31.0, 653.0], [400.0, 653.0], [400.0, 395.0])
o_line1 = Line([400.0, 395.0], [400.0, 204.0])
o_bez2 = Bezier([400.0, 204.0], [400.0, -52.0], [31.0, -52.0], [31.0, 204.0])
o_line2 = Line([31.0, 204.0], [31.0, 395.0])

o_trj = Trajectory([o_bez1, o_line1, o_bez2, o_line2])


#inner loop
bez1 = Bezier([95.0, 395.0], [95.0, 568.0], [336.0, 568.0], [337.0, 395.0])
line1 = Line([337.0, 395.0], [337.0, 204.0])
bez2 = Bezier([337.0, 204.0], [336.0, 33.0], [95.0, 33.0], [95.0, 204.0])
line2 = Line([95.0, 204.0], [95.0, 395.0])

trj = Trajectory([bez1, line1, bez2, line2])

"""
ax = plt.subplots(1, 1, figsize=(4.3, 6), dpi=80, facecolor='w')[1]
ax.set_xlim([0,430])
ax.set_ylim([0,600])
#ax.cla()
i = 0
it = 0.001
while(i <= 1.0):
	plot_point(ax, trj.get_point(i))
	plot_point(ax, o_trj.get_point(i))
	plot_point(ax, s_trj.get_point(i))
	i = i + it
plt.show()
"""
"""
print(closest_point(trj, [0.0, 0.0]))
print(closest_point(trj, [200.0, 400.0]))
print(closest_point(trj, [100.0, 300.0]))
print(closest_point(trj, [94.0, 12.0]))
"""
