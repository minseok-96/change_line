#!/usr/bin/python
# -*- coding: utf-8 -*-

from math import cos,sin,pi,sqrt,pow,atan2
import numpy as np

vehicle_position_x = 5
vehicle_position_y = 5
heading = 90.0 

vehicle_yaw = heading / 180*pi

translation = [vehicle_position_x , vehicle_position_y]

t = np.array([ 
	[cos(vehicle_yaw), -sin(vehicle_yaw) , translation[0]],
	[sin(vehicle_yaw),cos(vehicle_yaw),translation[1]],
	[0				,0				,1						]])

det_t = np.linalg.inv(t)

# local to global
local_x = 5
local_y = 0
local_point_vector = [local_x,local_y,1]
global_point_vector = t.dot(local_point_vector)
print(global_point_vector)

#global to local
global_x = 10
global_y = 10
global_point_vector = [global_x, global_y,1]
local_point_vector = det_t.dot(global_point_vector)
print(local_point_vector)

def draw_lange_change(self, start_link , end_link , lane_change_distance, step_size):
	output_path = []

	translation = [start_link.points[0][0], start_link.points[0][1]]
	theta = atan2(start_link.points[1][1] - start_link.points[0][1], start_link.points[1][0] - start_link.points[0[0]])

	t = np.array([
		[cos(theta), -sin(theta) , translation[0]],
		[sin(theta), cos(theta), translation[1]],
		[0		   , 0 			,1				]])

	det_t = np.array([
		[t[0][0],t[1][0], -(t[0][0]*translation[0]+t[1][0]*translation[1])],
		[t[0][1],t[1][1],-(t[0][1]*translation[0] + t[1][1]*translation[1])],
		[0		,0		,1					]])

	world_end_link_list=[]
	for point in end_link.points:
		world_end_link_list.append([point[0],point[1],1])

	world_end_link_metrix = np.array(world_end_link_list).t
	# world to local
	local_end_link_metrix = det_t.dot(world_end_link_metrix).t

	min_dis = float('inf')
	local_end_point_list=[]

	for point in local_end_link_metrix:
		if point[0] > 0:
			dis = abs(sqrt(point[0]*point[0] + point[1]*point[1]) - lane_change_distance)
			if dis < min_dis:
				min_dis = dis
				local_end_point_list = [[point[0]],[point[1]],[1]]
	
	local_end_point_matrix = np.array(local_end_point_list)


	x = []
	y = []
	x_interval = step_size
	xs = 0
	xf = local_end_point_matrix[0][0]

	ps = 0.0
	pf = local_end_point_matrix[1][0]

	x_num = xf  / x_interval
	for i in range(xs,int(x_num)):
		x.append(i*x_interval)

	# get coefficents to draw 3demension-graph
	a = [0.0 , 0.0 , 0.0 , 0.0]
	a[0] = ps
	a[1] = 0
	a[2] = 3.0 * (pf-ps) / (xf**2)
	a[3] = -2.0 * (pf-ps) / (xf**3)

	for i in x:
		result = a[3]*i**3 + a[2]*i**2 + a[1]*i +a[0]
		y.append(result)

	for i in range(0,len(y)):
		local_change_path = np.array([[x[i]],[y[i]],[1]])
		global_change_path = t.dot(local_change_path)
		output_path.append([global_change_path[0][0],global_change_path[1][0],0])

	end_point_index = 0
	for (i,end_point) in enumerate(local_end_link_metrix.tolist()):
		if end_point[0] == local_end_point_matrix[0][0] and end_point[1] == local_end_point_matrix[1][0]:
			end_point_index = i
			break

	for end_point in end_link.points[end_point_index]:
		output_path.append([end_point[0],end_point[1],0])

	return output_path

# dl=draw()
# dl.draw_lange_change(start, end , 1 ,1 )