#!/usr/bin/env python

from math import pow, atan2, sqrt, pi, acos, cos, sin
from numpy import array

def euclidean_distance(gx,gy,x,y):
	return sqrt(pow((gx-x),2)+pow((gy-y),2))

def roleselection(l):
	role=min(l)

	for count,line in enumerate(l):
		if line == role:
			if count == 0:
				return 'Lider','Seguidor','Seguidor'
			if count == 1:
				return 'Seguidor','Lider','Seguidor'
			if count == 2:
				return 'Seguidor','Seguidor','Lider'

#def linear_vel(gx,gy,x,y,Kv):
#	aux_vl=round(Kv*euclidean_distance(gx,gy,x,y),2)
#	if aux_vl > 0.16:
#		return 0.16
#	elif aux_vl < 0:
#		return 0
#	else:
#		return aux_vl

def linear_vel(dist,Kv,ei = 0.0,Ki = 0.0):
	aux_vl=round(Kv * dist + Ki * ei,2)
	if aux_vl > 0.1:
		return 0.1
	elif aux_vl < 0:
		return 0
	else:
		return aux_vl

def steering_angle(gx,gy,x,y):
	return atan2(gy-y,gx-x)

def angle_error(gx,gy,x,y,angle):
	return abs(steering_angle(gx,gy,x,y)-angle)

#def angular_vel(gx,gy,x,y,Ka,angle):
#	aux_va=round(Ka*(steering_angle(gx,gy,x,y)-angle),2)
#	if aux_va > 2:
#		return 2
#	elif aux_va < -2:
#		return -2
#	else:
#		return aux_va

def angular_vel(angle,Ka,ei = 0.0,Ki = 0.0):
	aux_va=round(Ka * angle + Ki * ei,2)
	if aux_va > 2:
		return 2
	elif aux_va < -2:
		return -2
	else:
		return aux_va

#======================================================================
	
def linear_vel_lf(dist,Kv,ei = 0.0,Ki = 0.0):
	aux_vl=round(Kv * dist + Ki * ei,2)
	if aux_vl > 0.13:
		return 0.13
	else:
		return aux_vl