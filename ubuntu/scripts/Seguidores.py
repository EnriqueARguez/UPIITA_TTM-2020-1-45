#!/usr/bin/env python

from tf.transformations import euler_from_quaternion, quaternion_from_euler
import move_p2p
import lineal_transformations
import VHF
import numpy as np
from math import pi, asin, cos
import time
import mapping as mp
import os
import sys
import matplotlib.pyplot as plt
import formation as fm

#Importamos rospy
import rospy

#Llamamos los nodos que vamos a usar
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

#Declaramos nuestros nodos
from std_msgs.msg import Float64MultiArray, Bool, String, Float32, Int8
from geometry_msgs.msg import PoseWithCovariance

def read_trayectory(name = None):
	dir = os.getcwd()
	if not os.path.exists(dir+'/Trayectorias'):
		os.mkdir(dir+'/Trayectorias')
	File = open(dir + '/Trayectorias/' + name + '.txt', 'r')
	d = File.readlines()
	a = [c.strip('\n') for c in d]
	ax = a[0].split(' ')
	ay = a[1].split(' ')

	px = [float(value) for value in ax]
	py = [float(value) for value in ay]

	return px, py

class Node:
	def __init__(self):
		self.form = Bool()
		self.form_node = rospy.Publisher('/sys_form',Bool,queue_size=1)
		self.sys_loc_node = rospy.Subscriber('/sys_loc',Bool,self.loc_callback)

	def loc_callback(self,msg):
		self.loc = msg.data

	def shutdown(self):
		self.form.data = False
		self.form_node.publish(self.form)
		rospy.sleep(1)

class TurtleBot:
	def __init__(self,name = None,rob_sys = True):
		self.ang=None
		self.hist_flag=False
		self.map_flag=False
		rob_r=0.105
		self.obs_dist_min=0.13+2*rob_r
		self.error_d=np.zeros(100,dtype='f')
		self.error_a=np.zeros(100,dtype='f')
		self.ed = Float64MultiArray()
		self.ea = Float64MultiArray()

		self.obtain_dir = False
		self.go2dir = None

		self.name=name
		self.role=String()
		self.role.data='Sin Asignar'
		self.role_pos = None #Posicion del seguidor en la formacion
		self.form = None

		try:
			self.pose_node=rospy.Subscriber('/'+name+'/odom',Odometry,callback=self.pose_callback)
			self.scan_node=rospy.Subscriber('/'+name+'/scan',LaserScan,self.scan_callback)
			self.vel_node=rospy.Publisher('/'+name+'/cmd_vel',Twist,queue_size=10)

			#Nodos para el histograma
			self.histogram_node = rospy.Subscriber('/'+name+'/hist',Float64MultiArray,self.hist_callback)
			self.map_flag_node = rospy.Subscriber('/'+name+'/map_flag',Bool,self.map_flag_callback)

			#Nodo posicion con respecto al escenario
			self.refpose_node = rospy.Subscriber('/'+name+'/refpose',PoseWithCovariance,self.refpose_callback)

			#Nodos para los errores
			self.error_dist_node = rospy.Publisher('/'+name+'/error_d',Float64MultiArray,queue_size=1)
			self.error_ang_node = rospy.Publisher('/'+name+'/error_a',Float64MultiArray,queue_size=1)

			#Nodo de rol del robot
			self.role_node = rospy.Publisher('/' + name + '/role',String,queue_size=1)
		except:
			print('No se ha podido realizar leer correctamente los datos del robot ', name , ', intente nuevamente...')

	def show_info(self):
		print('===============================================')
		print('Robot:        [{}]').format(self.name)
		print('Posicion:     [x   = {}   | y = {}]'.format(self.x,self.y))
		print('Orientacion:  [yaw = {}]\n'.format(self.yaw))


	def pose_callback(self,msg):
		self.qx=round(msg.pose.pose.orientation.x,2)
		self.qy=round(msg.pose.pose.orientation.y,2)
		self.qz=round(msg.pose.pose.orientation.z,2)
		self.qw=round(msg.pose.pose.orientation.w,2)

		(self.roll,self.pitch,self.yaw)=euler_from_quaternion([self.qx,self.qy,self.qz,self.qw])

		self.x=round(msg.pose.pose.position.x,2)
		self.y=round(msg.pose.pose.position.y,2)

	def scan_callback(self,msg):
		self.ranges=msg.ranges

	def hist_callback(self,msg):
		self.histogram = msg.data

	def map_flag_callback(self,msg):
		self.map_flag = msg.data

	def refpose_callback(self,msg):
		self.refx = round(msg.pose.position.x,4)
		self.refy = round(msg.pose.position.y,4)

		self.refyaw = round(msg.pose.orientation.z,4)

class tb3system(TurtleBot):
	def obtain_dist_min(self):
		scan_filter = []
		lidar_err = 0.004

		for i in range(360):
			if (i <= 105 and i >= 60) or (i > 255 and i < 300):
				if self.ranges[i] >= lidar_err:
					scan_filter.append(self.ranges[i])
		return min(scan_filter)

	def obtain_dist_min_form(self):
		scan_filter = []
		lidar_err = 0.004

		for i in range(360):
			if i <= 120 or i > 240:
				if self.ranges[i] >= lidar_err:
					scan_filter.append(self.ranges[i])
		
		return min(scan_filter)

	def H_matrix(self,get_angle = False):
		if get_angle==True:
			self.angle=self.refyaw
		self.H=lineal_transformations.cur_H(self.angle,self.refx,self.refy)
	
	def cur_pose(self,gx = 0,gy = 0):
		self.curx,self.cury=lineal_transformations.cur_point(self.H,self.refx,self.refy)

		dif=self.refyaw-self.angle
		if dif > pi:
			self.curyaw=dif-2*pi
		elif dif < -pi:
			self.curyaw=dif+2*pi
		else:
			self.curyaw=dif

	def evaluate_hist(self,ind):
		indccw = ind
		hccw = self.histogram[indccw]
		indcw = ind
		hcw = self.histogram[indcw]

		#Evaluamos a que direccion ir
		if self.obtain_dir == True:
			auxr=ind
			auxl=ind
			while self.histogram[auxr] == 1 and self.histogram[auxl] == 1:
				auxr = (auxr-1)%360
				auxl = (auxl+1)%360
				if auxl == ind:
					break
			if self.histogram[auxr] == 0:
				self.go2dir = 'r'
				hcw = self.histogram[auxr]
				self.obtain_dir = False
			elif self.histogram[auxl] == 0:
				self.go2dir = 'l'
				hccw = self.histogram[auxl]
				self.obtain_dir = False

		if self.go2dir == 'l':
			while hccw == 1:
				indccw = (indccw+1)%360
				hccw = self.histogram[indccw]
				if indccw == ind:
					break
			if hccw == 0:
				if self.dist_min < 0.13+0.105*1.25:
					indccw = (indccw+15)%360
				indccw = np.deg2rad(indccw)
				if indccw > pi:
					indccw -= 2*pi
				return indccw
			else:
				ind = np.deg2rad(ind)
				if ind > pi:
					ind -= 2*pi
				print('No se encontro algun camino sin obstaculo...')
				return ind
		elif self.go2dir == 'r':
			while hcw == 1:
				indcw = (indcw-1)%360
				hcw = self.histogram[indcw]
				if indcw == ind:
					break
			if hcw == 0:
				if self.dist_min < 0.13+0.105*1.25:
					indcw = (indcw-15)%360
				indcw = np.deg2rad(indcw)
				if indcw > pi:
					indcw -= 2*pi
				return indcw
			else:
				ind = np.deg2rad(ind)
				if ind > pi:
					ind -= 2*pi
				print('No se encontro algun camino sin obstaculo...')
				return ind

	def go2goal(self,gx,gy,action = None):
		global dist_min
		self.cur_pose()

		self.gdist = move_p2p.euclidean_distance(gx,gy,self.refx,self.refy)
		gyaw  = move_p2p.steering_angle(gx,gy,self.refx,self.refy)
		print(gyaw)

		if gyaw < 0:
			gyaw += 2*pi
		gtheta=int(np.rad2deg(gyaw)) #Valores de 0 a 360 grados

		self.dist_min = self.obtain_dist_min_form()

		print(self.histogram[gtheta])
		
		if self.histogram[gtheta] == 1 and dist_min > 0.29:
			gyaw = self.evaluate_hist(gtheta) - self.refyaw
		else:
			self.obtain_dir = True
			gyaw = move_p2p.steering_angle(gx,gy,self.refx,self.refy) - self.refyaw

		if gyaw < 0:
			gyaw += 2*pi

		self.gyaw = (gyaw - pi) % (2 * pi) - pi

		if self.gdist < 0.05:
			self.gyaw = 0.0
		
		vlineal=move_p2p.linear_vel_lf(self.gdist,5)
		vangular=move_p2p.angular_vel(self.gyaw,2)

		vel_msg=Twist()
		if abs(self.gyaw) > pi/2:
			vel_msg.linear.x=0
		else:
			vel_msg.linear.x=vlineal
		vel_msg.angular.z=vangular
		self.vel_node.publish(vel_msg)

		self.save_error()

	def save_error(self):
		self.error_d[1:] = self.error_d[:-1]
		self.error_a[1:] = self.error_a[:-1]

		self.error_d[0] = self.gdist
		self.error_a[0] = self.gyaw

		self.ed.data = self.error_d
		self.ea.data = self.error_a

		self.error_dist_node.publish(self.ed)
		self.error_ang_node.publish(self.ea)

	def show_refinfo(self):
		print('===============================================')
		print('Datos bajo el marco de referencia del escenario')
		print('Robot:        [{}]').format(self.name)
		print('Posicion:     [x   = {}   | y = {}]'.format(self.refx,self.refy))
		print('Orientacion:  [yaw = {}]\n'.format(self.refyaw))

	def show_newinfo(self):
		print('===============================================')
		print('Datos bajo otro marco de referencia')
		print('Robot:        [{}]').format(self.name)
		print('Posicion:     [x   = {}   | y = {}]'.format(self.curx,self.cury))
		print('Orientacion:  [yaw = {}]\n'.format(self.curyaw))

	def showdistances(self):
		print('===============================================')
		print('NO: {}  N: {}  NE: {}'.format(round(self.ranges[45],2),round(self.ranges[0],2),round(self.ranges[315],2)))
		print('O:  {}  E: {}\n'.format(round(self.ranges[90],2),round(self.ranges[270],2)))

	def shutdown(self):
		print('Deteniendo {}'.format(self.name))
		self.vel_node.publish(Twist())
		rospy.sleep(1)

def angle_lf(x1,y1,x2,y2,a1):
	if a1 < 0:
		a1 += 2*pi

	a = move_p2p.steering_angle(x2,y2,x1,y1) - a1
	a_lf = (a - pi) % (2 * pi) - pi #Angulo formado entre lider y seguidor

	return a_lf

if __name__=='__main__':
	global dist_min

	dir=os.getcwd()
	rospy.init_node('Seguidores',anonymous=True)

	c = None
	new_trayectory = True

	t1=tb3system(name='Robot1',rob_sys = True)
	t2=tb3system(name='Robot2',rob_sys = True)
	t3=tb3system(name='Robot3',rob_sys = True)

	n1 = Node()

	time.sleep(2)

	n1.form.data = False
	n1.form_node.publish(n1.form)

	#Esperamos los datos del mapeo ICP
	while t1.map_flag != True or t2.map_flag != True or t3.map_flag != True:
		n1.form.data = False
		n1.form_node.publish(n1.form)
		print('Esperando datos...')
		time.sleep(2)

	rate=rospy.Rate(10)

	#===== Obtenemos la rotacion inicial leida por el robot =====
	t1.ang=t1.yaw
	t2.ang=t2.yaw
	t3.ang=t3.yaw
	#============================================================

	t1.show_refinfo()
	t1.H_matrix(get_angle = True)
	t2.show_refinfo()
	t2.H_matrix(get_angle = True)
	t3.show_refinfo()
	t3.H_matrix(get_angle = True)

	t1.role_node.publish(t1.role)
	t2.role_node.publish(t2.role)
	t3.role_node.publish(t3.role)

	roles_asignados = False

	#Como las trayectorias definidas inician en una misma coordenada, se utiliza esta misma para la asignacion de roles
	px = 1
	py = 0.8

	# dist_min = None

	x1 = None
	y1 = None
	x2 = None
	y2 = None
	x3 = None
	y3 = None

	d = 0.275
	tol_dist_min = 0.29
	tol_f1 = (d*cos(145*pi/180))
	tol_f2 = (d*cos(pi))
	tol_seguimiento = 0.08

	rospy.on_shutdown(n1.shutdown)
	
	while not rospy.is_shutdown():

		t1.role_node.publish(t1.role)
		t2.role_node.publish(t2.role)
		t3.role_node.publish(t3.role)

		if roles_asignados != True:
			#Asignamos los roles al inicio
			list_tb=[move_p2p.euclidean_distance(px,py,t1.refx,t1.refy),move_p2p.euclidean_distance(px,py,t2.refx,t2.refy),move_p2p.euclidean_distance(px,py,t3.refx,t3.refy)]
			t1.role.data,t2.role.data,t3.role.data=move_p2p.roleselection(list_tb)

			print(list_tb)

			#Publicamos los roles
			t1.role_node.publish(t1.role)
			t2.role_node.publish(t2.role)
			t3.role_node.publish(t3.role)

			if t1.role.data == 'Lider':
				t2.role_pos = 'r'
				t3.role_pos = 'l'
				rospy.on_shutdown(t2.shutdown)
				rospy.on_shutdown(t3.shutdown)
			elif t2.role.data == 'Lider':
				t1.role_pos = 'r'
				t3.role_pos = 'l'
				rospy.on_shutdown(t1.shutdown)
				rospy.on_shutdown(t3.shutdown)
			elif t3.role.data == 'Lider':
				t1.role_pos = 'r'
				t2.role_pos = 'l'
				rospy.on_shutdown(t1.shutdown)
				rospy.on_shutdown(t2.shutdown)

			roles_asignados = True

			rate.sleep()

		#Si no asigno roles, volvemos al inicio del ciclo y lo volvemos a intentar
		if not (t1.role.data == 'Lider' or t2.role.data == 'Lider' or t3.role.data == 'Lider'):
			print('No se logro asignar los roles, volviendo a intentar...')
			roles_asignados = False
			rate.sleep()
			continue

		n1.form.data = True
		n1.form_node.publish(n1.form)

		if t1.role.data == 'Lider':
			dist_min = t1.obtain_dist_min()
		elif t2.role.data == 'Lider':
			dist_min = t2.obtain_dist_min()
		elif t3.role.data == 'Lider':
			dist_min = t3.obtain_dist_min()

		print(t1.role.data)
		print(t2.role.data)
		print(t3.role.data)

		if dist_min > tol_dist_min:
			x1,y1,x2,y2,x3,y3 = fm.first_form(t1.role.data,t2.role.data,t3.role.data,t1.refx,t1.refy,t1.refyaw,t2.refx,t2.refy,t2.refyaw,t3.refx,t3.refy,t3.refyaw)
		else:
			x1,y1,x2,y2,x3,y3 = fm.second_form(t1.role.data,t2.role.data,t3.role.data,t1.refx,t1.refy,t1.refyaw,t2.refx,t2.refy,t2.refyaw,t3.refx,t3.refy,t3.refyaw)

		#Robot 1
		if t1.role.data == 'Lider':

			a1 = angle_lf(t1.refx,t1.refy,t2.refx,t2.refy,t1.refyaw)
			a2 = angle_lf(t1.refx,t1.refy,t3.refx,t3.refy,t1.refyaw)
			a3 = angle_lf(t2.refx,t2.refy,t3.refx,t3.refy,t2.refyaw)

			if move_p2p.euclidean_distance(x1,y1,t2.refx,t2.refy) >= 0.05:
				#Para la formacion delta aseguramos una distancia entre el lider y cada robot, para la otra formacion aseguramos unicamente la distancia entre robots
				if dist_min > tol_dist_min and move_p2p.euclidean_distance(t1.refx,t1.refy,t2.refx,t2.refy)*(cos(a1)) < tol_f1:
					t2.go2goal(x2,y2)
				elif move_p2p.euclidean_distance(t1.refx,t1.refy,t2.refx,t2.refy)*(cos(a1)) < tol_f2:
					t2.go2goal(x2,y2)
				else:
					t2.vel_node.publish(Twist())
			else:
				t2.vel_node.publish(Twist())

			if move_p2p.euclidean_distance(x3,y3,t3.refx,t3.refy) >= 0.05:
				#Para la formacion delta aseguramos una distancia entre el lider y cada robot, para la otra formacion aseguramos unicamente la distancia entre robots
				if dist_min > tol_dist_min and move_p2p.euclidean_distance(t1.refx,t1.refy,t3.refx,t3.refy)*(cos(a2)) < tol_f1 and move_p2p.euclidean_distance(t2.refx,t2.refy,t3.refx,t3.refy) > 0.26:
					t3.go2goal(x3,y3)
				elif move_p2p.euclidean_distance(t2.refx,t2.refy,t3.refx,t3.refy)*(cos(a3)) < tol_f2:
					t3.go2goal(x3,y3)
				else:
					t3.vel_node.publish(Twist())
			else:
				t3.vel_node.publish(Twist())

		#Robot 2
		elif t2.role.data == 'Lider':

			a1 = angle_lf(t2.refx,t2.refy,t1.refx,t1.refy,t2.refyaw)
			a2 = angle_lf(t2.refx,t2.refy,t3.refx,t3.refy,t2.refyaw)
			a3 = angle_lf(t1.refx,t1.refy,t3.refx,t3.refy,t1.refyaw)

			if move_p2p.euclidean_distance(x1,y1,t1.refx,t1.refy) >= 0.05:
				#Para la formacion delta aseguramos una distancia entre el lider y cada robot, para la otra formacion aseguramos unicamente la distancia entre robots
				if dist_min > tol_dist_min and move_p2p.euclidean_distance(t2.refx,t2.refy,t1.refx,t1.refy)*(cos(a1)) < tol_f1:
					t1.go2goal(x1,y1)
				elif move_p2p.euclidean_distance(t2.refx,t2.refy,t1.refx,t1.refy)*(cos(a1)) < tol_f2:
					t1.go2goal(x1,y1)
				else:
					t1.vel_node.publish(Twist())
			else:
				t1.vel_node.publish(Twist())

			if move_p2p.euclidean_distance(x3,y3,t3.refx,t3.refy) >= 0.05:
				#Para la formacion delta aseguramos una distancia entre el lider y cada robot, para la otra formacion aseguramos unicamente la distancia entre robots
				if dist_min > tol_dist_min and move_p2p.euclidean_distance(t2.refx,t2.refy,t3.refx,t3.refy)*(cos(a2)) < tol_f1 and move_p2p.euclidean_distance(t1.refx,t1.refy,t3.refx,t3.refy) > 0.26:
					t3.go2goal(x3,y3)
				elif move_p2p.euclidean_distance(t1.refx,t1.refy,t3.refx,t3.refy)*(cos(a3)) < tol_f2:
					t3.go2goal(x3,y3)
				else:
					t3.vel_node.publish(Twist())
			else:
				t3.vel_node.publish(Twist())

		#Robot 3
		elif t3.role.data == 'Lider':

			a1 = angle_lf(t3.refx,t3.refy,t1.refx,t1.refy,t3.refyaw)
			a2 = angle_lf(t3.refx,t3.refy,t2.refx,t2.refy,t3.refyaw)
			a3 = angle_lf(t1.refx,t1.refy,t2.refx,t2.refy,t1.refyaw)

			if move_p2p.euclidean_distance(x1,y1,t1.refx,t1.refy) >= 0.05:
				#Para la formacion delta aseguramos una distancia entre el lider y cada robot, para la otra formacion aseguramos unicamente la distancia entre robots
				if dist_min > tol_dist_min and move_p2p.euclidean_distance(t3.refx,t3.refy,t1.refx,t1.refy)*(cos(a1)) < tol_f1:
					t1.go2goal(x1,y1)
				elif move_p2p.euclidean_distance(t3.refx,t3.refy,t1.refx,t1.refy)*(cos(a1)) < tol_f2:
					t1.go2goal(x1,y1)
				else:
					t1.vel_node.publish(Twist())
			else:
				t1.vel_node.publish(Twist())

			if move_p2p.euclidean_distance(x3,y3,t2.refx,t2.refy) >= 0.05:
				#Para la formacion delta aseguramos una distancia entre el lider y cada robot, para la otra formacion aseguramos unicamente la distancia entre robots
				if dist_min > tol_dist_min and move_p2p.euclidean_distance(t3.refx,t3.refy,t2.refx,t2.refy)*(cos(a2)) < tol_f1 and move_p2p.euclidean_distance(t1.refx,t1.refy,t2.refx,t2.refy) > 0.26:
					t2.go2goal(x2,y2)
				elif move_p2p.euclidean_distance(t1.refx,t1.refy,t2.refx,t2.refy)*(cos(a3)) < tol_f2:
					t2.go2goal(x2,y2)
				else:
					t2.vel_node.publish(Twist())
			else:
				t2.vel_node.publish(Twist())

		#Si el sistema de localizacion se detiene, este codigo tambien se detiene
		if n1.loc == False:
			sys.exit()

		rate.sleep()
	