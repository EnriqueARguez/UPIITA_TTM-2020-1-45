#!/usr/bin/env python

from tf.transformations import euler_from_quaternion, quaternion_from_euler
import move_p2p
import lineal_transformations
import VHF
import numpy as np
from math import pi, asin
import time
import mapping as mp
import os
import sys
import matplotlib.pyplot as plt

#Importamos rospy
import rospy

#Llamamos los nodos que vamos a usar
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

#Declaramos nuestros nodos
from std_msgs.msg import Float64MultiArray, Bool, String, Float32, Int8
from geometry_msgs.msg import PoseWithCovariance

class Node:
	def __init__(self):
		self.nav = Bool()
		self.finish_flag = Bool()
		self.nav_opt = 'p2p'
		self.tx = []
		self.ty = []
		self.init_sys = None

		self.nav_node = rospy.Publisher('/sys_nav',Bool,queue_size=1)
		self.sys_loc_node = rospy.Subscriber('/sys_loc',Bool,self.loc_callback)
		self.sys_form_node = rospy.Subscriber('/sys_form',Bool,self.form_callback)
		self.init_sys_node = rospy.Subscriber('/IniciarSistema',Bool,self.init_sys_callback)
		self.nav_opt_node = rospy.Subscriber('/OpcionNavegacion',Bool,self.nav_opt_callback)
		self.px_node = rospy.Subscriber('/PosicionX',Float32,self.px_callback)
		self.py_node = rospy.Subscriber('/PosicionY',Float32,self.py_callback)
		self.num_tray_node = rospy.Subscriber('/NumeroTrayectoria',Int8,self.num_tray_callback)
		self.finish_flag_node = rospy.Publisher('/finish_flag',Bool,queue_size = 1)

	def px_callback(self,msg):
		self.px = msg.data

	def py_callback(self,msg):
		self.py = msg.data

	def num_tray_callback(self,msg):
		self.num = msg.data

	def loc_callback(self,msg):
		self.loc = msg.data

	def form_callback(self,msg):
		self.form = msg.data

	def init_sys_callback(self,msg):
		self.init_sys = msg.data

	def nav_opt_callback(self,msg):
		self.nav_opt_flag = msg.data
		
		if self.nav_opt_flag == True:
			self.nav_opt = 'p2p'
		else:
			self.nav_opt = 'tray'

	def shutdown(self):
		self.nav.data = False
		self.nav_node.publish(self.nav)
		self.finish_flag.data = False
		self.finish_flag_node.publish(n1.finish_flag)
		rospy.sleep(1)

class TurtleBot:
	def __init__(self,name = None):
		self.ang=None
		self.map_flag=False
		rob_r=0.105
		self.obs_dist_min=0.13+2*rob_r
		self.error_d=np.zeros(100,dtype='f')
		self.error_a=np.zeros(100,dtype='f')
		self.ed = Float64MultiArray()
		self.ea = Float64MultiArray()
		self.obtain_dir = False
		self.go2dir = None

		try:
			self.name=name
			self.role='Seguidor'
			odometria='/'+name+'/odom'
			self.pose_node=rospy.Subscriber(odometria,Odometry,callback=self.pose_callback)
			laser='/'+name+'/scan'
			self.scan_node=rospy.Subscriber(laser,LaserScan,self.scan_callback)
			vel='/'+name+'/cmd_vel'
			self.vel_node=rospy.Publisher(vel,Twist,queue_size=10)

			self.histogram_node = rospy.Subscriber('/'+name+'/hist',Float64MultiArray,self.hist_callback)
			self.map_flag_node = rospy.Subscriber('/'+name+'/map_flag',Bool,self.map_flag_callback)
			self.refpose_node = rospy.Subscriber('/'+name+'/refpose',PoseWithCovariance,self.refpose_callback)
			self.error_dist_node = rospy.Publisher('/'+name+'/error_d',Float64MultiArray,queue_size=1)
			self.error_ang_node = rospy.Publisher('/'+name+'/error_a',Float64MultiArray,queue_size=1)

			#Nodo de rol del robot
			self.role_node = rospy.Subscriber('/' + name + '/role',String,self.role_callback)
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
		self.refx = round(msg.pose.position.x,2)
		self.refy = round(msg.pose.position.y,2)

		self.refyaw = round(msg.pose.orientation.z,2)

	def role_callback(self,msg):
		self.role = msg.data

class tb3system(TurtleBot):
	def obtain_dist_min(self):
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
		self.cur_pose()

		self.gdist = move_p2p.euclidean_distance(gx,gy,self.refx,self.refy)
		gyaw  = move_p2p.steering_angle(gx,gy,self.refx,self.refy)

		if gyaw < 0:
			gyaw += 2*pi
		gtheta=int(np.rad2deg(gyaw)) #Valores de 0 a 360 grados

		self.dist_min = self.obtain_dist_min()

		if self.histogram[gtheta] == 0:
			self.obtain_dir = True
			gyaw = move_p2p.steering_angle(gx,gy,self.refx,self.refy) - self.refyaw
		else:
			gyaw = self.evaluate_hist(gtheta) - self.refyaw

		if gyaw < 0:
			gyaw += 2*pi

		self.gyaw = (gyaw - pi) % (2 * pi) - pi

		if self.gdist < 0.08:
			self.gyaw = 0.0
		
		vlineal=move_p2p.linear_vel(self.gdist,5)
		vangular=move_p2p.angular_vel(self.gyaw,2)

		vel_msg=Twist()
		if abs(self.gyaw) > pi/4:
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

def read_trayectory(name):
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

if __name__=='__main__':
	dir=os.getcwd()
	rospy.init_node('system',anonymous=False)
	c = None
	new_trayectory = True

	t1=tb3system(name='Robot1')
	t2=tb3system(name='Robot2')
	t3=tb3system(name='Robot3')

	n1=Node()

	time.sleep(2)

	while (t1.map_flag != True or t2.map_flag != True or t3.map_flag != True) and n1.form != True:
		print('Esperando datos...')
		time.sleep(2)

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

	rate=rospy.Rate(10)

	if t1.role == 'Lider':
		rospy.on_shutdown(t1.shutdown)
	if t2.role == 'Lider':
		rospy.on_shutdown(t2.shutdown)
	if t3.role == 'Lider':
		rospy.on_shutdown(t3.shutdown)

	#Si se trabaja sin interfaz, asignar a True la siguiente variable
	nav_noGUI = True
	nav_tray = True
	tray_def = 3 #Numero de trayectorias definidas
	tx=[]
	ty=[]
	
	#Bandera que indica si se termino la navegacion
	n1.finish_flag.data = False
	n1.finish_flag_node.publish(n1.finish_flag)

	rospy.on_shutdown(n1.shutdown)

	while not rospy.is_shutdown():

		n1.nav.data = True
		n1.nav_node.publish(n1.nav)

		n1.finish_flag.data = False
		n1.finish_flag_node.publish(n1.finish_flag)

		#Inicio o finalizacion del sistema
		if nav_noGUI == True and nav_tray == False:
			n1.nav_node.publish(n1.nav)
			n1.finish_flag_node.publish(n1.finish_flag)
			n1.nav_opt = 'p2p'
			c = input('Desea ingresar un punto? [1 -> Si][0 -> No] : ')

			if c==1:
				px=input('Ingrese el punto destino en x: ')
				py=input('Ingrese el punto destino en y: ')

				print('===== Iniciando Navegacion =====')

				#Si el sistema de localizacion o el de formacion se detiene, este codigo tambien se detiene
				if n1.loc == False or n1.form == False:
					sys.exit()
			else:
				sys.exit()

			n1.finish_flag.data = False
			n1.finish_flag_node.publish(n1.finish_flag)

		elif nav_noGUI == True and nav_tray == True:
			n1.nav_node.publish(n1.nav)
			n1.finish_flag_node.publish(n1.finish_flag)
			n1.nav_opt = 'tray'
			c = input('Desea ingresar una trayectoria? [1 -> Si][0 -> No] : ')

			if c==1:
				num = input('Ingrese el numero de trayectoria: ')

				print('===== Iniciando Navegacion =====')

				if num > 0 and num <= tray_def:
					name = 'Trayectoria' + str(num)
					tx,ty = read_trayectory(name)
				else:
					print('La trayectoria ingresada no existe o no esta definida para los limites de trayectoria definidas')
					continue

				#Si el sistema de localizacion o el de formacion se detiene, este codigo tambien se detiene
				if n1.loc == False or n1.form == False:
					sys.exit()
			else:
				sys.exit()

			n1.finish_flag.data = False
			n1.finish_flag_node.publish(n1.finish_flag)

		else:
			while n1.init_sys != True:
				n1.nav_node.publish(n1.nav)
				n1.finish_flag_node.publish(n1.finish_flag)
				print('Esperando inicio de la navegacion...')

				#Si el sistema de localizacion o el de formacion se detiene, este codigo tambien se detiene
				if n1.loc == False or n1.form == False:
					sys.exit()

				rate.sleep()

			if n1.nav_opt == 'tray':

				if n1.num > 0 and n1.num <= tray_def:
					name = 'Trayectoria' + str(n1.num)
					n1.tx,n1.ty = read_trayectory(name)
				else:
					print('La trayectoria ingresada no existe o no esta definida para los limites de trayectoria definidas')
					continue

			n1.finish_flag.data = False
			n1.finish_flag_node.publish(n1.finish_flag)

			print('Iniciando navegacion')
		
		#============================================================================================
		#Inicio de la navegacion
		if n1.nav_opt == 'tray' and nav_noGUI == False:
			n1.nav_node.publish(n1.nav)
			for i in range(len(n1.tx)):
				if t1.role == 'Lider':
					while move_p2p.euclidean_distance(n1.tx[i],n1.ty[i],t1.refx,t1.refy) >= 0.05:
						if n1.init_sys == False:
							break
						#Si el sistema de localizacion o el de formacion se detiene, este codigo tambien se detiene
						if n1.loc == False or n1.form == False:
							sys.exit()
						t1.go2goal(n1.tx[i],n1.ty[i])

						rate.sleep()
					t1.vel_node.publish(Twist())
					#Reiniciamos los errores a 0
					t1.error_d = np.zeros(100,dtype='f')
					t1.error_a = np.zeros(100,dtype='f')

					t1.H_matrix(True)

				elif t2.role == 'Lider':
					while move_p2p.euclidean_distance(n1.tx[i],n1.ty[i],t2.refx,t2.refy) >= 0.05:
						if n1.init_sys == False:
							break
						#Si el sistema de localizacion o el de formacion se detiene, este codigo tambien se detiene
						if n1.loc == False or n1.form == False:
							sys.exit()
						t2.go2goal(n1.tx[i],n1.ty[i])

						rate.sleep()
					t2.vel_node.publish(Twist())
					#Reiniciamos los errores a 0
					t2.error_d = np.zeros(100,dtype='f')
					t2.error_a = np.zeros(100,dtype='f')

					t2.H_matrix(True)

				elif t3.role == 'Lider':
					while move_p2p.euclidean_distance(n1.tx[i],n1.ty[i],t3.refx,t3.refy) >= 0.05:
						if n1.init_sys == False:
							break
						#Si el sistema de localizacion o el de formacion se detiene, este codigo tambien se detiene
						if n1.loc == False or n1.form == False:
							sys.exit()
						t3.go2goal(n1.tx[i],n1.ty[i])

						rate.sleep()
					t3.vel_node.publish(Twist())
					#Reiniciamos los errores a 0
					t3.error_d = np.zeros(100,dtype='f')
					t3.error_a = np.zeros(100,dtype='f')

					t3.H_matrix(True)
		
		#============================================================================================
		elif n1.nav_opt == 'p2p' and nav_noGUI == False:
			n1.nav_node.publish(n1.nav)
			if t1.role == 'Lider':
				while move_p2p.euclidean_distance(n1.px,n1.py,t1.refx,t1.refy) >= 0.05:
					if n1.init_sys == False:
						break
					#Si el sistema de localizacion o el de formacion se detiene, este codigo tambien se detiene
					if n1.loc == False or n1.form == False:
						sys.exit()
					t1.go2goal(n1.px,n1.py)

					rate.sleep()
				t1.vel_node.publish(Twist())
				#Reiniciamos los errores a 0
				t1.error_d = np.zeros(100,dtype='f')
				t1.error_a = np.zeros(100,dtype='f')

				t1.H_matrix(True)

			elif t2.role == 'Lider':
				while move_p2p.euclidean_distance(n1.px,n1.py,t2.refx,t2.refy) >= 0.05:
					if n1.init_sys == False:
						break
					#Si el sistema de localizacion o el de formacion se detiene, este codigo tambien se detiene
					if n1.loc == False or n1.form == False:
						sys.exit()
					t2.go2goal(n1.px,n1.py)

					rate.sleep()
				t2.vel_node.publish(Twist())
				#Reiniciamos los errores a 0
				t2.error_d = np.zeros(100,dtype='f')
				t2.error_a = np.zeros(100,dtype='f')

				t2.H_matrix(True)

			elif t3.role == 'Lider':
				while move_p2p.euclidean_distance(n1.px,n1.py,t3.refx,t3.refy) >= 0.05:
					if n1.init_sys == False:
						break
					#Si el sistema de localizacion o el de formacion se detiene, este codigo tambien se detiene
					if n1.loc == False or n1.form == False:
						sys.exit()
					t3.go2goal(n1.px,n1.py)

					rate.sleep()
				t3.vel_node.publish(Twist())
				#Reiniciamos los errores a 0
				t3.error_d = np.zeros(100,dtype='f')
				t3.error_a = np.zeros(100,dtype='f')

				t3.H_matrix(True)

		#============================================================================================
		elif n1.nav_opt == 'p2p' and nav_noGUI == True:
			n1.nav_node.publish(n1.nav)
			if t1.role == 'Lider':
				while move_p2p.euclidean_distance(px,py,t1.refx,t1.refy) >= 0.05:
					if n1.init_sys == False:
						break
					#Si el sistema de localizacion o el de formacion se detiene, este codigo tambien se detiene
					if n1.loc == False or n1.form == False:
						sys.exit()
					t1.go2goal(px,py)

					print('===== Valores =====')
					print('Angulo destino:    {}'.format(t1.gyaw))
					print('Distancia destino: {}'.format(t1.gdist))
					rate.sleep()
				t1.vel_node.publish(Twist())
				#Reiniciamos los errores a 0
				t1.error_d = np.zeros(100,dtype='f')
				t1.error_a = np.zeros(100,dtype='f')

				t1.H_matrix(True)

			elif t2.role == 'Lider':
				while move_p2p.euclidean_distance(px,py,t2.refx,t2.refy) >= 0.05:
					if n1.init_sys == False:
						break
					#Si el sistema de localizacion o el de formacion se detiene, este codigo tambien se detiene
					if n1.loc == False or n1.form == False:
						sys.exit()
					t2.go2goal(px,py)

					print('===== Valores =====')
					print('Angulo destino:    {}'.format(t2.gyaw))
					print('Distancia destino: {}'.format(t2.gdist))
					rate.sleep()
				t2.vel_node.publish(Twist())
				#Reiniciamos los errores a 0
				t2.error_d = np.zeros(100,dtype='f')
				t2.error_a = np.zeros(100,dtype='f')

				t2.H_matrix(True)

			elif t3.role == 'Lider':
				while move_p2p.euclidean_distance(px,py,t3.refx,t3.refy) >= 0.05:
					if n1.init_sys == False:
						break
					#Si el sistema de localizacion o el de formacion se detiene, este codigo tambien se detiene
					if n1.loc == False or n1.form == False:
						sys.exit()
					t3.go2goal(px,py)

					print('===== Valores =====')
					print('Angulo destino:    {}'.format(t3.gyaw))
					print('Distancia destino: {}'.format(t3.gdist))
					rate.sleep()
				t3.vel_node.publish(Twist())
				#Reiniciamos los errores a 0
				t3.error_d = np.zeros(100,dtype='f')
				t3.error_a = np.zeros(100,dtype='f')

				t3.H_matrix(True)

		#============================================================================================
		elif n1.nav_opt == 'tray' and nav_noGUI == True:
			n1.nav_node.publish(n1.nav)
			for i in range(len(tx)):
				if t1.role == 'Lider':
					while move_p2p.euclidean_distance(tx[i],ty[i],t1.refx,t1.refy) >= 0.05:
						if n1.init_sys == False:
							break
						#Si el sistema de localizacion o el de formacion se detiene, este codigo tambien se detiene
						if n1.loc == False or n1.form == False:
							sys.exit()
						t1.go2goal(tx[i],ty[i])

						rate.sleep()
					t1.vel_node.publish(Twist())
					#Reiniciamos los errores a 0
					t1.error_d = np.zeros(100,dtype='f')
					t1.error_a = np.zeros(100,dtype='f')

					t1.H_matrix(True)

				elif t2.role == 'Lider':
					while move_p2p.euclidean_distance(tx[i],ty[i],t2.refx,t2.refy) >= 0.05:
						if n1.init_sys == False:
							break
						#Si el sistema de localizacion o el de formacion se detiene, este codigo tambien se detiene
						if n1.loc == False or n1.form == False:
							sys.exit()
						t2.go2goal(tx[i],ty[i])

						rate.sleep()
					t2.vel_node.publish(Twist())
					#Reiniciamos los errores a 0
					t2.error_d = np.zeros(100,dtype='f')
					t2.error_a = np.zeros(100,dtype='f')

					t2.H_matrix(True)

				elif t3.role == 'Lider':
					while move_p2p.euclidean_distance(tx[i],ty[i],t3.refx,t3.refy) >= 0.05:
						if n1.init_sys == False:
							break
						#Si el sistema de localizacion o el de formacion se detiene, este codigo tambien se detiene
						if n1.loc == False or n1.form == False:
							sys.exit()
						t3.go2goal(tx[i],ty[i])

						rate.sleep()
					t3.vel_node.publish(Twist())
					#Reiniciamos los errores a 0
					t3.error_d = np.zeros(100,dtype='f')
					t3.error_a = np.zeros(100,dtype='f')

					t3.H_matrix(True)
		
		n1.finish_flag.data = True
		n1.finish_flag_node.publish(n1.finish_flag)

		rate.sleep()
	