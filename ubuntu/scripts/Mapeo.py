#!/usr/bin/env python

from tf.transformations import euler_from_quaternion, quaternion_from_euler
import move_p2p
import lineal_transformations
import VHF
import cv2
import numpy as np
from math import pi, asin
import time
import mapping as mp
import os
import matplotlib.pyplot as plt

#Importamos rospy
import rospy

#Llamamos los nodos que vamos a usar
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

#Declaramos nuestros nodos
from std_msgs.msg import Float64MultiArray, Bool
from geometry_msgs.msg import PoseWithCovariance

class Node:
	def __init__(self):
		self.loc = Bool()
		self.loc_node = rospy.Publisher('/sys_loc',Bool,queue_size=1)
		self.ui_node = rospy.Subscriber('/IniciarSistema',Bool,self.ui_callback)

	def ui_callback(self,msg):
		self.ui = msg.data

	def shutdown(self):
		self.loc.data = False
		self.loc_node.publish(self.loc)
		rospy.sleep(1)

class TurtleBot:
	def __init__(self,name = None):
		self.T=np.identity(4)
		self.ang=None
		rob_r=0.105
		self.obs_dist_min=0.13+2*rob_r
		self.histogram = Float64MultiArray()
		self.refpose = PoseWithCovariance()
		self.flag = Bool()
		self.last_pos_x = Float64MultiArray()
		self.last_pos_y = Float64MultiArray()
		self.lpx = []
		self.lpy = []

		self.map_icpx = Float64MultiArray()
		self.map_icpy = Float64MultiArray()
		
		try:
			self.name=name
			self.role='Seguidor'
			odometria='/'+name+'/odom'
			self.pose_node=rospy.Subscriber(odometria,Odometry,callback=self.pose_callback)
			laser='/'+name+'/scan'
			self.scan_node=rospy.Subscriber(laser,LaserScan,self.scan_callback)

			self.histogram_node = rospy.Publisher('/'+name+'/hist',Float64MultiArray,queue_size = 1)
			self.map_flag_node = rospy.Publisher('/'+name+'/map_flag',Bool,queue_size = 1)
			self.refpose_node = rospy.Publisher('/'+name+'/refpose',PoseWithCovariance,queue_size = 1)
			self.last_pos_x_node = rospy.Publisher('/'+name+'/last_pos_x',Float64MultiArray,queue_size = 1)
			self.last_pos_y_node = rospy.Publisher('/'+name+'/last_pos_y',Float64MultiArray,queue_size = 1)
			self.map_icpx_node = rospy.Publisher('/'+name+'/map_icpx',Float64MultiArray,queue_size = 1)
			self.map_icpy_node = rospy.Publisher('/'+name+'/map_icpy',Float64MultiArray,queue_size = 1)
		except:
			print('No se ha podido realizar leer correctamente los datos del robot ', name , ', intente nuevamente...')

	def show_info(self):
		print('===============================================')
		print('Robot:        [{}]').format(self.name)
		print('Posicion:     [x   = {}   | y = {}]'.format(self.x,self.y))
		print('Orientacion:  [yaw = {}]\n'.format(self.yaw))


	def pose_callback(self,msg):
		self.qx=round(msg.pose.pose.orientation.x,4)
		self.qy=round(msg.pose.pose.orientation.y,4)
		self.qz=round(msg.pose.pose.orientation.z,4)
		self.qw=round(msg.pose.pose.orientation.w,4)

		(self.roll,self.pitch,self.yaw)=euler_from_quaternion([self.qx,self.qy,self.qz,self.qw])

		self.x=round(msg.pose.pose.position.x,4)
		self.y=round(msg.pose.pose.position.y,4)

	def scan_callback(self,msg):
		self.ranges=msg.ranges

class tb3system(TurtleBot):
	def ref_pose(self):
		self.refx,self.refy=lineal_transformations.ref_point(self.T,self.x,self.y)
		#Para el sistema en fisico se maneja asi, comentar las siguientes lineas en caso de simulacion
		self.refx = self.refx
		self.refy = self.refy

		ang=self.yaw + asin(self.T[1,0]) - self.ang

		if ang > pi:
			self.refyaw = ang - 2*pi
		elif ang < -pi:
			self.refyaw = ang + 2*pi
		else:
			self.refyaw = ang

	def mapping_env(self,save_map = False):
		xr,yr,name = mp.map(self.ranges,save_file = save_map)
		if name != None:
			mp.plot_map(name)
		self.data_pc = np.array([xr,yr])

	def icp(self,template,do_plot = False):
		print('====== {} ======'.format(self.name))
		T = mp.test_icp(self.data_pc,template)
		self.T[:2,:2] = T[:2,:2]

		#Verificar si asi seria para el fisico
		# self.T[0,3] = - (T[0,2] - self.x)
		# self.T[1,3] = - (T[1,2] - self.y)
		#==========
		self.T[0,3] = (T[0,2] - self.x)
		self.T[1,3] = (T[1,2] - self.y)

		result = cv2.transform(np.array([self.data_pc.T], copy=True).astype(np.float32), T).T
		self.map_icpx.data = result[0]
		self.map_icpy.data = result[1]

		if do_plot == True:
			mp.plotresult(self.data_pc,template,T)

def plot_hist(H,Ho):
	m,n=[2,1] # Divide la grafica en 2.
	plt.subplot(m,n,1) # Elige la primera grafica.
	plt.hold(False) # No guarda la grafica pasada.
	plt.plot(range(360),Ho) # Grafica el histograma original.
	plt.axis([0,360,-2,2]) # Establece los limites de la grafica.
	plt.title("Histograma Polar") # Le coloca un titulo a la grafica.
	plt.draw() # Hace visible la grafica. 
	plt.pause(0.00000000001) # Le da un tiempo para graficar.
	plt.subplot(m,n,2) # Elige la primera grafica.
	plt.hold(False) # No guarda la grafica pasada.
	plt.plot(range(360),H) # Grafica el histograma modificado.
	plt.axis([0,360,-1.5,1.5]) # Establece los limites de la grafica.
	plt.title("Histograma Polar Modificado") # Le coloca un titulo a la grafica.
	plt.draw() # Hace visible la grafica. 
	plt.pause(0.00000000001) # Le da un tiempo para graficar.

if __name__=='__main__':
	dir=os.getcwd()
	rospy.init_node('map_system',anonymous=True)

	t1=tb3system(name='Robot1')
	t2=tb3system(name='Robot2')
	t3=tb3system(name='Robot3')

	n1=Node()

	time.sleep(2)

	# while n1.ui != True:
	# 	print('Esperando inicio del sistema...')
	# Estas lineas son para la implementacion de otra logica para la interfaz

	#===== Obtenemos la rotacion inicial leida por el robot =====
	t1.ang=t1.yaw
	t2.ang=t2.yaw
	t3.ang=t3.yaw
	#============================================================

	#===== Abrimos el txt con la nube de puntos base =====
	File=open(dir+'/Mapas/Mapa_Base.txt','r')
	data=File.readlines()
	File.close()

	aux=[c.strip('\n') for c in data]
	auxx=aux[0].split(' ')
	auxy=aux[1].split(' ')
	x=[float(value) for value in auxx]
	y=[float(value) for value in auxy]
	template=np.array([x,y])
	#======================================================

	t1.mapping_env()
	t1.icp(template,do_plot = False)
	t1.ref_pose()
	
	t2.mapping_env()
	t2.icp(template,do_plot = False)
	t2.ref_pose()

	t3.mapping_env()
	t3.icp(template,do_plot = False)
	t3.ref_pose()

	rate=rospy.Rate(30)

	rospy.on_shutdown(n1.shutdown)

	while not rospy.is_shutdown():
		t1.ref_pose()
		t2.ref_pose()
		t3.ref_pose()

		H , Ho = VHF.generate_histogram(t1.ranges,t1.refyaw)
		t1.histogram.data = H
		t1.histogram_node.publish(t1.histogram)

		t1.refpose.pose.position.x = t1.refx
		t1.refpose.pose.position.y = t1.refy
		t1.refpose.pose.orientation.z = t1.refyaw

		if len(t1.lpx) < 5:
			t1.lpx.append(t1.refx)
			t1.lpy.append(t1.refy)
		elif move_p2p.euclidean_distance(t1.refx,t1.refy,t1.lpx[-1],t1.lpy[-1]) > 0.05:
			t1.lpx[:-1]=t1.lpx[1:]
			t1.lpx[-1]=t1.refx
			t1.lpy[:-1]=t1.lpy[1:]
			t1.lpy[-1]=t1.refy

		t1.last_pos_x.data = t1.lpx
		t1.last_pos_y.data = t1.lpy

		t1.last_pos_x_node.publish(t1.last_pos_x)
		t1.last_pos_y_node.publish(t1.last_pos_y)

		t1.map_icpx_node.publish(t1.map_icpx)
		t1.map_icpy_node.publish(t1.map_icpy)

		t1.refpose_node.publish(t1.refpose)

		if len(H) != 0:
			t1.flag.data = True

		t1.map_flag_node.publish(t1.flag)

		H , Ho = VHF.generate_histogram(t2.ranges,t2.refyaw)
		t2.histogram.data = H
		t2.histogram_node.publish(t2.histogram)

		t2.refpose.pose.position.x = t2.refx
		t2.refpose.pose.position.y = t2.refy
		t2.refpose.pose.orientation.z = t2.refyaw

		if len(t2.lpx) < 5:
			t2.lpx.append(t2.refx)
			t2.lpy.append(t2.refy)
		elif move_p2p.euclidean_distance(t2.refx,t2.refy,t2.lpx[-1],t2.lpy[-1]) > 0.05:
			t2.lpx[:-1]=t2.lpx[1:]
			t2.lpx[-1]=t2.refx
			t2.lpy[:-1]=t2.lpy[1:]
			t2.lpy[-1]=t2.refy
		
		t2.last_pos_x.data = t2.lpx
		t2.last_pos_y.data = t2.lpy

		t2.last_pos_x_node.publish(t2.last_pos_x)
		t2.last_pos_y_node.publish(t2.last_pos_y)

		t2.map_icpx_node.publish(t2.map_icpx)
		t2.map_icpy_node.publish(t2.map_icpy)

		t2.refpose_node.publish(t2.refpose)

		if len(H) != 0:
			t2.flag.data = True

		t2.map_flag_node.publish(t2.flag)

		H , Ho = VHF.generate_histogram(t3.ranges,t3.refyaw)
		t3.histogram.data = H
		t3.histogram_node.publish(t3.histogram)

		t3.refpose.pose.position.x = t3.refx
		t3.refpose.pose.position.y = t3.refy
		t3.refpose.pose.orientation.z = t3.refyaw

		if len(t3.lpx) < 5:
			t3.lpx.append(t3.refx)
			t3.lpy.append(t3.refy)
		elif move_p2p.euclidean_distance(t3.refx,t3.refy,t3.lpx[-1],t3.lpy[-1]) > 0.05:
			t3.lpx[:-1]=t3.lpx[1:]
			t3.lpx[-1]=t3.refx
			t3.lpy[:-1]=t3.lpy[1:]
			t3.lpy[-1]=t3.refy

		t3.last_pos_x.data = t3.lpx
		t3.last_pos_y.data = t3.lpy

		t3.last_pos_x_node.publish(t3.last_pos_x)
		t3.last_pos_y_node.publish(t3.last_pos_y)

		t3.map_icpx_node.publish(t3.map_icpx)
		t3.map_icpy_node.publish(t3.map_icpy)

		t3.refpose_node.publish(t3.refpose)

		if len(H) != 0:
			t3.flag.data = True

		t3.map_flag_node.publish(t3.flag)

		n1.loc.data = True
		n1.loc_node.publish(n1.loc)

		rate.sleep()