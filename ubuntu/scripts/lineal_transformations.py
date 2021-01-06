#!/usr/bin/env python

from math import cos, sin, pi
from numpy import array, transpose, matmul, linalg

def cur_H(angulo,dx,dy):
	return array([[cos(angulo), -sin(angulo), 0, dx] , [sin(angulo), cos(angulo), 0, dy] , [0, 0, 1, 0] , [0, 0, 0, 1]])

#Del marco de referencia fijo, al actual
def cur_point(H,x0,y0):
	Hinv=linalg.inv(H) #Inversa de la matriz de transformacion
	P0=array([x0,y0,0,1]) #Posicion destino con respecto al marco de referencia
	P1=matmul(Hinv,P0) #Punto destino con respecto al marco actual

	return P1[0], P1[1]

#Del marco de referencia actual, al fijo
def ref_point(H,x,y):
	P1=array([x,y,0,1])
	P0=matmul(H,P1)

	return P0[0], P0[1]
