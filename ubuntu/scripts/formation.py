from math import cos, pi, sin

#Formacion Delta
def first_form(role1,role2,role3,x1,y1,a1,x2,y2,a2,x3,y3,a3):
	roles=[role1,role2,role3]

	d=0.27
	a=145*pi/180

	for i in range(len(roles)):
		if roles[i] == 'Lider':
			mainrole=i
			break

	if mainrole == 0:
		g1x=x1
		g1y=y1
		g2x=x1+d*cos(-a+a1)
		g2y=y1+d*sin(-a+a1)
		g3x=x1+d*cos(a+a1)
		g3y=y1+d*sin(a+a1)
		return g1x,g1y,g2x,g2y,g3x,g3y
	elif mainrole == 1:
		g2x=x2
		g2y=y2
		g1x=x2+d*cos(-a+a2)
		g1y=y2+d*sin(-a+a2)
		g3x=x2+d*cos(a+a2)
		g3y=y2+d*sin(a+a2)
		return g1x,g1y,g2x,g2y,g3x,g3y
	elif mainrole == 2:
		g3x=x3
		g3y=y3
		g1x=x3+d*cos(-a+a3)
		g1y=y3+d*sin(-a+a3)
		g2x=x3+d*cos(a+a3)
		g2y=y3+d*sin(a+a3)
		return g1x,g1y,g2x,g2y,g3x,g3y

#Formacion Serie
def second_form(role1,role2,role3,x1,y1,a1,x2,y2,a2,x3,y3,a3):
	roles=[role1,role2,role3]

	d=0.27
	a=pi

	for i in range(len(roles)):
		if roles[i] == 'Lider':
			mainrole=i
			break

	if mainrole == 0:
		g1x=x1
		g1y=y1
		g2x=x1+d*cos(a+a1)
		g2y=y1+d*sin(a+a1)
		g3x=x2+d*cos(a+a2)
		g3y=y2+d*sin(a+a2)
		return g1x,g1y,g2x,g2y,g3x,g3y
	elif mainrole == 1:
		g2x=x2
		g2y=y2
		g1x=x2+d*cos(a+a2)
		g1y=y2+d*sin(a+a2)
		g3x=x1+d*cos(a+a1)
		g3y=y1+d*sin(a+a1)
		return g1x,g1y,g2x,g2y,g3x,g3y
	elif mainrole == 2:
		g3x=x3
		g3y=y3
		g1x=x3+d*cos(a+a3)
		g1y=y3+d*sin(a+a3)
		g2x=x1+d*cos(a+a1)
		g2y=y1+d*sin(a+a1)
		return g1x,g1y,g2x,g2y,g3x,g3y







