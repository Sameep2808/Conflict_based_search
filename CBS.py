import os
import math
import numpy as np
from numpy import array
import cv2
import time
import matplotlib.pyplot as plt
import matplotlib.animation as animation 


def eq(x1,y1,x2,y2,x,y,f):
	m = (y2-y1)/(x2-x1)
	if (f == 1):
		c = (m*x) - y <= (m*x1) - y1 
	else:
		c = (m*x) - y >= (m*x1) - y1 
	return c

def create_map():
	m = np.zeros((250,400))
	am = np.zeros((250,400,3))
	hl = 40.4145
	for y in range(m.shape[0]):
		for x in range(m.shape[1]):
			if (((y - 65) ** 2) + ((x - 300) ** 2) <= ((40) ** 2) ):
				m[y,x]=1
				am[y,x]=[255,0,0]
			if ((x > 200-35) and (x < 200 + 35) and (y <= 150) and eq(200,150-hl,165,150-(hl/2),x,y,1) and eq(200,150-hl,235,150-(hl/2),x,y,1) ):
				m[y,x]=1
				am[y,x]=[255,0,0]
			if ((x > 200-35) and (x < 200 + 35) and (y >= 150) and eq(200,150+hl,165,150+(hl/2),x,y,2) and eq(200,150+hl,235,150+(hl/2),x,y,2) ):
				m[y,x]=1
				am[y,x]=[255,0,0]
			if (eq(36,65,115,40,x,y,1) and eq(36,65,105,150,x,y,2) and (eq(80,70,105,150,x,y,1) and (y >= 250-180))):
				m[y,x]=1
				am[y,x]=[255,0,0]
			if (eq(36,65,115,40,x,y,1) and eq(36,65,105,150,x,y,2) and (eq(115,40,80,70,x,y,2) and (y <= 250-180))):
				m[y,x]=1
				am[y,x]=[255,0,0]
	return m,am


def detect(x,y):
	global cll
	for cl in range(0,cll):
		if (250 - (y+cl) > 0):
			if (m[249-(y+cl)][x] == 1):
				#print("1")
				return True
		if (250 - (y-cl) <= 249):
			if (m[249-(y-cl)][x] == 1):
				#print("2")
				return True
		if ( x+cl < 399 ):
			if (m[249-y][x+cl] == 1):
				#print("3")
				return True
		if ( x-cl > 0 ):
			if (m[249-y][x-cl] == 1):
				#print("4")
				return True
		if (250 - (y+cl) > 0) and ( x+cl < 399 ):
			if (m[249-(y+cl)][x+cl] == 1):
				#print("1")
				return True
		if (250 - (y+cl) > 0) and ( x-cl > 0 ):
			if (m[249-(y+cl)][x-cl] == 1):
				#print("2")
				return True
		if (250 - (y-cl) <= 249) and ( x+cl < 399 ):
			if (m[249-(y-cl)][x+cl] == 1):
				#print("2")
				return True
		if (250 - (y-cl) <= 249) and ( x-cl > 0 ):
			if (m[249-(y-cl)][x-cl] == 1):
				#print("2")
				return True
	global rr
	#print(rr)
	if (250 - (y+rr) < 0) or (250 - (y-rr)  >= 249) or ( x-rr < 0 ) or ( x+rr > 399 ):
		return True
	return False


class Node:
	def __init__(self, data, orientaion, cost, parent, gcost, pxy, step):
		self.d = data
		self.o = orientaion
		self.c = cost
		self.p = parent
		self.g = gcost
		self.pxy = pxy
		self.s = step


def action(CurrentNode,L,theta):
	x,y = CurrentNode.d
	thetat = theta + CurrentNode.o
	xd = L*math.cos((math.pi/180)*thetat)
	yd = L*math.sin((math.pi/180)*thetat)
	if ((250-round(y+yd)) > 0) and (round(x+xd) < 399) and ((250-round(y+yd)) <= 250) and (round(x+xd) > 0):
		if (m[249-round(y+yd)][round(x+xd)] == 1):
			return None
		if (detect(round(x+xd),round(y+yd))):
			#print("Here")
			return None
		else:
			cost = CurrentNode.c
			gc = gcost(x+xd,y+yd)
			orientation = theta + CurrentNode.o
			if(orientation >= 360):
				orientation = orientation - 360
			if(orientation < 0):
				orientation = orientation + 360
			s = CurrentNode.s + 1
			child_node = Node([(x+xd),(y+yd)],orientation,cost+1.4,CurrentNode,gc,CurrentNode.d,s)
			return child_node
	else:
		return None 


def move(direction, node, L):
	if direction == 1:
		return action(node,L,60)
	elif direction == 2:
		return action(node,L,30)
	elif direction == 3:
		return action(node,L,0)
	elif direction == 4:
		return action(node,L,-30)
	elif direction == 5:
		return action(node,L,-60)
	elif direction == 6:
		return action(node,L,0)

def DS(node, g, L,ga):
	global goal 
	goal = g
	Q = [node]
	CL = []
	OL,cOL,OLc,P = [],[],[],[]
	OL.append(node.d)
	OLc.append(node.d)
	cOL.append(node.c)
	P.append(node.pxy)
	action_set = [1,2,3,4,5,6,7,8]
	p=0
	while Q:
		l = cOL.index(min(cOL))
		cn = Q.pop(l)
		o=cOL.pop(l)
		o=OL.pop(l)
		ox,oy=o
		CL.append(cn.d)
		if (((cn.d[1] - g[1]) ** 2) + ((cn.d[0] - g[0]) ** 2) <= ((L) ** 2)):
			return cn,CL,OLc,P
		for a in action_set:
			NewNode = move(a,cn,L) 
			if NewNode != None:
				if (NewNode.d not in CL):
					if(NewNode.d not in OL):
						Q.append(NewNode)
						OL.append(NewNode.d)
						OLc.append(NewNode.d)
						P.append(NewNode.pxy)
						cOL.append(NewNode.g)
					else:
						T = Q[OL.index(NewNode.d)]
						if(T.g > NewNode.g):
							T.p = NewNode.p
							T.pxy = NewNode.pxy




def reverse_path(node):
	path = []
	path = [node]
	c=0
	while node.p != None:
		x,y = node.d
		c=c+1
		# print(node.s)
		m[249-round(y)][round(x)] = 1
		node = node.p
		path.append(node)
	path.reverse()
	return path


def gcost(x,y):
	global goal
	xg,yg = goal
	ec = (x-xg)**2 + (y-yg)**2
	return ec


def get_input():
	global cll
	global rr
	
	print('Enter Initial X (Range: 0 - 399):')
	x = int(input())
	if (x - rr <0) or (x + rr >399):
		print('INVALID X SETTING INITIAL X AS LEAST RADIUS')
		x=rr
		
	print('Enter Initial Y (Range: 0 - 249):')
	y = int(input())
	if (y - rr <0) or (y +rr >249):
		print('INVALID Y SETTING INITIAL Y AS LEAST RADIUS')
		y=rr
	sa = 0
	
	if detect(x,y):
		print('INVALID POINTS SETTING INITIAL POINT AS [Robot Radius,Robot Radius]')
		x=rr
		y=rr
		
	print('Enter Goal X (Range: 0 - 399):')
	xg = int(input())
	if (xg - rr <0) or (xg + rr >399):
		print('INVALID X SETTING GOAL X AS 399')
		xg=399 - rr
		
	print('Enter Goal Y (Range: 0 - 249):')
	yg = int(input())
	if (yg - rr <0) or (yg + rr>249):
		print('INVALID Y SETTING GOAL Y AS 249')
		yg=249 - rr 
	
	if detect(xg,yg):
		print('INVALID POINTS SETTING INITIAL POINT AS [399,249]')
		xg=399 - rr
		yg=249 - rr
	ga = 0
		
	return [x,y],[xg,yg],1,sa,ga


def VM(P1,P2,P3,P4):
	ma = max(len(P1),len(P2),len(P3),len(P4))
	
	if ma != len(P1):
		l = [P1[len(P1)-1]]*(ma-len(P1))
		P1 = P1 + l
	if ma != len(P2):
		l = [P2[len(P2)-1]]*(ma-len(P2))
		P2 = P2 + l
	if ma != len(P3):
		l = [P3[len(P3)-1]]*(ma-len(P3))
		P3 = P3 + l
	if ma != len(P4):
		l = [P4[len(P4)-1]]*(ma-len(P4))
		P4 = P4 + l
	for z,z2,z3,z4 in zip(P1,P2,P3,P4):
		x,y = z.d
		
		px,py = z.pxy
		fm = am.copy()
		fm = cv2.circle(fm, (int(x),int(249-y)), 3, (0,0,255), -1)
		x2,y2 = z2.d
		fm = cv2.circle(fm, (int(x2),int(249-y2)), 3, (255,255,0), -1)
		x2,y2 = z3.d
		fm = cv2.circle(fm, (int(x2),int(249-y2)), 3, (255,0,0), -1)
		x2,y2 = z4.d
		fm = cv2.circle(fm, (int(x2),int(249-y2)), 3, (0,255,0), -1)
		cv2.imshow("Output", fm)
		cv2.waitKey(100)
	cv2.imshow("Final Path", fm)
	cv2.waitKey(0)
	cv2.destroyAllWindows()

def conflict(P1,P2,P3,P4):
	ma = max(len(P1),len(P2),len(P3),len(P4))
	
	if ma != len(P1):
		l = [P1[len(P1)-1]]*(ma-len(P1))
		P1 = P1 + l
	if ma != len(P2):
		l = [P2[len(P2)-1]]*(ma-len(P2))
		P2 = P2 + l
	if ma != len(P3):
		l = [P3[len(P3)-1]]*(ma-len(P3))
		P3 = P3 + l
	if ma != len(P4):
		l = [P4[len(P4)-1]]*(ma-len(P4))
		P4 = P4 + l

	for z,z2,z3,z4 in zip(P1,P2,P3,P4):
		return

def cbs():
	global goal
	while(True):
		# start,goal,L,sa,ga=get_input()
		sa = 0
		ga =0 
		L = 5
		start = [10,30]
		goal = [380,30]
		root = Node(start, sa, 0 , None, 0, start, 0)
		F,C,O,Pxy = DS(root,goal,L,ga)
		p1=reverse_path(F)
		# start,goal,L,sa,ga=get_input()
		start = [50,10]
		goal = [50,239]
		sa = 0
		root1 = Node(start, sa, 0 , None, 0, start, 0)
		F,C,O,Pxy = DS(root1,goal,L,ga)
		p2=reverse_path(F)

		start = [10,10]
		goal = [300,240]
		sa = 0
		root1 = Node(start, sa, 0 , None, 0, start, 0)
		F,C,O,Pxy = DS(root1,goal,L,ga)
		p3=reverse_path(F)

		start = [380,50]
		goal = [150,150]
		sa = 180
		root1 = Node(start, sa, 0 , None, 0, start,0)
		F,C,O,Pxy = DS(root1,goal,L,ga)
		p4=reverse_path(F)

		VM(p1,p2,p3,p4)
		break

if __name__ == '__main__':
	m,am = create_map()
	global cll
	global rr
	cll,rr = 5,5
	cbs()
	