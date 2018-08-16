import UR10CameraController
from ur10_simulation import ur10_simulator
from copy import copy
import time
import numpy as np
import realtimelaserdet as laser
import symmetry as sym
import math as mt
import cv2
import os, sys
sys.path.append('../iLimb')
from iLimb import *

import vision_tools as vt

# def getcom(image):
# 	comx=0   ######## x-coordinate Centre of mass of events in filtered image
# 	comy=0   #######  y-coordinate Centre of mass of events in filtered image
# 	total_events=0  ######  total evens in that time frame

# 	for i in range(128):
# 		for j in range(128):
# 			comx+=j*image[i][j]
# 			comy+=i*image[i][j]
# 			total_events+= image[i][j]

# 	comx= int(comx/total_events)
# 	comy= int(comy/total_events)

# 	return comx,comy

# def createboundingbox(image_orig,noofeventsreq,threshed,comx,comy,upper_bound):
# 	noofevents=0
# 	if((comx-2) >= 0):
# 		leftx= comx-2
# 	else:
# 		leftx=comx

# 	if((comx+2) < 128):
# 		rightx=comx+2
# 	else:
# 		rightx=comx

# 	if((comy-2) >= 0):
# 		bottomy= comy-2
# 	else:
# 		bottomy= comy

# 	if((comy+2) < 128):
# 		topy=comy+2
# 	else:
# 		topy=comy

	
# 	for i in range(leftx,rightx+1):
# 		for j in range(bottomy,topy+1):
# 			noofevents += image_orig[j][i]
# 	####### Expanding the bounding box one step in a direction corresponding to maximum imcrease

# 	while(noofevents<upper_bound):
# 		valltrt = 0
# 		valbttp= 0
# 		for i in range(bottomy,topy+1):
# 			if(leftx-1 >=0):
# 				valltrt += image_orig[i][leftx-1]
# 			if(rightx+1<128):
# 				valltrt += image_orig[i][rightx+1]

# 		for j in range(leftx,rightx+1):
# 			if(bottomy-1 >=0):
# 				valbttp += image_orig[bottomy-1][j]
# 			if(topy+1< 128):
# 				valbttp += image_orig[topy+1][j]

# 		if(valbttp>valltrt and noofevents>noofeventsreq):
# 			if(valbttp<(rightx-leftx)*threshed):
# 				break
# 		if(valbttp<=valltrt and noofevents>noofeventsreq):
# 			if(valbttp<(topy-bottomy)*threshed):
# 				break

# 		if(valbttp > valltrt):
# 			if(topy+1<128):
# 				topy+=1
# 			if(bottomy-1>=0):
# 				bottomy -=1
# 			noofevents += valbttp
# 		elif(valltrt > valbttp):
# 			if(leftx-1>=0):
# 				leftx-=1
# 			if(rightx+1<128):
# 				rightx +=1
# 			noofevents += valltrt
# 		else:
# 			if(topy+1<128 or bottomy-1>=0):
# 				if(topy+1<128):
# 					topy+=1
# 				if(bottomy-1>=0):
# 					bottomy -=1
# 				noofevents += valbttp
# 			else:
# 				if(leftx-1>=0):
# 					leftx-=1
# 				if(rightx+1<128):
# 					rightx +=1
# 				noofevents += valltrt

# 	if(leftx>=2):
# 		leftx=leftx-2
# 	if(rightx<=125):
# 		rightx+=2
# 	if(bottomy>=2):
# 		bottomy=bottomy-2
# 	if(topy<=125):
# 		topy+=2
# 	return leftx,rightx,bottomy,topy

# def getTargetLoc():
# 	U1.jitter_UR10(1.5)
# 	image_transformed = U1.remap(300, -200, 1200, 500)

# 	comx,comy=getcom(image_transformed)

# 	total_events=0
# 	for i in range(128):
# 		for j in range(128):
# 			total_events+=image_transformed[i][j]

# 	thresh = 0.8
# 	threshed=total_events/(128*128)

# 	noofevents=0    ########## number of events present in current bounding box
# 	noofeventsreq= int(thresh*total_events)  #####  number of events that should be present in the bounding box

# 	upper_bound=0.95*total_events
# 	leftx,rightx,bottomy,topy=createboundingbox(image_transformed,noofeventsreq,threshed,comx,comy,upper_bound)


# 	orientation = sym.getSymAngle(image_transformed)
# 	print(orientation)
	
# 	# orientation = 0
# 	# if (orientation1 > 90):
# 	# 	orientation = orientation2
# 	# else:
# 	# 	orientation = orientation1

# 	# sym_angle = sym.determine_sym_axis(image_transformed)

	
# 	print(orientation)
# 	cv2.rectangle(image_transformed,(leftx,bottomy),(rightx,topy),(100,100,100),3)
# 	cv2.imshow('image',image_transformed)
# 	cv2.waitKey(0)


# 	positiony=leftx+mt.floor((rightx-leftx)/2)
# 	positionx=bottomy+mt.floor((topy-bottomy)/2)

# 	deltax = 0
# 	deltay = 0

# 	xw= 1200 - (900*(positionx/128)) + deltax 
# 	yw= 500	- (700*(positiony/128)) + deltay 

# 	return (xw, yw, orientation)

# def pivot_move(U1,dist_pivot,delta_joints,t,orient,dist,Yoffset):
# 	Sim = ur10_simulator()
# 	U1.read_joints()
# 	print(U1.joints)
# 	Sim.set_joints(U1.joints)
# 	U1.xyzR = Sim.joints2pose()
# 	new_joints = copy(U1.joints)
# 	new_joints = new_joints + delta_joints
# 	new_xyzR = U1.move_joint_with_constraints(new_joints,dist_pivot)

# 	aux = copy(new_xyzR)
# 	aux[1] += (dist*np.cos(np.deg2rad(-orient)) - Yoffset*np.sin(np.deg2rad(-orient)))
# 	aux[0] += -(dist*np.sin(np.deg2rad(-orient)) + Yoffset*np.cos(np.deg2rad(-orient)))
# 	print(aux)
# 	a = input("check")
# 	U1.movej(aux,t)
# 	# self.UR10Cont.movej(new_xyzR,t)
# 	time.sleep(t+0.2)
# 	return new_xyzR

# # def startup():
# # 	port = 30003
# # 	ip1 = '10.1.1.4'
# # 	serialPort = 'COM8' #windows, for linux -> /dev/ttyACM0
# # 	U1 = UR10CameraController.UR10CameraController(ip1)
# # 	il = iLimbController(serialPort)
# # 	il.connect()

# # startup()

base = [25.22, -100, -126, -192, -78.25, 90] #update
MiddleXYZR = [656.96, 102.20, 80, 2.12, 0.47, 1.85]
for i in range(6):
	base[i] = np.deg2rad(base[i])

Sim = ur10_simulator()

U1, il = vt.initializeURiL()

while(True):
	print("Return to base")

	# port = 30003
	# ip1 = '10.1.1.4'
	# serialPort = 'COM8' #windows, for linux -> /dev/ttyACM0
	# U1 = UR10CameraController.UR10CameraController(ip1)
	# il = iLimbController(serialPort)
	# il.connect()

	U1.movejoint(base, 3)
	x = ['index','middle', 'thumb']
	c = ['position']*len(x)
	pos = [290]*len(x)
	il.control(x,c,pos)
	time.sleep(2)

	time.sleep(3.1)
	il.setPose('openHand')
	time.sleep(1.5)

	il.control('thumbRotator', 'position', 600)
	time.sleep(1.5) #necessary for waiting the thumb to fully rotate
	# #move the index finger
	# fingers = ['index', 'thumb', 'middle']
	# #action = position
	# cmds = ['position']*len(fingers)
	# #set the position
	# pos = [200]*len(fingers)
	# print("moving fingers")
	# #send the serial command
	# il.control(fingers,cmds,pos)
	# time.sleep(1.5)



	a = input("everything fine")


	(xw, yw, orientation, pose) = vt.getTarget(U1)
	print(xw, yw)

	U1.movej(MiddleXYZR, 3)
	time.sleep(3.1)

	if (pose == "horizontal"):
		Sim= ur10_simulator()
		U1.read_joints_and_xyzR()
		Sim.set_joints(copy(U1.joints))
		Sim.joints2pose()
		handlength = 180
		target = copy(U1.xyzR)
		target[0] = xw - handlength
		target[1] = yw 
		target[2] = 20

		print(target)
		a=input("End Pos? ")
		b = input("check again ")


		U1.movej(target,3)
		time.sleep(3.1)

		a = input("wait")

		U1.do_circular_pivot_motion(-25, 180,"z",5,50)

		# if (orientation <= 90):
		# 		orientation = orientation - 90
		# else:

		if (orientation < 90):
			orientation = -1*orientation
		elif (orientation > 90):
			orientation = 180 - orientation

		print(orientation, "final orientation")
		a = input("wait")
		new_xyzR = vt.pivot_move(U1,180,[0,0,0,0,orientation,0],10,orientation,0,0)

		a = input("wait")

		U1.read_joints()
		Sim.set_joints(U1.joints)
		Sim.tcp_vec = Sim.joints2pose()
		tcp_new,unit_vec = Sim.position_along_endaxis(180)
		U1.read_xyz()

		#go down
		aux = copy(U1.xyzR)
		aux[2] += -(175 + tcp_new[2])
		U1.movej(aux,5.5)

		time.sleep(5.6) #wait for the hand to go down

		# il.control('thumb', 'position', 230)
		# time.sleep(0.5)
		# f = ['index','middle']
		# a = ['position']*len(f)
		# p = [250]*len(f)
		# il.control(f,a,p)

		ti = 100

		for i in range(6):
			ti = ti + i*(20)
			il.control('thumb', 'position', ti)
			time.sleep(0.5)
			f = ['index','middle']
			a = ['position']*len(f)
			p = [ti]*len(f)
			il.control(f,a,p)
			time.sleep(0.5)
			#lift the object

	else:
		Sim= ur10_simulator()
		U1.read_joints_and_xyzR()
		Sim.set_joints(copy(U1.joints))
		Sim.joints2pose()
		handlength = 180
		target = copy(U1.xyzR)
		target[0] = xw - handlength
		target[1] = yw 
		target[2] = 20

		print(target)
		a=input("End Pos? ")
		b = input("check again ")


		U1.movej(target,3)
		time.sleep(3.1)







	aux[2] = 80
	U1.movej(aux,5.5)
	time.sleep(5.6)

	aux[1] += -350
	U1.movej(aux,5.5)
	time.sleep(5.6)

	#go down again
	aux[2] = -50
	U1.movej(aux,10.5)
	time.sleep(10.6)

	#open the hand
	il.setPose('openHand')
	#self.iLimbCont.setPose('openHand')
	time.sleep(2) #wait for the fingers to open

	#go up
	aux[2] = 80
	U1.movej(aux,10)
	time.sleep(10.1)

	# #go to home position
	# self.UR10PoseManager.moveUR(self.UR10Cont,'homej',4)
	# f = ['thumbRotator']
	# a = ['open']*len(f)
	# p = [297]*len(f)
	# self.iLimbCont.control(f,a,p)

	# time.sleep(4.1)


