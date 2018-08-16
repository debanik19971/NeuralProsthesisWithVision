import myread as load
import numpy as np
import matplotlib.pyplot as plt
import cv2
import math as mt
from mpl_toolkits.mplot3d import Axes3D
from scipy import ndimage
import filtertd as filt
import time
import peakutils
def getSymAngle(images):
# output = load.xypt('data_dvslaser/bottle_wallet_phone.aedat')

	
	displayimages=[[0 for i in range(128)] for j in range(128)]


	images = ndimage.filters.median_filter(images,size  = 3)

	# for i in range(128):
	# 	for j in range(128):
	# 		displayimages[i][j]=images[127-i][j]

	filteredimages=[[0 for i in range(128)] for j in range(128)]
	t1 = time.time()
	distancetransformimage=ndimage.distance_transform_edt(np.logical_not(images))
	print(time.time() - t1)
	sigma=5.0
	comx=0.0
	comy=0.0
	totalevents=0

	for i in range(128):
		for j in range(128):
			# intensity=images[i][j]
			intensity= distancetransformimage[i][j]
			filteredimages[i][j] = mt.exp(-(intensity/sigma)**2)
			comx+=j*filteredimages[i][j]
			comy+=i*filteredimages[i][j]
			totalevents+=filteredimages[i][j]

	filteredimages = np.array(filteredimages)

	# # for i in range(

	for i in range(128):
		for j in range(128):
			displayimages[i][j]=filteredimages[127-i][j]


	# for i in range(128):
	# 	for j in range(128):
	# 		comx+=j*filteredimages[i][j]
	# 		comy+=i*filteredimages[i][j]
	# 		totalevents+=filteredimages[i][j]

	comx=mt.floor(comx/totalevents)
	comy=mt.floor(comy/totalevents)

	S=[0 for i in range(36)]
	for var in range(0,36):
		theta=5*var
		theta = 180-theta
		denom=0
		# print(theta)
		for i in range(128):
			for j in range(128):
				p=i-comy
				q=j-comx
				radiantheta = theta*mt.pi/180
				newx= 2*(comx+p*mt.sin(radiantheta)*mt.cos(radiantheta) + q*((mt.cos(radiantheta))**2)) - j
				newy = 2*(comy+q*mt.sin(radiantheta)*mt.cos(radiantheta) + p*((mt.sin(radiantheta))**2)) - i
				
				# newx = (mt.cos(2*radiantheta)*i + mt.sin(2*radiantheta)*j) 
				# newy = (mt.sin(2*radiantheta)*i - mt.cos(2*radiantheta)*j)

				newx=int(newx)
				newy=int(newy)
				# print(theta)
				# print(j,i)
				# print(newx,newy)
				# print("fwfe")

				denom += (filteredimages[i][j])**2
				if(newy>=0 and newy<128 and newx>=0 and newx<128):
					#print(filteredimages[i][j], " ", filteredimages[newy][newx])
					S[var] += filteredimages[i][j]*filteredimages[newy][newx]
		S[var]=S[var]/denom

	S_Axis = []
	Angle = []

	print(S)
	half_length = int(((len(S)/2)-1))

	snew = S+S[0:half_length]
	indexes = peakutils.indexes(snew, thres=0.05/max(snew))

	print(indexes)

	for idx in indexes:
		idx = idx % 36
		Angle.append(idx)

	# for i in range(0,36):
	# 	left = S[i-1]
	# 	if (i == 35):
	# 		right = S[0]
	# 	else:
	# 		right = S[i+1]
	# 	if(S[i]>left and S[i]>right):
	# 		S_Axis.append(S[i])
	# 		Angle.append(i)

	# print(S)

	# sortedS = sorted(S_Axis)
	# largestS = sortedS[-1]
	# secondS = sortedS[-2]
	# # thirdS = sortedS[-3]

	# i1 = S_Axis.index(largestS)
	# i2 = S_Axis.index(secondS)
	# # i3 = S_Axis.index(thirdS)

	# print("First symmetry axis = " , Angle[i1]*5)
	# print("Second symmetry axis = ", Angle[i2]*5)
	# print("Third symmetry axis = ", Angle[i3]*2)


	# displayimages= np.array(displayimages)
	# maxi =np.amax(displayimages)
	# # print(maxi)
	# displayimages= np.divide(displayimages,maxi)
	# cv2.namedWindow('image', cv2.WINDOW_NORMAL)
	# cv2.imshow('image', displayimages)
	# cv2.waitKey(0)

	# return(Angle[i1]*5, Angle[i2]*5)

	wdk=[0 for i in range(len(Angle))]
	counter=0
	for var in Angle:
		wk=[[0 for i in range(128)] for j in range(128)]
		theta = 5*var
		theta = 180 - theta
		for i in range(128):
			for j in range(128):
				p=i-comy
				q=j-comx
				radiantheta = theta*mt.pi/180
				newx= 2*(comx+p*mt.sin(radiantheta)*mt.cos(radiantheta) + q*((mt.cos(radiantheta))**2)) - j
				newy = 2*(comy+q*mt.sin(radiantheta)*mt.cos(radiantheta) + p*((mt.sin(radiantheta))**2)) - i

				# newx = (mt.cos(2*radiantheta)*i + mt.sin(2*radiantheta)*j) 
				# newy = (mt.sin(2*radiantheta)*i - mt.cos(2*radiantheta)*j) 
				newx=int(newx)
				newy=int(newy)
				if(newy>=0 and newy<128 and newx>=0 and newx<128):
					wk[i][j]= filteredimages[i][j]*filteredimages[newy][newx]

		wk = np.array(wk)
		wk_disp = np.array(wk)
		maxi =np.amax(wk_disp)
		# print(maxi)
		wk_disp= np.divide(wk_disp,maxi)
		cv2.namedWindow('mirror', cv2.WINDOW_NORMAL)
		print(theta)
		cv2.imshow('mirror', wk_disp)
		cv2.waitKey(0)


		denom = 0

		wk = cv2.Sobel(wk, cv2.CV_64F, 0, 1, ksize=3)
		wk = np.abs(wk)
		for y in range(128):
			for x in range(128):
				wkxy = wk[y][x]
				p = x - comx
				q = y - comy
				Dist = np.abs((q-(p*mt.tan(radiantheta)))*mt.cos(radiantheta))
				wdk[counter]+= Dist*wkxy
				# wdk[counter]+= wkxy*(y-x*mt.tan(radiantheta)+comx*mt.tan(radiantheta)-comy)/(1+(mt.tan(radiantheta))**2)
				denom+=wkxy
		wdk[counter]=wdk[counter]/(denom)
		# print(Angle[counter]*5, wdk[counter])
		counter+=1

	# print(np.array(Angle)*5)
	# print(wdk)




	# wdk=[0 for i in range(len(maxima))]
	# counter=0
	# for var in maxima:
	# 	theta=5*var
	# 	denom=0
	# 	for i in range(128):
	# 		for j in range(128):
	# 			wkxy = wk[i][j]
	# 			wdk[counter]+= wkxy*(i-j*mt.tan(theta)+comx*mt.tan(theta)-comy)/(1+(mt.tan(theta))**2)
	# 			denom+=wkxy
	# 	wdk[counter]=wdk[counter]/denom
	# 	counter+=1

	# print("hi")
	# print(wdk)


	displayimages= np.array(filteredimages)
	maxi =np.amax(filteredimages)
	# print(maxi)
	displayimages= np.divide(filteredimages,maxi)
	cv2.namedWindow('filtimage', cv2.WINDOW_NORMAL)
	cv2.imshow('filtimage', displayimages)
	cv2.waitKey(0)

	min_dist = 100000
	min_idx = -1

	print(np.array(Angle) * 5)
	print(wdk)

	for i in range(len(wdk)):
		if (wdk[i] < min_dist):
			min_dist = wdk[i]
			min_idx = i

	print("dist: ", min_dist, Angle[min_idx]*5)
	return(Angle[min_idx]*5)

	
