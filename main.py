#!/usr/bin/env python2

'''
ENPM 661 Spring 2019 
Final Project: Quadcopter Exploration

Authors:
Ashwin Varghese Kuruttukulam(ashwinvk94@gmail.com)
Rachith Prakash (rachithprakash@gmail.com)
Midhun Bharadwaj
Graduate Students in Robotics,
University of Maryland, College Park

Note: The axis is defined as the x axis down and y axis up with the origin at the bottom left corner

'''

import sys
import vrep
import PID
import rospy
import quad_helper
import tf.transformations
from geometry_msgs.msg import Pose
import numpy as np
import time
import cv2

class pos_pub:
	def __init__(self):
		print 'started'
		# Init vrep client
		vrep.simxFinish(-1)
		objectsList = ['Wall1','Wall2','Wall3','Wall4','Wall5','Wall6']
		resolution = 0.01
		self.clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
		if self.clientID != -1:
			#The quad helper contains function implementations for various quadcopter functions
			quad_functions = quad_helper.quad_helper(self.clientID)
			print('Main Script Started')
			quad_functions.init_sensors()
			quad_functions.start_sim()
			#Setting initial time
			init_time = time.time()
			d1=0

		#Initialize flags
		self.obstacleAvoidFlag = False

		positions = [[0,0,3],[0,1,3],[-1,1,3],[-2,1,3],[-2,2,3],[-2,3,3],[-1,3,3],[0,3,3],[1,3,3]]

		# Rate init
		self.rate = rospy.Rate(20.0)  # MUST be more then 20Hz

		#Initializing publishers
		rospy.Subscriber("/quad_explore/target_position", Pose, self.posCb)
		
		#Code to shift the origin from the center to the bottom left
		obstOccMat = self.createObstacleOccupancyMat(objectsList,self.clientID,resolution)
		self.cellDecomposition(obstOccMat)
		err,self.quadObjectHandle = vrep.simxGetObjectHandle(self.clientID,'Quadricopter',vrep.simx_opmode_blocking)

		while not rospy.is_shutdown() and vrep.simxGetConnectionId(self.clientID) != -1:
			
			#Getting object position with respect to first joint
			err,obj_pos = vrep.simxGetObjectPosition(self.clientID,self.quadObjectHandle,-1,vrep.simx_opmode_blocking)

			current_time = time.time()
			elapsed_time =  current_time-init_time
			#Setting the position of the quadcopter
			ind = int(elapsed_time/3)
			#Looping the movement
			if ind > 2*len(positions)-1:
				print 'finished one'
				init_time = time.time()
			elif ind > len(positions)-1:
				ind_rev = len(positions)-1-ind
				self.quad_pos = positions[ind_rev]
			else:
				self.quad_pos = positions[ind]
			
			# print 'moving quad'
			quad_functions.move_quad(self.quad_pos)

			vrep.simxSynchronousTrigger(self.clientID);
			self.rate.sleep()
		quad_functions.stop_sim()
	
	'''
	Target Positon  callback
	'''
	def posCb(self,data):
		position = data.pose.position
		self.quad_pos = [position.x,position.y,position.z]

	'''
	Creates the obstacle space and occupancy matrix
	0's - Empty Cell
	1's - Obstacle Cell
	2's - Visited Cell
	'''
	def createObstacleOccupancyMat(self,objectsList,clientID,resolution):

		#Getting origin handle
		err,originHandle = vrep.simxGetObjectHandle(clientID,'Origin',vrep.simx_opmode_blocking)

		#Getting map size from the top wall and left wall length
		err,topWallHandle = vrep.simxGetObjectHandle(clientID,'Top_Wall',vrep.simx_opmode_blocking)
		err,halfTopWallLen = vrep.simxGetObjectFloatParameter(clientID,topWallHandle,18,vrep.simx_opmode_blocking)
		self.xMapLen = halfTopWallLen*2
		err,leftWallHandle = vrep.simxGetObjectHandle(clientID,'Left_Wall',vrep.simx_opmode_blocking)
		err,halfLeftWallLen = vrep.simxGetObjectFloatParameter(clientID,leftWallHandle,19,vrep.simx_opmode_blocking)
		self.yMapLen = halfLeftWallLen*2
		obstacleMap = np.zeros((int(self.yMapLen/resolution), int(self.xMapLen/resolution)))

		rectsInfo = []
		
		for objectName in objectsList:
			err,objectHandle = vrep.simxGetObjectHandle(clientID,objectName,vrep.simx_opmode_blocking)
			err,obj_pos = vrep.simxGetObjectPosition(clientID,objectHandle,originHandle,vrep.simx_opmode_blocking)
			err,maxX = vrep.simxGetObjectFloatParameter(clientID,objectHandle,18,vrep.simx_opmode_blocking)
			err,maxY = vrep.simxGetObjectFloatParameter(clientID,objectHandle,19,vrep.simx_opmode_blocking)
			lengthX = maxX*2
			lengthY = maxY*2
			topX = obj_pos[0]-maxX
			topY = obj_pos[1]+maxY
			# Rectangles Format[left top corner x,left top corner y,x length,y length]
			rectsInfo.append([topX,topY,lengthX,lengthY])
		for x in range(int(self.xMapLen/resolution)):
			for y in range(int(self.yMapLen/resolution)):
				# Recangles
				for rectInfo in rectsInfo:
					if x>=rectInfo[0]/resolution and y<=rectInfo[1]/resolution and y>=(rectInfo[1]-rectInfo[3])/resolution and x<=(rectInfo[0]+rectInfo[2])/resolution:
						obstacleMap[y,x] = 1
		# cv2.imshow('obstaclemap',obstacleMap)
		# cv2.waitKey(0)
		# cv2.destroyAllWindows()

		return obstacleMap



	def cellDecomposition(self, obstOccMat):

		obstOccMat = obstOccMat.astype(np.uint8)*255

		# cv2.imshow('obstaclemap',obstOccMat)
		# cv2.waitKey(0)

		map_copy = np.zeros_like(obstOccMat, dtype=np.uint8)

		# get edges
		edges = cv2.Canny(obstOccMat, 100, 255)	

		# get contours - same as canny output
		_,cnts,_ = cv2.findContours(obstOccMat.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
		cv2.drawContours(map_copy, cnts, -1, 255, 1)

		dst = cv2.cornerHarris(np.float32(map_copy), 3, 3, 0.04)	

		## Finding sub-pixel resolution corner points
		_, dst = cv2.threshold(dst,0.01*dst.max(),255,0)
		dst = np.uint8(dst)

		_, _, _, centroids = cv2.connectedComponentsWithStats(dst, 8, cv2.CV_32S)

		## define the criteria to refine the corners
		criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.001)
		corners = cv2.cornerSubPix(map_copy, np.float32(centroids), (6,6), (-1,-1), criteria)
		corners = np.round(corners)
		# print(corners)

		count = 0
		for corner in corners:
			if count == 0:
				count += 1
				continue

			y_upside = int(corner[1])

			while y_upside >=0:
				y_upside -= 1
				if obstOccMat[y_upside, int(corner[0])] == 255:
					break
			cv2.line(obstOccMat, (int(corner[0]), int(corner[1])), (int(corner[0]), y_upside), 255, 1)

			y_bottom = int(corner[1])

			while y_bottom < obstOccMat.shape[0]-1:
				y_bottom += 1
				if obstOccMat[y_bottom, int(corner[0])] == 255:
					break
			cv2.line(obstOccMat, (int(corner[0]), int(corner[1])), (int(corner[0]), y_bottom), 255, 1)

		# cv2.imshow('obstaclemap',obstOccMat)
		# cv2.waitKey(0)



def main(args):
    rospy.init_node('pos_pub', anonymous=True)
    ic = pos_pub()

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main(sys.argv)