#!/usr/bin/env python2

'''
ENPM 661 Spring 2019 
Final Project: Quadcopter Exploration

Authors:
Ashwin Varghese Kuruttukulam(ashwinvk94@gmail.com)
Rachith Prakash (rachithprakash@gmail.com)
Mithun Bharadwaj (mithunbharadwaj02@gmail.com)
Graduate Students in Robotics,
University of Maryland, College Park

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
import math
import time
from matplotlib import pyplot as plt
from InsideCellPlanning import InsideCellPlanning
np.set_printoptions(threshold=sys.maxsize)

class pos_pub:
	def __init__(self):

		print 'started'
		# Init vrep client
		vrep.simxFinish(-1)
		# list obstacles in vrep here for it to get detected and mapped into obstacle space
		objectsList = ['Wall1','Wall2','Wall3','Wall4','Wall5','Wall6', 'Right_Wall', 'Bottom_Wall', 'Top_Wall', 'Left_Wall']
		
		# resolution where 0.01 corresponds to 1cm in the simulator
		resolution = 0.02
		
		# the delay between consecutive movment of the quad
		self.moveDelay = 0.1
		
		# get clientID
		self.clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
		if self.clientID != -1:
			#The quad helper contains function implementations for various quadcopter functions
			self.quad_functions = quad_helper.quad_helper(self.clientID)
			print('Main Script Started')
			# self.quad_functions.init_sensors()
			self.quad_functions.start_sim()
			#Setting initial time
			init_time = time.time()
			d1=0

		#Initialize flags
		self.obstacleAvoidFlag = False

		#Getting map size from the top wall and left wall length
		err,topWallHandle = vrep.simxGetObjectHandle(self.clientID,'Top_Wall',vrep.simx_opmode_blocking)
		err,self.halfTopWallLen = vrep.simxGetObjectFloatParameter(self.clientID,topWallHandle,18,vrep.simx_opmode_blocking)
		err,leftWallHandle = vrep.simxGetObjectHandle(self.clientID,'Left_Wall',vrep.simx_opmode_blocking)
		err,self.halfLeftWallLen = vrep.simxGetObjectFloatParameter(self.clientID,leftWallHandle,19,vrep.simx_opmode_blocking)

		# used for simulation purpose only!!
		positions = [[0,0,3],[0,1,3],[-1,1,3],[-2,1,3],[-2,2,3],[-2,3,3],[-1,3,3],[0,3,3],[1,3,3]]

		# Rate init
		# self.rate = rospy.Rate(20.0)  # MUST be more then 20Hz

		#Initializing publishers
		# rospy.Subscriber("/quad_explore/target_position", Pose, self.posCb)
		
		#Code to shift the origin from the center to the bottom left
		obstOccMat = self.createObstacleOccupancyMat(objectsList,self.clientID,resolution)
		
		corners_cell_wise, image_with_cells = self.cellDecomposition(obstOccMat)
		nCells = corners_cell_wise.shape[0]
		self.cellOccupancyFlag = np.zeros(nCells).tolist()
		print self.cellOccupancyFlag
		
		startPos = [1,1]
		err,self.quadHandle = vrep.simxGetObjectHandle(self.clientID,'Quadricopter',vrep.simx_opmode_blocking)
		err = vrep.simxSetObjectPosition(self.clientID,self.quadHandle, -1, [startPos[0]-self.halfTopWallLen, startPos[1]-self.halfLeftWallLen, 1.0], vrep.simx_opmode_blocking)
		
		for _ in range(50):
			vrep.simxSynchronousTrigger(self.clientID)

		startPos = [int(startPos[1]/resolution), int(startPos[0]/resolution)]
		# endPos = [3,5]
		# endPos = [int(endPos[1]/resolution),int(endPos[0]/resolution)]
		# astarDist, path = self.astar(startPos,endPos, obstOccMat)
		# endPos = [3,5]

		# untul all cells are covered, run the algorithm!
		while 0 in self.cellOccupancyFlag:

			# get the closest cell to the current position
			currCell = self.moveToClosestCorner(corners_cell_wise,resolution,obstOccMat)
			# change coordinates from (x,y) to (y,x)
			currCellYX = self.convertCornerstoYX(currCell)
			# get current position of the quad
			err, obj_pos_origin = vrep.simxGetObjectPosition(self.clientID, self.quadHandle,self.originHandle,vrep.simx_opmode_blocking)
			start_pos_origin =  [int(obj_pos_origin[1]/resolution),int(obj_pos_origin[0]/resolution)]

			# create a new image of same size as obstcale map where everything other than the current cell is an obstacle space
			cell_image = np.ones_like(obstOccMat, dtype=np.uint8)*255
			cell_image[int(currCellYX[0, 0]): int(currCellYX[3, 0]), int(currCellYX[0, 1]):int(currCellYX[3, 1])] = 0

			# generate a path to traverse optimally inside the cell and process it
			path = InsideCellPlanning(start_pos_origin, currCellYX, cell_image, 10, 10)
			path = np.hstack((path, 100*np.ones((path.shape[0], 1), dtype = np.uint8)))
			path = (path*resolution).tolist()
			# move according to the path generated above
			self.simPath(path, self.moveDelay, resolution)
			print self.cellOccupancyFlag
			for _ in range(100):				# wait sometime for the quad to settle in
				vrep.simxSynchronousTrigger(self.clientID)

		# err,self.quadObjectHandle = vrep.simxGetObjectHandle(self.clientID,'Quadricopter',vrep.simx_opmode_blocking)

		# while not rospy.is_shutdown() and vrep.simxGetConnectionId(self.clientID) != -1:
			
		# 	#Getting object position with respect to first joint
		# 	err,obj_pos = vrep.simxGetObjectPosition(self.clientID,self.quadObjectHandle,-1,vrep.simx_opmode_blocking)

		# 	current_time = time.time()
		# 	elapsed_time =  current_time-init_time
		# 	#Setting the position of the quadcopter
		# 	ind = int(elapsed_time/3)
		# 	#Looping the movement
		# 	if ind > 2*len(positions)-1:
		# 		print 'finished one'
		# 		init_time = time.time()
		# 	elif ind > len(positions)-1:
		# 		ind_rev = len(positions)-1-ind
		# 		self.quad_pos = positions[ind_rev]
		# 	else:
		# 		self.quad_pos = positions[ind]
			
		# 	# print 'moving quad'
		# 	self.quad_functions.move_quad(self.quad_pos)

		# 	vrep.simxSynchronousTrigger(self.clientID)
		# 	self.rate.sleep()
		# self.quad_functions.stop_sim()
	
	'''
	This function converts the xy coordinates to yx coordinates
	'''
	def convertCornerstoYX(self,xycoordinates):
		X = xycoordinates[:,0].reshape(4,1)
		Y = xycoordinates[:,1].reshape(4,1)
		yxcoordinates = np.hstack((Y,X))
		return yxcoordinates


	def getNearestCorners(self, corners_cell_wise, current_position, resolution):
		radius = 500
		flag = False
		corner_list = []
		cell_list = []

		# go through each corner until you find a corner and then return the cell(corners) and the corner
		while not flag:
			for indCell, cell in enumerate(corners_cell_wise):
				if not self.cellOccupancyFlag[indCell]:
					for ind, corner in enumerate(cell):
						if (((current_position[0]/resolution)-corner[0])**2 + ((current_position[1]/resolution)-corner[1])**2 - radius**2)<= 0:
							flag = True
							corner_list.append([corner, ind])
							cell_list.append(cell)
			radius += 50
			if radius >= 1000:
				break

		# print 'radius', radius
		return corner_list, cell_list, flag


	'''
	This function moves the quadcopter to the closest corner
	'''
	def moveToClosestCorner(self, corners_cell_wise,resolution,obstOccMat):
		err, obj_pos = vrep.simxGetObjectPosition(self.clientID, self.quadHandle,-1,vrep.simx_opmode_blocking)
		err, obj_pos_origin = vrep.simxGetObjectPosition(self.clientID, self.quadHandle,self.originHandle,vrep.simx_opmode_blocking)
		# go through each corner of every cell and get nearest corners
		corners, cells, retVal = self.getNearestCorners(corners_cell_wise, obj_pos, resolution)

		if not retVal:
			print "No corner found to be within reach. Please check your code/map. Thank you!"
			self.quitGrace()

		dist = []
		path = []

		for corner in corners:

			if corner[1] == 0:
				endPos = [int(corner[0][1]+1), int(corner[0][0]+1)]
			elif corner[1] == 1:
				endPos = [int(corner[0][1]+1), int(corner[0][0]-1)]
			elif corner[1] == 2:
				endPos = [int(corner[0][1]-1), int(corner[0][0]+1)]
			else:
				endPos = [int(corner[0][1]-1), int(corner[0][0]-1)]
			start_pos_origin =  [int(obj_pos_origin[1]/resolution), int(obj_pos_origin[0]/resolution)]
			# print start_pos_origin
			try:
				astarDist, astarPath = self.astar(start_pos_origin, endPos, obstOccMat)
			except:
				print corner
				print obstOccMat[start_pos_origin[0], start_pos_origin[1]]
				print 'ASTAR FAILED'
				self.quitGrace()
			dist.append(astarDist)
			path.append(astarPath)

		ind = np.argmin(dist)
		next_path = np.array(path[ind])
		next_cell = np.array(cells[ind])
		next_corner = np.array(corners[ind])

		next_path = np.hstack((next_path, 100*np.ones((next_path.shape[0], 1), dtype = np.uint8)))
		next_path = (next_path*resolution).tolist()
		self.simPath(next_path, self.moveDelay, resolution)

		# update the cell being occupied
		complete_list = corners_cell_wise.tolist()
		update  = next_cell.tolist()
		ind = complete_list.index(update)
		self.cellOccupancyFlag[ind] = 1

		return next_cell

	'''
	Quit Gracefully
	'''
	def quitGrace(self):
		vrep.simxStopSimulation(self.clientID, vrep.simx_opmode_blocking)
		quit()


	'''
	Target Positon  callback
	'''
	def posCb(self,data):
		position = data.pose.position
		self.quad_pos = [position.x,position.y,position.z]

	'''
	Function to move the quad according to the input path
	'''
	def simPath(self,path,delay,resolution):

		for pos in path:
			# print 'Movement step in image coordinates', [pos[0]/resolution, pos[1]/resolution]
			err, obj_pos_origin = vrep.simxGetObjectPosition(self.clientID, self.quadHandle,self.originHandle,vrep.simx_opmode_blocking)
			# print '-------------------Feedback position', [obj_pos_origin[1]/resolution, obj_pos_origin[0]/resolution]
			posCorner = [(pos[1]-self.halfTopWallLen), (pos[0]-self.halfLeftWallLen), pos[2]/5.0]
			# posCorner = pos
			self.quad_functions.move_quad(posCorner)
			start_time = time.time()
			while time.time() - start_time<delay:
				vrep.simxSynchronousTrigger(self.clientID)


	def showImage(self, image):
		cv2.namedWindow("Display frame",cv2.WINDOW_NORMAL)
		cv2.imshow("Display frame",image)
		cv2.waitKey(0)


	def drawCorners(self, corners, image):

		if  len(corners.shape) == 1:
			cv2.circle(image, (int(corners[0]), int(corners[1])), 1, 255, -1)
			return image

		for corner in corners:
			cv2.circle(image, (int(corner[0]), int(corner[1])), 4, 255, -1)

		return image

	def cellDecomposition(self, obstacleMap):

		obstOccMat = obstacleMap.copy()

		obstOccMat = obstOccMat.astype(np.uint8)*255

		map_copy = np.zeros_like(obstOccMat, dtype=np.uint8)

		# get edges
		# edges = cv2.Canny(obstOccMat, 100, 255)	

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
		before_corner_image = np.zeros_like(obstOccMat, dtype=np.uint8)
		before_corner_image = self.drawCorners(corners, before_corner_image)

		# self.showImage(np.hstack((obstOccMat, before_corner_image)))

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

		# getting cells
		cells = np.zeros_like(obstOccMat, dtype=np.uint8)

		_,cnts,_ = cv2.findContours(obstOccMat.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
		# cv2.drawContours(cells, cnts, -1, 255, 1)

		# show cells
		for cnt in cnts:
			x,y,w,h = cv2.boundingRect(cnt)
			cv2.rectangle(cells, (x+1,y+1), (x+w-1,y+h-1), 255, 1)
			# self.showImage(cells)

		# find corners of each contour/cell
		cell_corners = []
		count = 0
		cell_corners_image = np.zeros_like(obstOccMat, dtype=np.uint8)
		temp_image = np.zeros_like(obstOccMat, dtype=np.uint8)
		for cnt in cnts:
			# skip first cell which is the map's boundary
			if count == 0:
				count += 1
				continue
			view_image = np.zeros_like(obstOccMat, dtype=np.uint8)
			x,y,w,h = cv2.boundingRect(cnt)
			cv2.rectangle(obstOccMat, (x+1,y+1), (x+w-1,y+h-1), 255, 1)
			cv2.rectangle(view_image, (x+1,y+1), (x+w-1,y+h-1), 255, 1)
			dst = cv2.cornerHarris(np.float32(view_image), 3, 3, 0.04)	

			## Finding sub-pixel resolution corner points
			_, dst = cv2.threshold(dst,0.01*dst.max(),255,0)
			dst = np.uint8(dst)
 
			_, _, _, centroids = cv2.connectedComponentsWithStats(dst, 8, cv2.CV_32S)

			# define the criteria to refine the corners
			criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.001)
			new_corners = cv2.cornerSubPix(view_image, np.float32(centroids), (3,3), (-1,-1), criteria)
			new_corners = np.round(new_corners)
			cell_corners.append(new_corners[1:])
			# cell_corners_image = self.drawCorners(new_corners, cell_corners_image)
			# self.showImage(np.hstack((cell_corners_image, obstacleMap)))

		# print len(cell_corners)
		# print np.array(cell_corners)
		# self.showImage(obstOccMat)
		# cv2.destroyAllWindows()

		return np.array(cell_corners), obstOccMat

	
	def astar(self,startPos,endPos, obstacleMap):
		'''
		Main astar function
		Initial and final position is in the format x,y
		'''
		obstOccMat = obstacleMap.copy()
		#Astar Init
		self.priorityQueue = self.astarInit(obstOccMat)
		yMapLen = obstOccMat.shape[0]
		xMapLen = obstOccMat.shape[1]
		self.insert([startPos[0],startPos[1]])
		yFinal = endPos[0]
		xFinal = endPos[1]
		validFlag = self.checkValidInputs([startPos[0],startPos[1]],[yFinal,xFinal],obstOccMat,xMapLen,yMapLen)
		if not validFlag:
			print 'Inputs are not valid'
			self.quitGrace()
		#Initializing distance array value of init node as 0 and visited arr value of init node value as 1
		self.nodeComeDistArr[startPos[0],startPos[1]] = 0
		self.nodeVisitArr[startPos[0],startPos[1]] = 1
		# print len(self.nodeParentArr)
		# print len(self.nodeParentArr[0])
		# print obstOccMat.shape
		# print self.nodeTotalDistArr.shape

		#Set initial node as current node - (y,x)
		currNode = [startPos[0],startPos[1]]
		visitedNodes = []

		while not self.isEmpty():
			#Making the current node the one that was popped from the queue	
			currNode = self.pop()

			#Checking whether goal node is reached
			if currNode==[yFinal,xFinal]:
				goalFlag=1
				break
			yCurr = currNode[0]
			xCurr = currNode[1]

			newIndices = [[yCurr,xCurr-1],[yCurr,xCurr+1],[yCurr-1,xCurr],[yCurr+1,xCurr]]
			for newIndex in newIndices:
				yCheckIndex = newIndex[0]
				xCheckIndex = newIndex[1]
				# print yCheckIndex,xCheckIndex

				#Skip if the index lies outside the map
				if xCheckIndex<0 or xCheckIndex>= xMapLen or yCheckIndex<0 or yCheckIndex>= yMapLen:
					continue

				#Skip if the index is a obstacle
				# print obstOccMat[yCheckIndex,xCheckIndex]
				if obstOccMat[yCheckIndex, xCheckIndex]==1:
					# print 'obstacle'
					continue

				euclDist = self.eucldDist([yCheckIndex,xCheckIndex],[yFinal,xFinal])
				if self.nodeVisitArr[yCheckIndex,xCheckIndex]==0:
					visitedNodes.append([yCheckIndex,xCheckIndex])
					#If the node is not visited, then mark it as visited and add to the priority queue
					self.nodeVisitArr[yCheckIndex,xCheckIndex]=1
					self.insert([yCheckIndex,xCheckIndex])
					self.nodeParentArr[yCheckIndex][xCheckIndex] = [yCurr,xCurr]
					weight = 1
					self.nodeComeDistArr[yCheckIndex,xCheckIndex] = self.nodeComeDistArr[yCurr,xCurr] + weight
					self.nodeGoDistArr[yCheckIndex,xCheckIndex] = euclDist
					self.nodeTotalDistArr[yCheckIndex,xCheckIndex] = self.nodeComeDistArr[yCheckIndex,xCheckIndex] + self.nodeGoDistArr[yCheckIndex,xCheckIndex]
				else:
					weight = 1
					if self.nodeComeDistArr[yCheckIndex,xCheckIndex] > self.nodeComeDistArr[yCurr,xCurr] + weight:
						self.nodeComeDistArr[yCheckIndex,xCheckIndex] = self.nodeComeDistArr[yCurr,xCurr] + weight
						self.nodeGoDistArr[yCheckIndex,xCheckIndex] = euclDist
						self.nodeTotalDistArr[yCheckIndex,xCheckIndex] = self.nodeComeDistArr[yCheckIndex,xCheckIndex] + self.nodeGoDistArr[yCheckIndex,xCheckIndex]
						self.nodeParentArr[yCheckIndex][xCheckIndex] = [yCurr,xCurr]
		
		optimalRoute = self.backtrack(self.nodeParentArr,[startPos[0],startPos[1]],[yFinal,xFinal])
		optimalRoute.reverse()
		# for pathPoint in optimalRoute:
		# 	obstOccMat[pathPoint[0],pathPoint[1]] = 120
		# self.showImage(obstOccMat)
		# print optimalRoute
		return len(optimalRoute), optimalRoute


	'''
	Initialized astar variables
	'''
	def astarInit(self,obstacleMap):
		yMapLen = obstacleMap.shape[0]
		xMapLen = obstacleMap.shape[1]
		self.queue = []
		self.nodeComeDistArr = np.full((yMapLen, xMapLen), np.inf)
		self.nodeGoDistArr = np.full((yMapLen, xMapLen), np.inf)
		self.nodeTotalDistArr = np.full((yMapLen, xMapLen), np.inf)
		self.nodeVisitArr = np.zeros((yMapLen,xMapLen))
		self.nodeParentArr = [[[None,None] for j in range(xMapLen)] for i in range(yMapLen)]

	def eucldDist(self,node,goalNode):
		x1 = node[0]
		y1 = node[1]
		x2 = goalNode[0]
		y2 = goalNode[1]

		return math.sqrt((x1-x2)**2+(y1-y2)**2)
	'''
	Astar functions
	'''
	# for checking if the queue is empty 
	def isEmpty(self): 
		return len(self.queue) == 0  
  
	# for inserting an element in the queue 
	def insert(self, data): 
		self.queue.append(data) 
  
	# for popping an element based on Priority 
	def pop(self):
		'''
		Start search setting the first element as the supposed 
		minimum value and then compared with all the other elements whether it
		id actually the largest, if not then updated the new value as the 
		largest
		'''
		minind = 0
		minY = self.queue[0][0] 
		minX = self.queue[0][1]
		# print len(self.queue)
		for i in range(len(self.queue)):
			y = self.queue[i][0]
			x = self.queue[i][1]
			if self.nodeTotalDistArr[y,x] < self.nodeTotalDistArr[minY,minX]: 
				minind = i
				minX = x 
				minY = y
				# print 'new min'
				# print minX
				# print minY
		item = self.queue[minind]
		del self.queue[minind]
		# print 'length'
		# print len(self.queue)
		return item

	def backtrack(self,ParentArr,startNode,goalNode):
		backTrackList = []
		currNode = goalNode
		while currNode!=startNode:
			backTrackList.append(currNode)
			currNode = self.nodeParentArr[currNode[0]][currNode[1]]
		backTrackList.append(startNode)
		return backTrackList
	def checkValidInputs(self,initPos,finalPos,obstacleMap,xMapLen,yMapLen):
		if initPos[0]<0 or initPos[1]<0 or initPos[1]>xMapLen or initPos[0]>yMapLen:
			print 'Initial position index is out of bounds'
			return False
		elif finalPos[0]<0 or finalPos[1]<0 or finalPos[1]>xMapLen or finalPos[0]>yMapLen:
			print 'Final position index out of bounds'
			return False
		elif obstacleMap[initPos[0],initPos[1]]==1:
			print obstacleMap[initPos[0],initPos[1]]
			print 'Initial position is inside an obstacle'
			return False
		elif obstacleMap[finalPos[0],finalPos[1]]==1:
			print 'Final position is inside an obstacle'
			return False
		else:
			return True


	'''
	Creates the obstacle space and occupancy matrix
	0's - Empty Cell
	1's - Obstacle Cell
	2's - Visited Cell
	'''
	def createObstacleOccupancyMat(self,objectsList,clientID,resolution):

		#Getting origin handle
		err,self.originHandle = vrep.simxGetObjectHandle(clientID,'Origin',vrep.simx_opmode_blocking)

		#Getting map size from the top wall and left wall length
		err,topWallHandle = vrep.simxGetObjectHandle(clientID,'Top_Wall',vrep.simx_opmode_blocking)
		err,self.halfTopWallLen = vrep.simxGetObjectFloatParameter(clientID,topWallHandle,18,vrep.simx_opmode_blocking)
		self.xMapLen = self.halfTopWallLen*2
		err,leftWallHandle = vrep.simxGetObjectHandle(clientID,'Left_Wall',vrep.simx_opmode_blocking)
		err,self.halfLeftWallLen = vrep.simxGetObjectFloatParameter(clientID,leftWallHandle,19,vrep.simx_opmode_blocking)
		self.yMapLen = self.halfLeftWallLen*2
		obstacleMap = np.zeros((int(self.yMapLen/resolution), int(self.xMapLen/resolution)))

		rectsInfo = []
		radius = 0.18
		for objectName in objectsList:
			err,objectHandle = vrep.simxGetObjectHandle(clientID,objectName,vrep.simx_opmode_blocking)
			err,obj_pos = vrep.simxGetObjectPosition(clientID,objectHandle,self.originHandle,vrep.simx_opmode_blocking)
			err,maxX = vrep.simxGetObjectFloatParameter(clientID,objectHandle,18,vrep.simx_opmode_blocking)
			err,maxY = vrep.simxGetObjectFloatParameter(clientID,objectHandle,19,vrep.simx_opmode_blocking)
			lengthX = maxX*2+2*radius
			lengthY = maxY*2+2*radius
			topX = obj_pos[0]-maxX-radius
			topY = obj_pos[1]+maxY+radius
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


def main(args):
    # rospy.init_node('pos_pub', anonymous=True)
    try:
    	ic = pos_pub()
    except:
    	self.quitGrace()

    # try:
    #     rospy.spin()
    # except rospy.ROSInterruptException:
    #     self.quitGrace()

if __name__ == '__main__':
    main(sys.argv)