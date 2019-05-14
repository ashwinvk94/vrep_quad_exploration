import numpy as np



# def map(pos, image = image):        # Image is the decomposed obstacle map
#     return image[pos[0], pos[1]]


def InsideCellPlanning(startPos, corners, obstacleMap, xJump, yJump):       # Inputs: start position (Image co-ordinates), corners of the cell 
	path = list()
	xs = list()
	ys = list()
	corners = corners.tolist()

	for i in range(len(corners)):
		xs.append(corners[i][1])
		ys.append(corners[i][0])

	maxX = max(xs) - min(xs)       # Max separation in x
	maxY = max(ys) - min(ys)       # Max separation in y
	counter = 1
	path.append(startPos)
	newPoint = startPos

	if maxX >= maxY:  # Move in the X direction
		if (startPos[1] - min(xs) < max(xs) - startPos[1]):        # Closer to bottom, passes should move right
			xUpdate = xJump
		else:
			xUpdate = -xJump

		if startPos[0] - min(ys) < max(ys) - startPos[0]:
			yUpdate = yJump
		else:
			yUpdate = -yJump 
		
		while (True):
			newPoint = [newPoint[0] , newPoint[1] + xUpdate]
			if obstacleMap[newPoint[0],newPoint[1]] != 255:
				path.append(newPoint)
				if distanceFromCorner(corners, newPoint, xJump):
					counter += 1
			if obstacleMap[newPoint[0],newPoint[1]] == 255:
				newPoint = [newPoint[0] + yUpdate, newPoint[1] - xUpdate]                
				xUpdate = - xUpdate
				# while (obstacleMap[newPoint[0],newPoint[1]] == 255):
					# newPoint = [newPoint[0] , newPoint[1] + xUpdate]
				path.append(newPoint)
				if distanceFromCorner(corners, newPoint, xJump):
					counter += 1
			if counter == len(corners):
				break

	if maxY  > maxX:  # Move in the Y direction
		if (startPos[0] - min(ys) < max(ys) - startPos[0]):        # Closer to bottom, passes should move up
			yUpdate = yJump
		else:
			yUpdate = -yJump

		if startPos[1] - min(xs) < max(xs) - startPos[1]:
			xUpdate = xJump 
		else:
			xUpdate = -xJump 
		
		while (True):
			newPoint = [newPoint[0] + yUpdate, newPoint[1]]
			if obstacleMap[newPoint[0],newPoint[1]] != 255:
				path.append(newPoint)
				if distanceFromCorner(corners, newPoint, xJump):
					counter += 1
			if obstacleMap[newPoint[0],newPoint[1]] == 255:
				newPoint = [newPoint[0] -yUpdate, newPoint[1] + xUpdate]                
				yUpdate = -yUpdate
				# while (obstacleMap[newPoint[0],newPoint[1]] == 255):
					# newPoint = [newPoint[0] + yUpdate, newPoint[1]]
				path.append(newPoint)
				if distanceFromCorner(corners, newPoint, xJump):
					counter += 1
			if counter == len(corners):
				break


	path = np.array(path[:len(path)-4])
	
	return path

def distanceFromCorner(corners, current_position, xJump):

	radius = xJump
	for corner in corners:
		if ((current_position[0]-corner[0])**2 + (current_position[1]-corner[1])**2 - radius**2)<= 0:
			return True

	return False

