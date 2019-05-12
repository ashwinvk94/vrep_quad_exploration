import numpy as np



def map(pos, image = image):        # Image is the decomposed obstacle map
    return image[pos[0], pos[1]]


def InsideCellPlanning(startPos, corners, droneCamCover = 1):       # Inputs: start position (Image co-ordinates), corners of the cell 
    path = list()
    xs = list()
    ys = list()
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
            xUpdate = 1
        else:
            xUpdate = -1

        if startPos[0] - min(ys) < max(ys) - startPos[0]:
            yUpdate = droneCamCover 
        else:
            yUpdate = -droneCamCover 
        
        while (True):
            newPoint = [newPoint[0] , newPoint[1] + xUpdate]
            if map(newPoint) != 255:
                path.append(newPoint)
                if newPoint in corners:
                    counter += 1
            if map(newPoint) == 255:
                newPoint = [newPoint[0] + yUpdate, newPoint[1] - xUpdate]                
                xUpdate = - xUpdate
                while (map(newPoint) == 255):
                    newPoint = [newPoint[0] , newPoint[1] + xUpdate]
                path.append(newPoint)
                if newPoint in corners:
                    counter += 1
            if counter == len(corners):
                break




    if maxY  > maxX:  # Move in the Y direction
        if (startPos[0] - min(ys) < max(ys) - startPos[0]):        # Closer to bottom, passes should move up
            yUpdate = 1
        else:
            yUpdate = -1

        if startPos[1] - min(xs) < max(xs) - startPos[1]:
            xUpdate = droneCamCover 
        else:
            xUpdate = -droneCamCover 
        
        while (True):
            newPoint = [newPoint[0] + yUpdate, newPoint[1]]
            if map(newPoint) != 255:
                path.append(newPoint)
                if newPoint in corners:
                    counter += 1
            if map(newPoint) == 255:
                newPoint = [newPoint[0] -yUpdate, newPoint[1] + xUpdate]                
                yUpdate = -yUpdate
                while (map(newPoint) == 255):
                    newPoint = [newPoint[0] + yUpdate, newPoint[1]]
                path.append(newPoint)
                if newPoint in corners:
                    counter += 1
            if counter == len(corners):
                break

    path = np.array(path)

    return path
