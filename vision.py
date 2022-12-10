import cv2
import numpy as np
import math
from IPython.display import display, clear_output
import time

## FIELD PARAMETERS ##
fieldWidthM = 0.841  # width of A0 sheet [m]
fieldLengthM = 1.189  # length of A0 sheet [m]
fieldWidthP = 480  # width of FoV camera [pixels]
fieldLengthP = 640  # width of FoV camera [pixels]

# POSITION SEGMENTATION - GREEN
TLowPos = np.array([245, 245, 245])
THighPos = np.array([255, 255, 255])

# GOAL SEGMENTATION - DARK RED
TLowGoal = np.array([37, 20, 81])
THighGoal = np.array([106, 84, 167])
#TLowGoal = np.array([57, 32, 137])
#THighGoal = np.array([77, 52, 157])

# DIRECTION SEGMENTATION - RED
TLowDir = np.array([180, 120, 230])
THighDir = np.array([255, 255, 255])
# TLowDir = np.array([150,130,200])
# THighDir = np.array([200,200,255])

# OBSTACLES SEGMENTATION - BLACK
TLowObs = np.array([0, 0, 0])
THighObs = np.array([50, 50, 50])

# CUSTOM STRUCTURAL ELEMENT BLOB DILATATION
SE = np.array([[0, 0, 1, 0, 0],
               [0, 1, 1, 1, 0],
               [1, 1, 1, 1, 1],
               [0, 1, 1, 1, 0],
               [0, 0, 1, 0, 0]], 'uint8')


## FUNCTIONS ##

# Frame Reading
def frameReader(videoFeed):
    ''' 
    Frame reader function: Reads the frame from the video feed and apply median filter
    Inputs:
        videoFeed - video feed
    Outputs:
        frame.astype('uint8') - filtered frame, converted in uint8
    '''
    try:
        ret, frame = videoFeed.read()
        frame = cv2.medianBlur(frame, 5)
        return frame.astype('uint8')
    except:
        #clear_output(wait=True)
        #print("Frame can not be read")
        raise TypeError("Frame can not be read")


# Fetching odometry
def odoFetch(videoFeed):
    ''' 
    Position and angle fetching function: Tries to find the robot's position and direction
    Inputs:
        videoFeed - video feed
    Outputs:
        odometry - array with [x,y] position of the robot [m] and its angle [rad] with respect to the x axis
    '''
  
    frame = frameReader(videoFeed)

    posIsFetch = False
    angleIsFetch = False

    # Fetching green blobs
    maskPos = cv2.inRange(frame, TLowPos, THighPos) # image segmentation for green/white pixels
    totalLabelsPos, _, statsPos, BlobPos = cv2.connectedComponentsWithStats(maskPos, connectivity=8) # blob analysis
    
    # print(statsPos[:, cv2.CC_STAT_AREA] )
    BlobPosCenter = []
    for i in range(1, totalLabelsPos):
        areaBlobPos = statsPos[i, cv2.CC_STAT_AREA]
        if (areaBlobPos > 70) and (areaBlobPos < 350):  # filtering the blobs by area, only want 2
            BlobPosCenter.append(BlobPos[i])

    if np.shape(BlobPosCenter)[0] == 2:  # if only two green blobs left (after filtering) center is computed
        robotPos = [BlobPosCenter[0][0] + (BlobPosCenter[1][0] - BlobPosCenter[0][0]) / 2,
                    BlobPosCenter[0][1] + (BlobPosCenter[1][1] - BlobPosCenter[0][1]) / 2]
        robotPos[0] = fieldLengthP - robotPos[0] # inverting the x axis for the other modules
        robotPos = np.rint(robotPos).astype(np.int32) 
        posIsFetch = True

    if posIsFetch: # if position is obtained, the blob for the direction is worth finding
        maskDir = cv2.inRange(frame, TLowDir, THighDir) # image segmentation for red/white pixels
        totalLabelsDir, _, statsDir, BlobDir = cv2.connectedComponentsWithStats(maskDir, connectivity=8) # blob analysis

        # print(statsDir[:, cv2.CC_STAT_AREA])
        BlobDirCenter = []
        goodBlobsIdx = []
        for i in range(1, totalLabelsDir):
            areaBlobDir = statsDir[i, cv2.CC_STAT_AREA]
            # if (areaBlobDir > 25) and (areaBlobDir < 150): # Size of the desired blobs
            if (areaBlobDir > 10) and (areaBlobDir < 40):  # filtering the blobs by area, only want 1
                BlobDirCenter.append(BlobDir[i])
                goodBlobsIdx.append(i)

        if np.shape(BlobDirCenter)[0] == 1:  # if only one blob, the center is found and the angle computed
            BlobDirCenter = [j for sub in BlobDirCenter for j in sub]
            BlobDirCenter[0] = fieldLengthP - BlobDirCenter[0]
            angle = math.atan2(BlobDirCenter[1] - robotPos[1], BlobDirCenter[0] - robotPos[0])
            angleIsFetch = True
        # if more than 1 but less than 3 blobs are found, the blob with max area is chosen
        elif ((np.shape(BlobDirCenter)[0] <= 3) and (np.shape(BlobDirCenter)[0] > 1)): # if more than
            maxBlob = 0
            for idx in goodBlobsIdx:
                if (statsDir[idx, cv2.CC_STAT_AREA] >= maxBlob):
                    BlobDirCenter = BlobDir[idx]
            BlobDirCenter[0] = fieldLengthP - BlobDirCenter[0]  # inverting the x axis for the other modules
            angle = math.atan2(BlobDirCenter[1] - robotPos[1], BlobDirCenter[0] - robotPos[0]) # direction is computed
            angleIsFetch = True

    # if both the angle and position are found, everything is returned in an array
    if (angleIsFetch and posIsFetch) == True:
        odometry = np.append(robotPos, angle)
        return odometry
    else:
        #clear_output(wait=True)
        #print("Odometry can not be found")
        raise TypeError("Odometry can not be found")


# Fetching goal
def goalFetch(videoFeed):
    ''' 
    goal fetching function: Tries to find the goal on the map
    Inputs:
        videoFeed - video feed
    Outputs:
        goal position [m]
    '''
    try:
        while (True):
            frame = frameReader(videoFeed)
            maskGoal = cv2.inRange(frame, TLowGoal, THighGoal) # image segmentation for dark red pixels
            totalLabelsGoal, _, statsGoal, BlobGoal = cv2.connectedComponentsWithStats(maskGoal, connectivity=8) # blob analysis

            BlobGoalCenter = []
            for i in range(1, totalLabelsGoal):
                areaBlobGoal = statsGoal[i, cv2.CC_STAT_AREA]
                if (areaBlobGoal > 900) and (areaBlobGoal < 1300):
                #if (areaBlobGoal > 200) and (areaBlobGoal < 1000): # filtering the blobs by area, only want 1
                    BlobGoalCenter.append(BlobGoal[i])

            if np.shape(BlobGoalCenter)[0] == 1: # if only one blob, the center is found and the position computed
                goal = np.array([fieldLengthP - BlobGoalCenter[0][0], BlobGoalCenter[0][1]]) # inverting the x axis for the other modules
                return goal.astype(np.float) * fieldWidthM / fieldWidthP # conversion of the goal position to meters
    except:
        #clear_output(wait=True)
        #print("Goal can not be found")
        raise TypeError("Goal can not be found")


# Fetching terrain
def terrainFetch(videoFeed, goalM):
    # conversion of the goal position to pixels
    goal = goalM * fieldWidthP / fieldWidthM 

    frame = frameReader(videoFeed)
    
    # image segmentation for dark pixels (obstacle)
    maskObs = cv2.inRange(frame, TLowObs, THighObs) 
    totalLabelsObs, labelIdObs, statsObs, BlobObs = cv2.connectedComponentsWithStats(maskObs, connectivity=8) # blob analysis
    maskObs = np.zeros(maskObs.shape, dtype="uint8")
    
    # Getting rid of the noise on our mask
    for i in range(1, totalLabelsObs):  
        areaBlobObs = statsObs[i, cv2.CC_STAT_AREA]
        
         # filtering the blobs by area (the obstacle have a minimum size)
        if (areaBlobObs > 3000) and (areaBlobObs < 10000):
            componentMask = (labelIdObs == i).astype("uint8") * 255
            maskObs = cv2.bitwise_or(maskObs, componentMask)
    
    # mask dilatation to take the robot's size into account
    maskObsDilated = cv2.dilate(maskObs, SE, iterations=23)
    maskObsMargin = cv2.dilate(maskObsDilated, SE, iterations=5)
    
    # finding the dilated obsacles' corners
    dst = cv2.cornerHarris(maskObsMargin, 20, 3, 0.04)  # Fetching the angles
    blobCorner = np.zeros(frame.shape[:2], 'uint8')
    blobCorner[dst > 0.04 * dst.max()] = 1
    
    # nodes are the center of the dilated obsacles' corners 
    _, _, _, nodes = cv2.connectedComponentsWithStats(blobCorner, connectivity=8)

    odo = odoFetch(videoFeed)
    # adding position to nodes, and reconverting x wrt the image origin
    nodes = np.append(nodes, [[fieldLengthP - odo[0], odo[1]]], axis=0)
    # adding goal to nodes, and reconverting x wrt the image origin
    nodes = np.append(nodes, [[fieldLengthP - goal[0], goal[1]]], axis=0)
    nodes = np.rint(nodes[1:]).astype(np.int32)
    
    # matrix saving the nodes connection
    linkMat = np.ones((nodes.shape[0], nodes.shape[0]))
    
    
    # iteration over every node
    for i in range(nodes.shape[0]):
        # iteration over all the nodes with index > i
        for j in range(i + 1, nodes.shape[0]):
            
            # computing if the distance in Y between 2 nodes is bigger than distance in X
            rangeYbigger = abs(nodes[i, 1] - nodes[j, 1]) > abs(nodes[i, 0] - nodes[j, 0])
            
            if ((rangeYbigger and (nodes[i, 1] > nodes[j, 1])) or
                    (not (rangeYbigger) and (nodes[i, 0] > nodes[j, 0]))):
                node1, node2 = nodes[j, :], nodes[i, :]
            else:
                node1, node2 = nodes[i, :], nodes[j, :]

            if rangeYbigger:
                alpha = (node2[0] - node1[0]) / (node2[1] - node1[1])
                for y in range(node2[1] - node1[1]):
                    if (maskObsDilated[y + node1[1], round(alpha * y) + node1[0]] > 0):
                        linkMat[j, i] = 0
                        break

            elif not (rangeYbigger):
                alpha = (node2[1] - node1[1]) / (node2[0] - node1[0])
                for x in range(node2[0] - node1[0]):
                    if (maskObsDilated[round(alpha * x) + node1[1], x + node1[0]] > 0):
                        linkMat[j, i] = 0
                        break

    #print(linkMat)
    linkMat = np.nonzero(np.tril(linkMat, k=-1))
    nodeCon = np.array(linkMat).T
    nodes[:, 0] = fieldLengthP - nodes[:, 0]

    return (nodes * fieldWidthM / fieldWidthP), nodeCon, maskObsDilated


# Display terrain
def liveFeedback(videoFeed, nodesM, nodeCon, maskObsDilated, optPathM):
    nodes = nodesM * fieldWidthP / fieldWidthM
    optPath = optPathM * fieldWidthP / fieldWidthM
    nodes = np.rint(nodes).astype(np.int32)
    optPath = np.rint(optPath).astype(np.int32)

    frame = frameReader(videoFeed)
    output = frame

    output = cv2.bitwise_and(frame, frame, mask=(~maskObsDilated))
    output[np.where((output == [0, 0, 0]).all(axis=2))] = [100, 100, 100]

    nodes[:, 0] = fieldLengthP - nodes[:, 0]
    for idx, node in enumerate(nodes):
        output = cv2.circle(output, node, 5, (255, 0, 0), -1)
        font = cv2.FONT_HERSHEY_SIMPLEX
        output = cv2.putText(output, str(idx), node, font, 1, (255, 255, 255), 1, cv2.LINE_AA)

    nodes[:, 0] = fieldLengthP - nodes[:, 0]
    for con in nodeCon:
        output = cv2.line(output, [fieldLengthP - nodes[con[0]][0], nodes[con[0]][1]],
                          [fieldLengthP - nodes[con[1]][0], nodes[con[1]][1]], (255, 0, 0), 1)

    for i in range(len(optPath) - 1):
        output = cv2.line(output, [fieldLengthP - optPath[i][0], optPath[i][1]],
                          [fieldLengthP - optPath[i + 1][0], optPath[i + 1][1]], (255, 0, 0), 10)

    goal = nodes[-1, :]
    output = cv2.circle(output, [fieldLengthP - goal[0], goal[1]], 5, (0, 255, 255), -1)

    odo = odoFetch(videoFeed)
    pos = np.rint(odo[:2]).astype(np.int32)
    pos[0] = fieldLengthP - pos[0]
    angle = math.pi - odo[2]
    output = cv2.circle(output, pos, 5, (0, 0, 255), -1)

    endLX = pos[0] + np.rint(40 * math.cos(angle))
    endLY = pos[1] + np.rint(40 * math.sin(angle))
    endL = np.array([endLX, endLY]).astype(np.int32)
    output = cv2.line(output, pos, endL, (0, 255, 0), 3)

    return output


# Fetching odometry in meters
def fetchOdoMeters(videoFeed, n=5):
    i = 0
    while (i < n):
        try:
            odoPix = odoFetch(videoFeed)
            odoMet = odoPix[:2] * fieldWidthM / fieldWidthP
        except:
            pass
        else:
            return np.array([odoMet[0], odoMet[1]]), np.array([odoPix[2]])
        i += 1
    #clear_output(wait=True)
    #print("Can not find odoM")
    raise TypeError("Can not find odoM")


