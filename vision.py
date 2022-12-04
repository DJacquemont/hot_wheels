import cv2
import numpy as np
import math


## PARAMETERS ##

goal = [10, 10]
fieldWidthM = 0.841              # width of A0 sheet [m]
fieldLengthM = 1.189             # length of A0 sheet [m]
fieldWidthP = 480                # width of FoV camera [pixels]
fieldLengthP = 640                # width of FoV camera [pixels]

 # POSITION - GREEN
TLowPos = np.array([245,245,175])
THighPos = np.array([255,255,255])

 # DIRECTION - RED
TLowDir = np.array([150,130,200])
THighDir = np.array([200,200,255])

 # OBSTACLE - BLACK
TLowObs = np.array([0, 0, 0])
THighObs = np.array([40, 40, 40])


SE = np.array([[0,0,1,0,0],
               [0,1,1,1,0],
               [1,1,1,1,1],
               [0,1,1,1,0],
               [0,0,1,0,0]], 'uint8')

## FUNCTIONS ##

# Frame Reading
def frameReader(videoFeed):
    try:
        ret, frame = videoFeed.read()
        frame = cv2.medianBlur(frame,5)
        return frame.astype('uint8')
    except:
        print("video can't be read")
        return False



# Fetching odometry
def odoFetch(videoFeed):
    frame = frameReader(videoFeed)
    if type(frame) == bool:
        return False

    posIsFetch = False
    angleIsFetch = False

    # Fetching Position
    maskPos = cv2.inRange(frame, TLowPos, THighPos)
    totalLabelsPos, _, statsPos, BlobPos = cv2.connectedComponentsWithStats(maskPos, connectivity=8)

    #print(statsPos[:, cv2.CC_STAT_AREA] )
    BlobPosCenter = []
    for i in range(1, totalLabelsPos):
        areaBlobPos = statsPos[i, cv2.CC_STAT_AREA] 
        if (areaBlobPos > 130) and (areaBlobPos < 400): # Size of the desired blobs
            BlobPosCenter.append(BlobPos[i])

    if np.shape(BlobPosCenter)[0] == 2: # Checking if the position was found
        robotPos = [BlobPosCenter[0][0]+(BlobPosCenter[1][0]-BlobPosCenter[0][0])/2, BlobPosCenter[0][1]+(BlobPosCenter[1][1]-BlobPosCenter[0][1])/2]
        robotPos[0] = fieldLengthP - robotPos[0]
        robotPos = np.rint(robotPos).astype(np.int32)
        posIsFetch = True

    if posIsFetch:
        # Fetching Direction
        maskDir = cv2.inRange(frame, TLowDir, THighDir)
        totalLabelsDir, _, statsDir, BlobDir = cv2.connectedComponentsWithStats(maskDir, connectivity=8)

        #print(statsDir[:, cv2.CC_STAT_AREA])
        BlobDirCenter = []
        for i in range(1, totalLabelsDir):
            areaBlobDir = statsDir[i, cv2.CC_STAT_AREA] 
            if (areaBlobDir > 25) and (areaBlobDir < 150): # Size of the desired blobs
                BlobDirCenter.append(BlobDir[i])

        if np.shape(BlobDirCenter)[0] == 1: # Checking if the angle was found
            BlobDirCenter = [j for sub in BlobDirCenter for j in sub]
            BlobDirCenter[0] = fieldLengthP - BlobDirCenter[0]
            angle = math.atan2(BlobDirCenter[1]-robotPos[1], BlobDirCenter[0]-robotPos[0])
            angleIsFetch = True

    # Combining position & angle
    if (angleIsFetch and posIsFetch) == True:
        odometry = np.append(robotPos, angle)
        print("Odometry obtained")
        return odometry
    else:
        #print("Can not obtain odometry")
        return False




# Fetching terrain
def terrainFetch(videoFeed):
    frame = frameReader(videoFeed)
    if type(frame) == bool:
        return False

    maskObs = cv2.inRange(frame, TLowObs, THighObs)
    totalLabelsObs, labelIdObs, statsObs, BlobObs = cv2.connectedComponentsWithStats(maskObs, connectivity=8)
    maskObs = np.zeros(maskObs.shape, dtype="uint8")

    for i in range(1, totalLabelsObs): # Getting rid of the nois on our mask
        areaBlobObs = statsObs[i, cv2.CC_STAT_AREA] 
    
        if (areaBlobObs > 3000) and (areaBlobObs < 10000):
            componentMask = (labelIdObs == i).astype("uint8") * 255
            maskObs = cv2.bitwise_or(maskObs, componentMask) #

    maskObsDilated = cv2.dilate(maskObs, SE, iterations = 10)
    maskObsMargin = cv2.dilate(maskObsDilated, SE, iterations = 5)

    dst = cv2.cornerHarris(maskObsMargin,20,3,0.04) # Fetching the angles
    blobCorner = np.zeros(frame.shape[:2], 'uint8')
    blobCorner[dst>0.04*dst.max()]=1
    
    _, _, _, nodes = cv2.connectedComponentsWithStats(blobCorner, connectivity=8)

    odo = odoFetch(videoFeed)
    if type(odo) == bool:
        return False
    else:
        nodes = np.append(nodes, [odo[:2]], axis = 0)
        nodes = np.append(nodes, [goal], axis = 0)
        nodes = np.rint(nodes[1:]).astype(np.int32)
        linkMat = np.ones((nodes.shape[0], nodes.shape[0]))

        for i in range(nodes.shape[0]):
            for j in range(i+1, nodes.shape[0]):
                rangeYbigger = abs(nodes[i,1]-nodes[j,1]) > abs(nodes[i,0]-nodes[j,0])

                if ((rangeYbigger and (nodes[i,1] > nodes[j,1])) or (not(rangeYbigger) and (nodes[i,0] > nodes[j,0]))):
                    node1, node2 =  nodes[j,:], nodes[i,:]
                else:
                    node1, node2 =  nodes[i,:], nodes[j,:]
                    
                if rangeYbigger:
                    alpha = (node2[0]-node1[0])/(node2[1]-node1[1])
                    for y in range(node2[1]-node1[1]):
                        if (maskObsDilated[y+node1[1], round(alpha*y)+node1[0]] > 0):
                            linkMat[j, i] = 0
                            break
                            
                elif not(rangeYbigger):
                    alpha = (node2[1]-node1[1])/(node2[0]-node1[0])
                    for x in range(node2[0]-node1[0]):
                        if (maskObsDilated[round(alpha*x)+node1[1], x+node1[0]] > 0):
                            linkMat[j, i] = 0
                            break

        linkMat = np.nonzero(np.tril(linkMat, k=-1))
        nodeCon = np.array(linkMat).T

        return nodes, nodeCon, maskObsDilated



# Disply terrain
def liveFeedback(videoFeed, nodes, nodeCon, maskObsDilated):
    frame = frameReader(videoFeed)
    if type(frame) == bool:
        return False

    output = cv2.bitwise_and(frame, frame, mask=(~maskObsDilated))
    output[np.where((output == [0, 0, 0]).all(axis=2))] = [100, 100, 100]

    for idx, node in enumerate(nodes):
        output = cv2.circle(output, node, 5, (255, 0, 0), -1)
        font = cv2.FONT_HERSHEY_SIMPLEX
        output = cv2.putText(output, str(idx), node, font, 1,(255,255,255),1,cv2.LINE_AA)

    for con in nodeCon:
        output = cv2.line(output, nodes[con[0]], nodes[con[1]], (255,0,00), 1)
    
    output = cv2.circle(output, goal, 5, (0, 255, 255), -1)

    odo = odoFetch(videoFeed)
    if type(odo) == bool:
        pass
    else:
        pos = np.rint(odo[:2]).astype(np.int32)
        angle = odo[2]
        output = cv2.circle(output, pos, 5, (0,0,255), -1)

        endLX = pos[0] + np.rint(40*math.cos(angle))
        endLY = pos[1] + np.rint(40*math.sin(angle))
        endL = np.array([endLX, endLY]).astype(np.int32)
        output = cv2.line(output, pos, endL, (0,255,0), 3)

    return output


# Fetching odometry in meters
def fetchOdoMeters(videoFeed):
    try:
        odoPix = odoFetch(videoFeed)
        odoMet = odoPix[:2]*fieldWidthM/fieldWidthP
        return np.array([odoMet[0], odoMet[1], odoPix[2]])
    except:
        return False