import numpy as np
import cv2 as cv
import random as rng
import math
from scipy import interpolate
import matplotlib.pyplot as plt

# Parameters
KP = 1  # attractive potential gain
ETA = 75.0  # repulsive potential gain
AREA_WIDTH = 50.0  # potential area width [m]

show_animation = True

def getMarkerPositions(rawImg,centresImg):
    if rawImg.any() == None:
        print("No image Found\n")
        exit(1)

    hsvImg = cv.cvtColor(rawImg, cv.COLOR_BGR2HSV)

    lower_red1 = np.array([160,100,100])
    upper_red1 = np.array([179,255,255])

    lower_red2 = np.array([0,100,100])
    upper_red2 = np.array([10,255,255])
    filteredHsvImg1 = cv.inRange(hsvImg, lower_red1, upper_red1)
    filteredHsvImg2 = cv.inRange(hsvImg, lower_red2, upper_red2)
    filteredHsvImg = cv.bitwise_or(filteredHsvImg1,filteredHsvImg2)
    #cv.imshow('filteredHsvImg',filteredHsvImg2)

    kernel = np.ones((3,3),np.uint8)
    dilatedImg = cv.dilate(filteredHsvImg,kernel,iterations = 2)
    # this could be repaced using "Opening" operation
    #cv.imshow('erosion/dilation',dilatedImg)

    dilatedImg = cv.blur(dilatedImg, (3,3)) # uncomment potentially?
    low_threshold = 0
    ratio = 3
    kernel_size = 3
    erodedImg = cv.erode(dilatedImg,kernel,iterations =1)
    canny_edImg = cv.Canny(erodedImg, low_threshold, low_threshold*ratio, kernel_size)
    cv.imshow('canny_edImg',canny_edImg)

    try:
        contours, _ = cv.findContours(canny_edImg, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)
        # where contours is a vector of a vector of points in c++
        #break
    except ValueError:
        _, contours, _ = cv.findContours(canny_edImg, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)

    finalCentresImg = rawImg.copy()#np.zeros(rawImg.shape)
    areas = []
    momentsList = []
    areaLowerThreshold = 85
    areaUpperThreshold = 130
    #contCounter = 0
    for cont in contours:
        if (cv.contourArea(cont) > areaLowerThreshold and cv.contourArea(cont) < areaUpperThreshold):
            #print("good contour {} has area {}".format(contCounter, cv.contourArea(cont)))
            #contCounter = contCounter + 1
            areas.append(cv.contourArea(cont))
            momentsList.append(cv.moments(cont))
            cv.drawContours(finalCentresImg,cont, -1, (255, 255, 0), 1)
        else:
            pass
            #print("bad contour {} has area {}".format(contCounter, cv.contourArea(cont)))
            #contCounter = contCounter + 1

    centres = []
    distThreshold = 20

    #centresImg = rawImg.copy()
    for M in momentsList:
        tempCent = np.array([int(M['m10']/M['m00']),int(M['m01']/M['m00'])])
        toBeAdded = True
        #print("temp cent: {}".format(tempCent))
        if(len(centres) > 0):
            tempCentres = centres
            for c in tempCentres:
                if np.linalg.norm(tempCent - c) < distThreshold: # some centres are close due to duplication in moments calc, this removes them
                    toBeAdded = False
                    #pass

            if toBeAdded == True:
                centres.append(tempCent)
                cv.circle(finalCentresImg,tuple(tempCent), 1, (0,255,0), 1)
                #print("adding circle")

        else:
            centres.append(tempCent)
            cv.circle(finalCentresImg,tuple(tempCent), 1, (0,255,0), 1)
            #print("adding circle")
    cv.imshow('finalCentresImg',finalCentresImg)

    print("Found {} markers".format(len(centres)))
    return centres

def getRobotPositions(centres):
    robotPositions = np.array([]) # create numpy array to store robot positions
    maxabNorm = 35
    minabNorm = 20
    cDevThreshold = 20

    centresAllocated = np.zeros((1,len(centres)))
    #print(centres)
    #print(centresAllocated)
    centresAllocated = centresAllocated[0]
    #print(centresAllocated)
    for aIndex in range(len(centres)):
        if centresAllocated[aIndex] == 0: # ie if we havent allocated it yet
            a = centres[aIndex]
            for bIndex in range(len(centres)):
                if (bIndex != aIndex) and (centresAllocated[aIndex] == 0): # ie if we havent allocated it yet and its not 'a' candidate
                    b = centres[bIndex]
                    ab = b-a;
                    abNorm = np.linalg.norm(b-a)
                    if abNorm > maxabNorm or abNorm < minabNorm: # ie if its larger or smaller than expected
                        continue # ie begin loop on next b candidate
                    abOrth = np.array([-ab[1],ab[0]])
                    cEst0 = (0.5*ab+a) + 2*abOrth # get estimates for positions of third marker
                    cEst1 = (0.5*ab+a) - 2*abOrth
                    bestMatch = 0
                    bestMatchIndex = 0
                    smallestDist = 10000
                    for cIndex in range(len(centres)):
                        if cIndex != aIndex and bIndex != aIndex and centresAllocated[aIndex] == 0: # ie if we havent allocated it yet and its not 'a' or 'b' candidate
                            c = centres[cIndex]
                            if np.linalg.norm(c-cEst0) < smallestDist and  np.linalg.norm(c-cEst0) < cDevThreshold: # find the point closest to each estimate
                                smallestDist = np.linalg.norm(c-cEst0)
                                bestMatch = c
                                bestMatchIndex = cIndex

                            if np.linalg.norm(c-cEst1) < smallestDist and  np.linalg.norm(c-cEst1) < cDevThreshold:
                                smallestDist = np.linalg.norm(c-cEst1)
                                bestMatch = c
                                bestMatchIndex = cIndex

                    if smallestDist < 10000: # make sure one point has actually been selected
                        ab_c = np.array([[(0.5*ab+a)],[bestMatch]])

                        centroid = np.mean(ab_c,axis=0)[0]
                        ab2c = bestMatch-(0.5*ab+a)
                        print("ab2c: {}".format(ab2c))
                        conv = np.array([[1,0],[0,1j]])
                        angle = np.angle(np.sum(np.matmul(ab2c,conv)),deg=False)
                        if robotPositions.shape == (0,):
                            robotPositions = np.array([centroid.item(0),centroid.item(1),angle])
                        else:
                            currentEntry = np.array([centroid.item(0),centroid.item(1),angle])
                            if robotPositions.shape == (3,):
                                temp_robotPositions = np.zeros((2,3))
                                temp_robotPositions[:-1,:] = robotPositions
                                temp_robotPositions[-1:,:] = currentEntry
                                robotPositions = temp_robotPositions
                            else:
                                rows,cols = robotPositions.shape
                                temp_robotPositions = np.zeros((rows+1,cols))
                                temp_robotPositions[:-1,:] = robotPositions
                                temp_robotPositions[-1:,:] = currentEntry
                                robotPositions = temp_robotPositions

                        centresAllocated[aIndex] = 1
                        centresAllocated[bIndex] = 1
                        centresAllocated[bestMatchIndex] = 1
    return robotPositions

def getObjectPerimeters(rawImg, pathPointResolution, robotPositions, ShowImages):
    hsvImg = cv.cvtColor(rawImg, cv.COLOR_BGR2HSV)
    lower_red = np.array([0,0,0])
    upper_red = np.array([180,255,60])
    filteredHsvImg = cv.inRange(hsvImg, lower_red, upper_red)
    filteredHsvImg = (255 - filteredHsvImg);
    kernel = np.ones((3,3),np.uint8)
    erodedImg = cv.erode(filteredHsvImg,kernel,iterations = 1)
    dilatedImg = cv.dilate(erodedImg,kernel,iterations = 3)
    dilatedImg = cv.blur(dilatedImg, (3,3)) # uncomment potentially?
    low_threshold = 0
    ratio = 3
    kernel_size = 5
    canny_edImg = cv.Canny(dilatedImg, low_threshold, low_threshold*ratio, kernel_size)

    try:
        _, contours, _ = cv.findContours(canny_edImg, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)
        #break
    except ValueError:
        contours, _ = cv.findContours(canny_edImg, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)
    contourImg = np.zeros(rawImg.shape)
    cv.drawContours(contourImg, contours, -1, (0,0,255), 3) if ShowImages == True
    X_list = []
    Y_list = []
    List = []
    cX_list = []
    cY_list = []
    drawing = np.zeros(rawImg.shape,dtype=np.uint8)
    NoOfRobots = int(((robotPositions.shape)[0])/3)
    for i in range(len(contours)):
        CurrentContour = contours[i]
        M = cv.moments(CurrentContour)
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
        cX_list.append(cx/0.225)
        cY_list.append(cy/0.225)
        Continue = True


        for k in range(NoOfRobots):
            X_robot = robotPositions[3*k]
            Y_robot = robotPositions[(3*k)+1]
            Distance = ((X_robot-cx)**2 + (Y_robot-cy)**2)**0.5
            if(Distance < 15):
                i = i+1
                if(i < len(contours)):
                    CurrentContour = contours[i]
                else:
                    Continue = False
        if(Continue):
            ContourSize = (CurrentContour.shape)[0]
            if(ContourSize > 1):
                X1 = CurrentContour[0,0,0]
                X2 = CurrentContour[1,0,0]
                Y1 = CurrentContour[0,0,1]
                Y2 = CurrentContour[1,0,1]

                ContourResolution = ((X2-X1)**2 + (Y2-Y1)**2)**0.5
                SamplingInterval = int(pathPointResolution/ContourResolution)
                NoOfPts = int(ContourSize/SamplingInterval)
                #X_list.append(X1)
                #Y_list.append(Y1)
                for j in range(NoOfPts):
                    Xn = CurrentContour[j*SamplingInterval,0,0]
                    X_list.append(Xn/0.225)
                    Yn = CurrentContour[j*SamplingInterval,0,1]
                    Y_list.append(Yn/0.225)
                    cv.circle(drawing,(Xn,Yn), 2, (0,255,0), 1) if ShowImages == True

                Last_Index = (ContourSize - math.ceil(SamplingInterval/2))-1
                if(Last_Index > 0 and Last_Index < ContourSize):
                    X_last = CurrentContour[Last_Index,0,0]
                    X_list.append(X_last/0.225)
                    Y_last = CurrentContour[Last_Index,0,1]
                    Y_list.append(Y_last/0.225)
                    cv.circle(drawing,(X_last,Y_last), 2, (0,255,0), 1) if ShowImages == True
    # X_list = np.asarray(X_list)
    # Y_list = np.asarray(Y_list)
    cv.imshow("Drawing",drawing) if ShowImages == True
    return X_list, Y_list


def calc_potential_field(gx, gy, ox, oy, reso, rr):
    minx = min(ox) - AREA_WIDTH / 2.0
    miny = min(oy) - AREA_WIDTH / 2.0
    maxx = max(ox) + AREA_WIDTH / 2.0
    maxy = max(oy) + AREA_WIDTH / 2.0
    xw = int(round((maxxprint("nre x {}".format(newX)) - minx) / reso))
    yw = int(round((maxy - miny) / reso))

    # calc each potential
    pmap = [[0.0 for i in range(yw)] for i in range(xw)]

    for ix in range(xw):
        x = ix * reso + minx

        for iy in range(yw):
            y = iy * reso + miny
            ug = calc_attractive_potential(x, y, gx, gy)
            uo = calc_repulsive_potential(x, y, ox, oy, rr)
            uf = ug + uo
            pmap[ix][iy] = uf

    return pmap, minx, miny


def calc_attractive_potential(x, y, gx, gy):
    return 0.5 * KP * np.hypot(x - gx, y - gy)


def calc_repulsive_potential(x, y, ox, oy, rr):
    # search nearest obstacle
    minid = -1
    dmin = float("inf")
    for i, _ in enumerate(ox):
        d = np.hypot(x - ox[i], y - oy[i])
        if dmin >= d:
            dmin = d
            minid = i

    # calc repulsive potential
    dq = np.hypot(x - ox[minid], y - oy[minid])

    if dq <= rr:
        if dq <= 0.1:
            dq = 0.1

        return 0.5 * ETA * (1.0 / dq - 1.0 / rr) ** 2
    else:
        return 0.0


def get_motion_model():
    # dx, dy
    Step = 100
    motion = [[Step, 0],
              [0, Step],
              [-Step, 0],
              [0, -Step],
              [-Step, -Step],
              [-Step, Step],
              [Step, -Step],
              [Step, Step]]
    return motion


def potential_field_planning(sx, sy, gx, gy, ox, oy, reso, rr):

    # calc potential field
    pmap, minx, miny = calc_potential_field(gx, gy, ox, oy, reso, rr)

    # search path
    d = np.hypot(sx - gx, sy - gy)
    ix = round((sx - minx) / reso)
    iy = round((sy - miny) / reso)
    gix = round((gx - minx) / reso)
    giy = round((gy - miny) / reso)

    if show_animation:
        draw_heatmap(pmap)
        plt.plot(ix, iy, "*k")
        plt.plot(gix, giy, "*m")

    rx, ry = [sx], [sy]
    motion = get_motion_model()
    while d >= reso:
        minp = float("inf")
        minix, miniy = -1, -1
        for i, _ in enumerate(motion):
            inx = int(ix + motion[i][0])
            iny = int(iy + motion[i][1])
            if inx >= len(pmap) or iny >= len(pmap[0]):
                p = float("inf")  # outside area
            else:
                p = pmap[inx][iny]
            if minp > p:
                minp = p
                minix = inx
                miniy = iny
        ix = minix
        iy = miniy
        xp = ix * reso + minx
        yp = iy * reso + miny
        d = np.hypot(gx - xp, gy - yp)
        rx.append(xp)
        ry.append(yp)

        if show_animation:
            plt.plot(ix, iy, ".r")
            plt.pause(0.01)

    #print("Goal!!")
    # for i in range(len(rx)):
    #     print("{rx},{ry}".format(rx=rx[i],ry=ry[i]))
    return rx, ry


def draw_heatmap(data):
    data = np.array(data).T
    plt.pcolor(data, vmax=100.0, cmap=plt.cm.Blues)
