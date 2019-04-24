import numpy as np
import cv2 as cv
import random as rng
import math
from scipy import interpolate
import matplotlib.pyplot as plt

# Parameters
# KP = 100  # attractive potential gain
# ETA = 5 # repulsive potential gain
# AREA_WIDTH = 4.0  # potential area width [m]
show_animation = True
ShowImages = False

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
    if ShowImages == True: cv.imshow('canny_edImg',canny_edImg)

    try:
        contours, _ = cv.findContours(canny_edImg, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)
        # where contours is a vector of a vector of points in c++
        #break
    except ValueError:
        _, contours, _ = cv.findContours(canny_edImg, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)

    finalCentresImg = rawImg.copy()#np.zeros(rawImg.shape)
    areas = []
    momentsList = []
    areaLowerThreshold = 50
    areaUpperThreshold = 150
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
    #cv.imshow('finalCentresImg',finalCentresImg)

    #print("Found {} markers".format(len(centres)))
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
                    cEst0 = (0.5*ab+a) + 2.85*abOrth # get estimates for positions of third marker
                    cEst1 = (0.5*ab+a) - 2.85*abOrth
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
                        #print("ab2c: {}".format(ab2c))
                        conv = np.array([[1,0],[0,1j]])
                        angle = np.angle(np.sum(np.matmul(ab2c,conv)),deg=False)
                        if(angle<0):
                            angle =  2*math.pi - abs(angle)

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
    upper_red = np.array([180,255,30])
    filteredHsvImg = cv.inRange(hsvImg, lower_red, upper_red)
    filteredHsvImg = (255 - filteredHsvImg);
    kernel = np.ones((3,3),np.uint8)
    erodedImg = cv.erode(filteredHsvImg,kernel,iterations = 2)
    dilatedImg = cv.dilate(erodedImg,kernel,iterations = 3)
    dilatedImg = cv.blur(dilatedImg, (3,3)) # uncomment potentially?

    low_threshold = 0
    ratio = 3
    kernel_size = 5
    canny_edImg = cv.Canny(dilatedImg, low_threshold, low_threshold*ratio, kernel_size)
    #cv.imshow('Floor Mapping',canny_edImg)

    try:
        _, contours, _ = cv.findContours(canny_edImg, cv.RETR_TREE, cv.CHAIN_APPROX_NONE)
        #break
    except ValueError:
        contours, _ = cv.findContours(canny_edImg, cv.RETR_TREE, cv.CHAIN_APPROX_NONE)
    contourImg = np.zeros(rawImg.shape)
    if ShowImages == True: cv.drawContours(contourImg, contours, -1, (0,0,255), 3)
    X_list = []
    Y_list = []
    List = []
    cX_list = []
    cY_list = []
    drawing = np.zeros(rawImg.shape,dtype=np.uint8)
    try:
        NoOfRobots = int(((robotPositions.shape)[0])/3)
    except AttributeError as e:
        NoOfRobots = int(((len(robotPositions))/3))

    for i in range(len(contours)):
        CurrentContour = contours[i]
        M = cv.moments(CurrentContour)
        if M['m00'] != 0:
            cx = int(M['m10']/M['m00'])/225
            cy = int(M['m01']/M['m00'])/225
            cX_list.append(cx)
            cY_list.append(cy)
            Continue = True


            for k in range(NoOfRobots):
                X_robot = (robotPositions[k])/1000
                Y_robot = (robotPositions[k+1])/1000
                Distance = (((X_robot-cx)**2) + ((Y_robot-cy)**2))**0.5
                if(Distance > 0.5):
                    Continue = False
                    ContourSize = (CurrentContour.shape)[0]
                    if(ContourSize > 1): #and cv.contourArea(CurrentContour) > 50:
                        for j in range(ContourSize):
                            Xn = CurrentContour[j,0,0]
                            X_list.append(Xn/225)
                            Yn = CurrentContour[j,0,1]
                            Y_list.append(Yn/225)
                            if ShowImages == True: cv.circle(drawing,(Xn,Yn), 2, (0,255,0), 1)

    # X_list = np.asarray(X_list)
    # Y_list = np.asarray(Y_list)
    #if ShowImages == True: cv.imshow("Drawing",drawing)
    return X_list, Y_list


# def calc_potential_field(gx, gy, ox, oy, reso, rr):
#     minx = min(ox) - AREA_WIDTH / 2.0
#     miny = min(oy) - AREA_WIDTH / 2.0
#     maxx = max(ox) + AREA_WIDTH / 2.0
#     maxy = max(oy) + AREA_WIDTH / 2.0
#     xw = int(round((maxx - minx) / reso))
#     yw = int(round((maxy - miny) / reso))
#
#     # calc each potential
#     pmap = [[0.0 for i in range(yw)] for i in range(xw)]
#
#     for ix in range(xw):
#         x = ix * reso + minx
#
#         for iy in range(yw):
#             y = iy * reso + miny
#             ug = calc_attractive_potential(x, y, gx, gy)
#             uo = calc_repulsive_potential(x, y, ox, oy, rr)
#             uf = ug + uo
#             pmap[ix][iy] = uf
#
#     return pmap, minx, miny
#
#
# def calc_attractive_potential(x, y, gx, gy):
#     return 0.5 * KP * np.hypot(x - gx, y - gy)
#
#
# def calc_repulsive_potential(x, y, ox, oy, rr):
#     # search nearest obstacle
#     minid = -1
#     dmin = float("inf")
#     for i, _ in enumerate(ox):
#         d = np.hypot(x - ox[i], y - oy[i])
#         if dmin >= d:
#             dmin = d
#             minid = i
#
#     # calc repulsive potential
#     dq = np.hypot(x - ox[minid], y - oy[minid])
#
#     if dq <= rr:
#         if dq <= 0.1:
#             dq = 0.1
#
#         return 0.5 * ETA * (1.0 / dq - 1.0 / rr) ** 2
#     else:
#         return 0.0
#
#
# def get_motion_model():
#     # dx, dy
#     Step = 0.05
#     motion = [[Step, 0],
#               [0, Step],
#               [-Step, 0],
#               [0, -Step],
#               [-Step, -Step],
#               [-Step, Step],
#               [Step, -Step],
#               [Step, Step]]
#     return motion
#
#
# def potential_field_planning(sx, sy, gx, gy, ox, oy, reso, rr):
#
#     # calc potential field
#     pmap, minx, miny = calc_potential_field(gx, gy, ox, oy, reso, rr)
#
#     # search path
#     d = np.hypot(sx - gx, sy - gy)
#     ix = round((sx - minx) / reso)
#     iy = round((sy - miny) / reso)
#     gix = round((gx - minx) / reso)
#     giy = round((gy - miny) / reso)
#
#     if show_animation:
#         draw_heatmap(pmap)
#         plt.plot(ix, iy, "*k")
#         plt.plot(gix, giy, "*m")
#
#     rx, ry = [sx], [sy]
#     motion = get_motion_model()
#     while d >= reso:
#         minp = float("inf")
#         minix, miniy = -0.1, -0.1
#         for i, _ in enumerate(motion):
#             inx = int(ix + motion[i][0])
#             iny = int(iy + motion[i][1])
#             if inx >= len(pmap) or iny >= len(pmap[0]):
#                 p = float("inf")  # outside area
#             else:
#                 p = pmap[inx][iny]
#             if minp > p:
#                 minp = p
#                 minix = inx
#                 miniy = iny
#         ix = minix
#         iy = miniy
#         xp = ix * reso + minx
#         yp = iy * reso + miny
#         d = np.hypot(gx - xp, gy - yp)
#         rx.append(xp)
#         ry.append(yp)
#         if show_animation:
#             plt.plot(ix, iy, ".r")
#             plt.pause(0.01)
#
#     #print("Goal!!")
#     # for i in range(len(rx)):
#     #     print("{rx},{ry}".format(rx=rx[i],ry=ry[i]))
#     return rx, ry
#
#
# def draw_heatmap(data):
#     data = np.array(data).T
#     plt.pcolor(data, vmax=100.0, cmap=plt.cm.Blues)



class Node:

    def __init__(self, x, y, cost, pind):
        self.x = x
        self.y = y
        self.cost = cost
        self.pind = pind

    def __str__(self):
        return str(self.x) + "," + str(self.y) + "," + str(self.cost) + "," + str(self.pind)


def calc_final_path(ngoal, closedset, reso):
    # generate final course
    rx, ry = [ngoal.x * reso], [ngoal.y * reso]
    pind = ngoal.pind
    while pind != -1:
        n = closedset[pind]
        rx.append(n.x * reso)
        ry.append(n.y * reso)
        pind = n.pind

    return rx, ry


def a_star_planning(sx, sy, gx, gy, ox, oy, reso, rr):
    """
    gx: goal x position [m]
    gx: goal x position [m]
    ox: x position list of Obstacles [m]
    oy: y position list of Obstacles [m]
    reso: grid resolution [m]
    rr: robot radius[m]
    """

    nstart = Node(int(sx / reso), int(sy / reso), 0.0, -1)
    ngoal = Node(int(gx / reso), int(gy / reso), 0.0, -1)
    ox = [iox / reso for iox in ox]
    oy = [ioy / reso for ioy in oy]

    obmap, minx, miny, maxx, maxy, xw, yw = calc_obstacle_map(ox, oy, reso, rr)

    motion = get_motion_model()

    openset, closedset = dict(), dict()
    openset[calc_index(nstart, xw, minx, miny)] = nstart

    while 1:
        c_id = min(
            openset, key=lambda o: openset[o].cost + calc_heuristic(ngoal, openset[o]))
        current = openset[c_id]

        # show graph
        if show_animation:  # pragma: no cover
            plt.plot(current.x * reso, current.y * reso, "xc")
            if len(closedset.keys()) % 10 == 0:
                plt.pause(0.001)

        if current.x == ngoal.x and current.y == ngoal.y:
            #print("Find goal")
            ngoal.pind = current.pind
            ngoal.cost = current.cost
            break

        # Remove the item from the open set
        del openset[c_id]
        # Add it to the closed set
        closedset[c_id] = current

        # expand search grid based on motion model
        for i, _ in enumerate(motion):
            node = Node(current.x + motion[i][0],
                        current.y + motion[i][1],
                        current.cost + motion[i][2], c_id)
            n_id = calc_index(node, xw, minx, miny)

            if n_id in closedset:
                continue

            if not verify_node(node, obmap, minx, miny, maxx, maxy):
                continue

            if n_id not in openset:
                openset[n_id] = node  # Discover a new node
            else:
                if openset[n_id].cost >= node.cost:
                    # This path is the best until now. record it!
                    openset[n_id] = node

    rx, ry = calc_final_path(ngoal, closedset, reso)

    return rx, ry


def calc_heuristic(n1, n2):
    w = 1  # weight of heuristic
    d = w * math.sqrt((n1.x - n2.x)**2 + (n1.y - n2.y)**2)
    return d


def verify_node(node, obmap, minx, miny, maxx, maxy):

    if node.x < minx:
        return False
    elif node.y < miny:
        return False
    elif node.x >= maxx:
        return False
    elif node.y >= maxy:
        return False

    if obmap[node.x][node.y]:
        return False

    return True


def calc_obstacle_map(ox, oy, reso, vr):

    minx = int(min(ox))
    miny = int(min(oy))
    maxx = int(max(ox))
    maxy = int(max(oy))
    #  print("minx:", minx)
    #  print("miny:", miny)
    #  print("maxx:", maxx)
    #  print("maxy:", maxy)

    xwidth = int(round(maxx - minx))
    ywidth = int(round(maxy - miny))
    #  print("xwidth:", xwidth)
    #  print("ywidth:", ywidth)

    # obstacle map generation
    obmap = [[False for i in range(ywidth)] for i in range(xwidth)]
    for ix in range(xwidth):
        x = ix + minx
        for iy in range(ywidth):
            y = iy + miny
            #  print(x, y)
            for iox, ioy in zip(ox, oy):
                d = math.sqrt((iox - x)**2 + (ioy - y)**2)
                if d <= vr / reso:
                    obmap[ix][iy] = True
                    break

    return obmap, minx, miny, maxx, maxy, xwidth, ywidth


def calc_index(node, xwidth, xmin, ymin):
    return (node.y - ymin) * xwidth + (node.x - xmin)


def get_motion_model():
    # dx, dy, cost
    Step = 1
    motion = [[Step, 0, Step],
              [0, Step, Step],
              [-Step, 0, Step],
              [0, -Step, Step],
              [-Step, -Step, math.sqrt(2*(Step**2))],
              [-Step, Step, math.sqrt(2*(Step**2))],
              [Step, -Step, math.sqrt(2*(Step**2))],
              [Step, Step, math.sqrt(2*(Step**2))]]

    return motion
