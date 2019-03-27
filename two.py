#!/usr/bin/python3
ros_path = '/opt/ros/kinetic/lib/python2.7/dist-packages'
#cvpath = "/home/jamie/Libraries/opencv-4.0.1/build/lib"
import sys
import numpy as np
import cv2 as cv
import matplotlib.pyplot as plt
from imgProcessingFunctions import *
print(cv.__version__)


################### get test image
inputImg = cv.imread('Screenshot5.png')
#cam = cv2.VideoCapture(0)
rows = 400
cols = 800
height, width, depth = inputImg.shape
scale = cols/width
newX,newY = int(inputImg.shape[1]*scale), int(inputImg.shape[0]*scale)
rawImg = cv.resize(inputImg,(newX, newY), interpolation = cv.INTER_CUBIC)

centresImg = rawImg.copy()#np.zeros(rawImg.shape)
parametersImg = rawImg.copy()


####################### run functions
centres = getMarkerPositions(rawImg,centresImg)
print("centres: {}".format(centres))

robotPositions = getRobotPositions(centres)

List = getObjectPerimeters(parametersImg, 50, robotPositions)
List = np.asarray(List)
X_list = list(List[0:(int(List.size/2)-1)])
Y_list = list(List[(int(List.size/2)-1):-1])
#################################### image demo rendering code:

print("robotPositions")
print(robotPositions)
if robotPositions.size != 0:
    if robotPositions.shape == (3,):
        baseNo = 0
        details = robotPositions.shape
        cv.circle(centresImg, (int(robotPositions[0]),int(robotPositions[1])), 5, ( 0, 0, 255 ), 1, 8 )
        length = 30;
        x = int(np.cos(robotPositions[2])*length)+int(robotPositions[0])
        print("x: {}".format(x))
        y = int(np.sin(robotPositions[2])*length)+int(robotPositions[1])
        print("y: {}".format(y))
        cv.arrowedLine(centresImg, (int(robotPositions[0]),int(robotPositions[1])), (x, y), (0,255,0), 2)

    else:
        print(size(robotPositions.shape))
        bases,details = robotPositions.shape
        for baseNo in range(bases):
            cv.circle(centresImg, (int(robotPositions.item(baseNo,0)),int(robotPositions.item(baseNo,1))), 5, ( 0, 0, 255 ), 1, 8 )
            length = 30;
            x = int(np.cos(robotPositions.item(baseNo,2))*length)+int(robotPositions.item(baseNo,0))
            print("x: {}".format(x))
            y = int(np.sin(robotPositions.item(baseNo,2))*length)+int(robotPositions.item(baseNo,1))
            print("y: {}".format(y))
            cv.arrowedLine(centresImg, (int(robotPositions.item(baseNo,0)),int(robotPositions.item(baseNo,1))), (x, y), (0,255,0), 2)

#fig=plt.figure(figsize=(8, 8))


####################### Path Planning ###########################

sx = (robotPositions[0])/10  # start x position [m]
sy = (robotPositions[1])/10 # start y positon [m]
gx = 50/10  # goal x position [m]
gy = 250/10  # goal y position [m]
grid_size = 1  # potential grid size [m]
robot_radius = 5  # robot radius [m]

ox = X_list # obstacle x position list [m]
oy = Y_list  # obstacle y position list [m]

# if show_animation:
#     plt.grid(True)
#     plt.axis("equal")

# path generation
rx, ry = potential_field_planning(
    sx, sy, gx, gy, ox, oy, grid_size, robot_radius)

for i in range(len(rx)):
    cv.circle(centresImg, (int(rx[i]*10),int(ry[i]*10)), 5, ( 0, 0, 255 ), 1, 8 )
cv.imshow('centresImg',centresImg)
# if show_animation:
#     plt.show()



if cv.waitKey(0):# and 0xFF == ord('q'):
    cv.destroyAllWindows()
    exit()
