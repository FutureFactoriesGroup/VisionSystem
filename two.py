#!/usr/bin/python3
#ros_path = '/opt/ros/kinetic/lib/python2.7/dist-packages'
#cvpath = "/home/jamie/Libraries/opencv-4.0.1/build/lib"
import pkg_resources
import sys
DIR = "/usr/local/lib/python3.5/dist-packages"
sys.path.insert(0, DIR)
pkg_resources.require("opencv-python==4.0.0.21")
import sys
import numpy as np
import cv2 as cv
import time
import matplotlib.pyplot as plt
from imgProcessingFunctions import *
print(cv.__version__)


exposureVal = 105
saturationVal = 150
brightnessVal = 0
contrastVal = 300
gainVal = 150


################### get test image
#inputImg = cv.imread('WIN_20190327_14_06_40_Pro.jpg')
cam = cv.VideoCapture(0)
cv.namedWindow( "outputWindow", cv.WINDOW_AUTOSIZE );
def exposureCallback(val):
    exposureVal = cv.getTrackbarPos("exposureTrackbar", "outputWindow")
    cam.set(15,exposureVal)

def saturationCallback(val):
    saturationVal = cv.getTrackbarPos("saturationTrackbar", "outputWindow")
    cam.set(12,saturationVal)

def brightnessCallback(val):
    brightnessVal = cv.getTrackbarPos("brightnessTrackbar", "outputWindow")
    cam.set(10,brightnessVal)

def contrastCallback(val):
    contrastVal = cv.getTrackbarPos("contrastTrackbar", "outputWindow")
    cam.set(11,contrastVal)

def gainCallback(val):
    gainVal = cv.getTrackbarPos("gainTrackbar", "outputWindow")
    cam.set(14,gainVal)

cv.createTrackbar("exposureTrackbar",   "outputWindow", exposureVal, 300, exposureCallback)
cv.createTrackbar("saturationTrackbar", "outputWindow", saturationVal, 300, saturationCallback)
cv.createTrackbar("brightnessTrackbar", "outputWindow", brightnessVal, 300, brightnessCallback)
cv.createTrackbar("contrastTrackbar",   "outputWindow", contrastVal, 500, contrastCallback)
cv.createTrackbar("gainTrackbar",       "outputWindow", gainVal, 300, gainCallback)



cam.set(3, 1920) #Width
#time.sleep(1)
cam.set(4, 1080) #Height
cam.set(cv.CAP_PROP_AUTO_EXPOSURE,1)
#time.sleep(1)0
#cam.set(10, -100) #Brightness
#time.sleep(1)
#cam.set(14, 4) #Gain
#time.sleep(1)
#cam.set(11, 0)#Contrast
#time.sleep(1)
#cam.set(15, -50)#Exposure
#time.sleep(1)
#cam.set(12, 100)#saturation
#time.sleep(1)

#out = cv.VideoWriter('output.mp4',cv.VideoWriter_fourcc('X','2','6','4'),10.0,(800,400))
while(True):
    ret,frame = cam.read()
    inputImg = frame.copy()

    #rows = 450
    cols = 800
    height, width, depth = inputImg.shape
    scale = cols/width
    newX,newY = int(inputImg.shape[1]*scale), int(inputImg.shape[0]*scale)
    rawImg = cv.resize(inputImg,(newX, newY), interpolation = cv.INTER_CUBIC)
    cv.imshow('outputWindow',rawImg)

    centresImg = rawImg.copy()#np.zeros(rawImg.shape)
    parametersImg = rawImg.copy()


    ####################### run functions
    centres = getMarkerPositions(rawImg,centresImg)
    print("centres: {}".format(centres))

    robotPositions = getRobotPositions(centres)

    #X_list, Y_list = getObjectPerimeters(parametersImg, 15, robotPositions)
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
            #print(size(robotPositions.shape))
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


    # ####################### Path Planning ###########################
    #
    # sx = (robotPositions[0])/10  # start x position [m]
    # sy = (robotPositions[1])/10 # start y positon [m]
    # gx = 100/10  # goal x position [m]
    # gy = 350/10  # goal y position [m]
    # grid_size = 1  # potential grid size [m]
    # robot_radius = 5  # robot radius [m]
    #
    # ox = X_list # obstacle x position list [m]
    # oy = Y_list  # obstacle y position list [m]
    #
    # # if show_animation:
    # #     plt.grid(True)
    # #     plt.axis("equal")
    #
    # # path generation
    # rx, ry = potential_field_planning(
    #     sx, sy, gx, gy, ox, oy, grid_size, robot_radius)
    #
    # for i in range(len(rx)):
    #     cv.circle(centresImg, (int(rx[i]*10),int(ry[i]*10)), 5, ( 0, 0, 255 ), 1, 8 )
    cv.imshow('centresImg',centresImg)
    #out.write(cv.cvtColor(centresImg, cv.COLOR_HSV2BGR))
    # if show_animation:
    #     plt.show()



    if cv.waitKey(1) & 0xFF == ord('q'):
        cv.destroyAllWindows()
        break

cam.release()
#out.release()
