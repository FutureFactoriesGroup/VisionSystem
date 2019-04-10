#!/usr/bin/python3
#ros_path = '/opt/ros/kinetic/lib/python2.7/dist-packages'
#cvpath = "/home/jamie/Libraries/opencv-4.0.1/build/lib"
import pkg_resources
import sys
DIR = "/usr/local/lib/python3.5/dist-packages"
#DIR = "/usr/local/lib/python2.7/dist-packages"

sys.path.insert(0, DIR)
#pkg_resources.require("opencv-python==4.0.0.21")
import sys
import numpy as np
import cv2 as cv
import time
import matplotlib.pyplot as plt
from imgProcessingFunctions import *
print(cv.__version__)
import hashlib
import rospy
from std_msgs.msg import String
import threading

#threadLock = threading.Lock()
#threads = []
ShowImages = False
rawImg = None
robotPositions = []
PathPlanning = None
PathImg = None
Canny_Floor = None
drawing = None

class PathPlanningThread(threading.Thread):
    def __init__(self,rawImg,robotPositions,TargetID):
        #threadLock.acquire()
        self.Image = rawImg
        self.RobotPositions = robotPositions
        self.TargetID = TargetID
        #threadLock.release()
        threading.Thread.__init__(self)
    def run(self):
        global ShowImages
        global DataToSend
        global pub
        global robotPositions
        global PathImg
        parametersImg = self.Image
        centresImg = parametersImg.copy()
        PathImg = parametersImg.copy()
        #robotPositions = self.RobotPositions
        TargetID = self.TargetID

        InitPathData = '3'+'1'+'4'+'1'+'019'

        X_list, Y_list = getObjectPerimeters(parametersImg, 15, robotPositions, ShowImages)
        ####################### Path Planning ###########################

        sx = (robotPositions[0])/1000 # start x position [m]
        sy = (robotPositions[1])/1000# start y positon [m]

        if TargetID == "A":
            gx = 0.5#/0.225  # goal x position [m]
            gy = 1.5#/0.225  # goal y position [m]
        elif TargetID == "B":
            gx = 100/225  # goal x position [m]
            gy = 350/225  # goal y position [m]
        elif TargetID == "C":
            gx = 100/225  # goal x position [m]
            gy = 350/225  # goal y position [m]
        elif TargetID == "D":
            gx = 100/225  # goal x position [m]
            gy = 350/225  # goal y position [m]

        grid_size = 0.1  # potential grid size [m]
        robot_radius = 0.25  # robot radius [m]

        ox = X_list # obstacle x position list [m]
        oy = Y_list  # obstacle y position list [m]

        # path generation
        #rx, ry = potential_field_planning(sx, sy, gx, gy, ox, oy, grid_size, robot_radius)
        if show_animation:  # pragma: no cover
            plt.plot(ox, oy, ".k")
            plt.plot(sx, sy, "xr")
            plt.plot(gx, gy, "xb")
            plt.grid(True)
            plt.axis("equal")
        rx, ry = a_star_planning(sx, sy, gx, gy, ox, oy, grid_size, robot_radius)
        rx = list(reversed(rx))
        ry = list(reversed(ry))
        rx.append(gx)
        ry.append(gy)
        if show_animation:  # pragma: no cover
            plt.plot(rx, ry, "-r")
            plt.draw()
        PathData = "(" + str(len(rx))
        for i in range(len(rx)):
            #if ShowImages == True:
            cv.circle(PathImg, (int(rx[i]*225),int(ry[i]*225)), 5, ( 0, 0, 255 ), 1, 8 )
            #PathData = PathData + '(' + str(int(rx[i]*1000)) + ',' + str(int(ry[i]*1000)) + ')'
            PathData = PathData + ',' + str(int(rx[i]*1000)) + ',' + str(int(ry[i]*1000))
        PathData = PathData + ')'

        m = hashlib.sha256()
        #threadLock.acquire()
        DataToSend = InitPathData + str(len(PathData)) + PathData
        m.update(PositionData.encode('utf-8'))
        Checksum = m.hexdigest()
        DataToSend = DataToSend + Checksum
        pub.publish(DataToSend)
        #threadLock.release()
        #if ShowImages == True:


def callback(data):
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    if (data.data.startswith('41')):
        global robotPositions
        global PathPlanning
        TargetID = "A"
        try:
            Positions =  (robotPositions[0]),(robotPositions[1])
        except IndexError as e:
            print(robotPositions)
        PathPlanning = PathPlanningThread(rawImg,Positions,TargetID)
        PathPlanning.start()


exposureVal = 105
saturationVal = 150
brightnessVal = 0
contrastVal = 300
gainVal = 150

rospy.init_node('Vision', anonymous=True)
rospy.Subscriber("/transport", String, callback)

pub = rospy.Publisher('/transport', String, queue_size=10)
rospy.init_node('Vision', anonymous=True)
#rospy.loginfo(DataToSend)

################### get test image
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
cam.set(4, 1080) #Height
cam.set(cv.CAP_PROP_AUTO_EXPOSURE,1)# TEMP:
Start = 1
while(True):
    ret,frame = cam.read()
    inputImg = frame.copy()
    cols = 800
    height, width, depth = inputImg.shape
    scale = float(cols)/float(width)
    newX,newY = int(inputImg.shape[1]*scale), int(inputImg.shape[0]*scale)
    #threadLock.acquire()
    rawImg = cv.resize(inputImg,(newX, newY))#, interpolation = cv.INTER_CUBIC)
    #threadLock.release()
    #if ShowImages == True:
    cv.imshow('outputWindow',rawImg)
    centresImg = rawImg
    InitPositionData = '3'+'1'+'4'+'1'+'022'
    if(Start == 1):
        PathImg = rawImg.copy()
        Start = 0
    cv.imshow('Path',PathImg)
    ####################### run functions
    centres = getMarkerPositions(rawImg,centresImg)
    UnfilteredRobotPositions = getRobotPositions(centres)

    ################## Ros Publisher ###############
    m = hashlib.sha256()
    if UnfilteredRobotPositions.size != 0:
        try:
            PositionData = '(1,' + str(int(UnfilteredRobotPositions[0]/0.225)) + ',' + str(int(UnfilteredRobotPositions[1]/0.225)) + ',' + str(UnfilteredRobotPositions[2]) + ')'
            robotPositions = UnfilteredRobotPositions/0.225
        except:
            PositionData = '(1,' + str(int(UnfilteredRobotPositions[0,0]/0.225)) + ',' + str(int(UnfilteredRobotPositions[0,1]/0.225)) + ',' + str(UnfilteredRobotPositions[0,2]) + ')'
            #print("2 detections")
            robotPositions = UnfilteredRobotPositions[0,:]/0.225
        #threadLock.acquire()
        DataToSend = InitPositionData + str(len(PositionData)) + PositionData
        m.update(PositionData.encode('utf-8'))
        Checksum = m.hexdigest()
        DataToSend = DataToSend + Checksum
        #pub.publish(DataToSend)
        #threadLock.release()
        if ShowImages == True: cv.imshow('centresImg',centresImg)# TEMP:

    #################################### image demo rendering code:
    if ShowImages == True:
        #print("robotPositions")
        #print(robotPositions)
        if UnfilteredRobotPositions.size != 0:
            if UnfilteredRobotPositions.shape == (3,):
                baseNo = 0
                details = UnfilteredRobotPositions.shape
                cv.circle(centresImg, (int(UnfilteredRobotPositions[0]),int(UnfilteredRobotPositions[1])), 5, ( 0, 0, 255 ), 1, 8 )
                length = 30;
                x = int(np.cos(UnfilteredRobotPositions[2])*length)+int(UnfilteredRobotPositions[0])
                y = int(np.sin(UnfilteredRobotPositions[2])*length)+int(UnfilteredRobotPositions[1])
                cv.arrowedLine(centresImg, (int(UnfilteredRobotPositions[0]),int(UnfilteredRobotPositions[1])), (x, y), (0,255,0), 2)

            else:
                bases,details = UnfilteredRobotPositions.shape
                for baseNo in range(bases):
                    cv.circle(centresImg, (int(UnfilteredRobotPositions.item(baseNo,0)),int(UnfilteredRobotPositions.item(baseNo,1))), 5, ( 0, 0, 255 ), 1, 8 )
                    length = 30;
                    x = int(np.cos(UnfilteredRobotPositions.item(baseNo,2))*length)+int(UnfilteredRobotPositions.item(baseNo,0))
                    y = int(np.sin(UnfilteredRobotPositions.item(baseNo,2))*length)+int(UnfilteredRobotPositions.item(baseNo,1))
                    cv.arrowedLine(centresImg, (int(UnfilteredRobotPositions.item(baseNo,0)),int(UnfilteredRobotPositions.item(baseNo,1))), (x, y), (0,255,0), 2)

    if cv.waitKey(1) & 0xFF == ord('q'):
        cv.destroyAllWindows()
        cam.release()
        sys.exit()
