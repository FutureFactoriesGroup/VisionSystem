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

threadLock = threading.Lock()
threads = []
ShowImages = False
rawImg = []
DataToSend = ""
rospy.init_node('Vision', anonymous=True)
rospy.Subscriber("/transport", String, callback)
# spin() simply keeps python from exiting until this node is stopped
rospy.spin()

pub = rospy.Publisher('/transport', String, queue_size=10)
rospy.init_node('Vision', anonymous=True)
rospy.loginfo(DataToSend)

Tracking = TrackingThread()
Tracking.start()

if cv.waitKey(1) & 0xFF == ord('q'):
    cv.destroyAllWindows()
    cam.release()
    exit()

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    if (data.data.startswith('41'))
        TargetID = "A"
        if robotPositions.size != 0:
            try:
                Positions =  (robotPositions[0]/225),(robotPositions[1]/255)
            except IndexError as e:
                Positions =  (robotPositions[0,0]/225),(robotPositions[0,1]/255)
            PathPlanning = PathPlanningThread(rawImg,Positions,TargetID)
        else:
            callback(data)


class TrackingThread(threading.Thread):
    def __init__(self):

    def run(self):
        exposureVal = 105
        saturationVal = 150
        brightnessVal = 0
        contrastVal = 300
        gainVal = 150
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
                            cam.set(cv.CAP_PROP_AUTO_EXPOSURE,1)

                            while(True):
                                ret,frame = cam.read()
                                inputImg = frame.copy()
                                cols = 800
                                height, width, depth = inputImg.shape
                                scale = float(cols)/float(width)
                                newX,newY = int(inputImg.shape[1]*scale), int(inputImg.shape[0]*scale)
                                threadLock.acquire()
                                rawImg = cv.resize(inputImg,(newX, newY))#, interpolation = cv.INTER_CUBIC)
                                threadLock.release()
                                if ShowImages == True:
                                    cv.imshow('outputWindow',rawImg)
                                centresImg = rawImg
                                InitPositionData = '3'+'1'+'4'+'1'+'022'

                                ####################### run functions
                                centres = getMarkerPositions(rawImg,centresImg)
                                robotPositions = getRobotPositions(centres)

                                #################################### image demo rendering code:

                                if ShowImages == True:
                                    print("robotPositions")
                                    print(robotPositions)
                                    if robotPositions.size != 0:
                                        if robotPositions.shape == (3,):
                                            baseNo = 0
                                            details = robotPositions.shape
                                            cv.circle(centresImg, (int(robotPositions[0]),int(robotPositions[1])), 5, ( 0, 0, 255 ), 1, 8 )
                                            length = 30;
                                            x = int(np.cos(robotPositions[2])*length)+int(robotPositions[0])
                                            y = int(np.sin(robotPositions[2])*length)+int(robotPositions[1])
                                            cv.arrowedLine(centresImg, (int(robotPositions[0]),int(robotPositions[1])), (x, y), (0,255,0), 2)

                                        else:
                                            bases,details = robotPositions.shape
                                            for baseNo in range(bases):
                                                cv.circle(centresImg, (int(robotPositions.item(baseNo,0)),int(robotPositions.item(baseNo,1))), 5, ( 0, 0, 255 ), 1, 8 )
                                                length = 30;
                                                x = int(np.cos(robotPositions.item(baseNo,2))*length)+int(robotPositions.item(baseNo,0))
                                                y = int(np.sin(robotPositions.item(baseNo,2))*length)+int(robotPositions.item(baseNo,1))
                                                cv.arrowedLine(centresImg, (int(robotPositions.item(baseNo,0)),int(robotPositions.item(baseNo,1))), (x, y), (0,255,0), 2)

                                                m = hashlib.sha256()
                                                if robotPositions.size != 0:
                                                    try:
                                                        PositionData = '(1,' + str(robotPositions[0]/225) + ',' + str(robotPositions[1]/255) + ',' + str(robotPositions[2]) + ')'
                                                    except IndexError as e:
                                                        PositionData = '(1,' + str(robotPositions[0,0]/225) + ',' + str(robotPositions[0,1]/255) + ',' + str(robotPositions[0,2]) + ')'
                                                        #print("2 detections")
                                                        threadLock.acquire()
                                                        DataToSend = InitPositionData + str(len(PositionData)) + PositionData
                                                        m.update(PositionData.encode('utf-8'))
                                                        Checksum = m.hexdigest()
                                                        DataToSend = DataToSend + Checksum
                                                        pub.publish(DataToSend)
                                                        threadLock.release()
                                                        cv.imshow('centresImg',centresImg) if ShowImages == True


class PathPlanningThread(threading.Thread):
    def __init__(self,rawImg,robotPositions,TargetID):
        threading.Thread.__init__(self)
        threadLock.acquire()
        self.Image = rawImg
        self.RobotPositions = robotPositions
        self.TargetID = TargetID
        threadLock.release()
    def run(self):
        parametersImg = self.Image
        robotPositions = self.RobotPositions
        TargetID = self.TargetID

        InitPathData = '3'+'1'+'4'+'1'+'019'

        X_list, Y_list = getObjectPerimeters(parametersImg, 15, robotPositions, ShowImages)
        ####################### Path Planning ###########################

            sx = (robotPositions[0])/0.225  # start x position [m]
            sy = (robotPositions[1])/0.225 # start y positon [m]

            if TargetID == "A":
                gx = 100/0.225  # goal x position [m]
                gy = 350/0.225  # goal y position [m]
            else if TargetID == "B":
                gx = 100/0.225  # goal x position [m]
                gy = 350/0.225  # goal y position [m]
            else if TargetID == "C":
                gx = 100/0.225  # goal x position [m]
                gy = 350/0.225  # goal y position [m]
            else if TargetID == "D":
                gx = 100/0.225  # goal x position [m]
                gy = 350/0.225  # goal y position [m]

            grid_size = 1  # potential grid size [m]
            robot_radius = 0.3  # robot radius [m]

            ox = X_list # obstacle x position list [m]
            oy = Y_list  # obstacle y position list [m]

            # path generation
            rx, ry = potential_field_planning(
                sx, sy, gx, gy, ox, oy, grid_size, robot_radius)

            PathData = "(" + str(len(rx)) + ','
            for i in range(len(rx)):
                cv.circle(centresImg, (int(rx[i]*0.225),int(ry[i]*0.225)), 5, ( 0, 0, 255 ), 1, 8 ) if ShowImages == True
                PathData = PathData + ',(' + str(rx[i]) + ',' + str(ry[i]) + ')'
            PathData = PathData + ')'

            m = hashlib.sha256()
            threadLock.acquire()
            DataToSend = InitPathData + str(len(PathData)) + PathData
            m.update(PositionData.encode('utf-8'))
            Checksum = m.hexdigest()
            DataToSend = DataToSend + Checksum
            pub.publish(DataToSend)
            threadLock.release()
            cv.imshow('centresImg',centresImg) if ShowImages == True
