#!/usr/bin/python3

import sys
DIR = "/usr/local/lib/python3.5/dist-packages"
sys.path.insert(0, DIR)

import cv2 as cv


#exposure
#saturation
#brightness
# contrast
#gain

#cv.createTrackbar(trackbarName, windowName, value, count, onChange)
exposureVal = 150
saturationVal = 150
brightnessVal = 150
contrastVal = 150
gainVal = 150
cap = cv.VideoCapture(0)

cv.namedWindow( "outputWindow", cv.WINDOW_AUTOSIZE );

def exposureCallback(val):
    exposureVal = cv.getTrackbarPos("exposureTrackbar", "outputWindow")
    cap.set(15,exposureVal)

def saturationCallback(val):
    saturationVal = cv.getTrackbarPos("saturationTrackbar", "outputWindow")
    cap.set(12,saturationVal)

def brightnessCallback(val):
    brightnessVal = cv.getTrackbarPos("brightnessTrackbar", "outputWindow")
    cap.set(10,brightnessVal)

def contrastCallback(val):
    contrastVal = cv.getTrackbarPos("contrastTrackbar", "outputWindow")
    cap.set(11,contrastVal)

def gainCallback(val):
    gainVal = cv.getTrackbarPos("gainTrackbar", "outputWindow")
    cap.set(14,gainVal)

cv.createTrackbar("exposureTrackbar",   "outputWindow", 150, 300, exposureCallback)
cv.createTrackbar("saturationTrackbar", "outputWindow", 150, 300, saturationCallback)
cv.createTrackbar("brightnessTrackbar", "outputWindow", 150, 300, brightnessCallback)
cv.createTrackbar("contrastTrackbar",   "outputWindow", 150, 300, contrastCallback)
cv.createTrackbar("gainTrackbar",       "outputWindow", 150, 300, gainCallback)



while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()
    cv.imshow('outputWindow',frame)

    if cv.waitKey(1) & 0xFF == ord('q'):
        break
