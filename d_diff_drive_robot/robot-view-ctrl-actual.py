from __future__ import division
import ipc
import ach
import sys
import time
from ctypes import *
import socket
import cv2.cv as cv
import cv2
import numpy as np
import diff_drive
import numpy as np
import math

ref = ipc.E_SERVO()

PYoutput = ipc.E_PY()

CHAN_1 = ach.Channel(ipc.EXAMPLE_CHAN_1)
CHAN_1.flush()

capture = cv2.VideoCapture(0)

newx = 320
newy = 240
scale=2
nx = int(newx/scale)
ny = int(newy/scale)


r = ach.Channel(ipc.EXAMPLE_CHAN_1)
r.flush()

X_output=0
Y_output=0
i=0
tLast = 0
eLastX = 0
eLastY = 0
cLastX = 0
cLastY = 0
counter=0

depth=0.0048/scale
print "depth : ",depth
multiplier = (4/1000000)/(depth)
print "multiplier is : ",multiplier
while True:
    # Get Frame
    ret,img = capture.read()
    #print img
#    c_image = img.copy()
    img = cv2.resize(img,(nx,ny))
    #img = cv2.cvtColor(img,cv2.COLOR_BGR2RGB)
    cv2.imshow("wctrl2", img)
    cv2.waitKey(10)
    numRows = img.shape[0]
    numCols = img.shape[1]
    lower = np.array([130, 0, 0], np.uint8)
    upper = np.array([255, 180, 150], np.uint8)
    # use cv funtion to find the blue mask
    mask = cv2.inRange(img, lower, upper)
    cv2.imshow("wctrl", mask)
    cv2.waitKey(10)



    tx = 0
    ty = 0
    a = 0
    for x in range(numCols):
        for y in range(numRows):
            if mask[y, x] > 0:
                tx += x
                ty += y
                a = a + 1

   # compute final CG based on found points
    x = -1
    y = -1
    found = False
    # require a > 50 to avoid small blue noise
    if a > 30:
        x = tx / a - nx/2
        y = -(ty / a - ny/2)
        found = True
#        print "Found: ", found, "at", x, ",", y
#	print "a equals : ", a
    else: 
        #print "Not found"
	counter = counter +1
	#print counter
	print "Hit CTRL+C to end the process!"
	print "THEN print ./robot-view kill  to kill all the processes>>>>"
	print "."
	print "."
	print "."
	print "."
	print "."
	print "."

    tNow = time.time()
    if found:
        # compute offset from center (-1 to 1)
        #x = float((x )) / (numCols/2)
        #y = float(-(y)) / (numRows/2)
        xDes = 0
        yDes = 0
        kp = 6
        kd = 0.50
        eX = x 
        eY = y
#	print "ex, ey : ", eX, eY
        cx = (kp * eX) + (kd * ((eX - eLastX) / (tNow - tLast)))
        cy = (kp * eY) + (kd * ((eY - eLastY) / (tNow - tLast)))
        cLastX = cx
        cLastY = cy
        eLastX = eX
        eLastY = eY
        #print "C: ", x, y
        #print "multiplier : ",multiplier
        x=math.atan(x*multiplier)
        y=math.atan(y*multiplier)
	#print "added is : ", x,y
	X_output=x
	Y_output=y
	#print "output is : ", 	X_output, Y_output
        
      

        # set commands
#        ref.command[0] = cx
#        ref.command[1] = cy
        PYoutput.P[0] = X_output#bottom
        PYoutput.Y[0] = Y_output#top
	CHAN_1.put(PYoutput)
	#print "1"
    else: 
        # if not found, maximize e
	#print "not found>>>??>>>"    
#        ref.command[0] = 0
#        ref.command[1] = 0
        PYoutput.P[0] = 0
        PYoutput.Y[0] = 0
	CHAN_1.put(PYoutput)
#	print "2"
    # Sets reference to robot
    r.put(ref);

    # Sleeps
    time.sleep(.0)   
#-----------------------------------------------------
#--------[ Do not edit below ]------------------------
#-----------------------------------------------------
