from __future__ import division
import diff_drive
import ach
import ipc
import sys
import time
from ctypes import *
import socket
import cv2.cv as cv
import cv2
import math
import numpy as np

PYoutput = ipc.E_PY()

CHAN_1 = ach.Channel(ipc.EXAMPLE_CHAN_1)
CHAN_1.flush()


dd = diff_drive
ref = dd.H_REF()
tim = dd.H_TIME()

ROBOT_DIFF_DRIVE_CHAN   = 'robot-diff-drive'
ROBOT_CHAN_VIEW   = 'robot-vid-chan'
ROBOT_TIME_CHAN  = 'robot-time'
# CV setup 
cv.NamedWindow("wctrl", cv.CV_WINDOW_AUTOSIZE)
#capture = cv.CaptureFromCAM(0)
#capture = cv2.VideoCapture(0)

# added
##sock.connect((MCAST_GRP, MCAST_PORT))
newx = 320
newy = 240
scale=2
nx = int(newx/scale)
ny = int(newy/scale)

r = ach.Channel(ROBOT_DIFF_DRIVE_CHAN)
r.flush()
v = ach.Channel(ROBOT_CHAN_VIEW)
v.flush()
t = ach.Channel(ROBOT_TIME_CHAN)
t.flush()

i=0
X_output=0
Y_output=0
tLast = 0
eLastX = 0
eLastY = 0
cLastX = 0
cLastY = 0
counter=0
depth=0.085/scale
print "depth : ",depth
multiplier = (280/1000000)/(depth)
print "multiplier is : ",multiplier

print '======================================'
print '============= Robot-View ============='
print '========== Daniel M. Lofaro =========='
print '========= dan@danLofaro.com =========='
print '======================================'
while True:
    # Get Frame
    img = np.zeros((newx,newy,3), np.uint8)
    c_image = img.copy()
    vid = cv2.resize(c_image,(newx,newy))
    [status, framesize] = v.get(vid, wait=False, last=True)
    if status == ach.ACH_OK or status == ach.ACH_MISSED_FRAME or status == ach.ACH_STALE_FRAMES:
        vid2 = cv2.resize(vid,(nx,ny))
        img = cv2.cvtColor(vid2,cv2.COLOR_BGR2RGB)
        cv2.imshow("wctrl", img)
        cv2.waitKey(10)
        numRows = img.shape[0]
        numCols = img.shape[1]
        lower = np.array([150, 0, 0], np.uint8)
        upper = np.array([255, 150, 150], np.uint8)
        # use cv funtion to find the blue mask
        mask = cv2.inRange(img, lower, upper)
#       cv2.imshow("wctrl", mask)
#       cv2.waitKey(10)

    else:
        raise ach.AchException( v.result_string(status) )


    [status, framesize] = t.get(tim, wait=False, last=True)
    if status == ach.ACH_OK or status == ach.ACH_MISSED_FRAME or status == ach.ACH_STALE_FRAMES:
        pass
        #print 'Sim Time = ', tim.sim[0]
    else:
        raise ach.AchException( v.result_string(status) )

#-----------------------------------------------------
#--------[ Do not edit above ]------------------------
#-----------------------------------------------------
    # Def:
    # ref.ref[0] = Right Wheel Velos
    # ref.ref[1] = Left Wheel Velos
    # tim.sim[0] = Sim Time
    # img        = cv image in BGR format

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
    if a > 3:
        x = tx / a - nx/2
        y = -(ty / a - ny/2)
        found = True
#        print "Found: ", found, "at", x, ",", y
#	print "a equals : ", a
    else: 
        print "Not found"
	counter = counter +1
	print counter

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
        print "C: ", x, y
        print "multiplier : ",multiplier
        x=math.atan(x*multiplier)
        y=math.atan(y*multiplier)
	print "added is : ", 	x,y
	X_output=X_output-x
	Y_output=Y_output-y
	print "output is : ", 	X_output, Y_output
        
		

        # set commands
        #ref.ref[0] = X_output
        #ref.ref[1] = Y_output
        PYoutput.P[0] = X_output#bottom
        PYoutput.Y[0] = Y_output#top
	CHAN_1.put(PYoutput)

	print "1"
    else: 
        # if not found, maximize e
	#print "not found>>>??>>>"    
        PYoutput.P[0] = X_output#bottom
        PYoutput.Y[0] = Y_output#top
	CHAN_1.put(PYoutput)
	#print "2"
    # Sets reference to robot
#    r.put(ref);

    # Sleeps
    time.sleep(0.1)   
#-----------------------------------------------------
#--------[ Do not edit below ]------------------------
#-----------------------------------------------------
