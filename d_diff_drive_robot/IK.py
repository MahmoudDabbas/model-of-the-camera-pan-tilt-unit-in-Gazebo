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

dd = diff_drive
ref = dd.H_REF()
tim = dd.H_TIME()
ref2 = ipc.E_SERVO()
PY_IK = ipc.E_PY()#pitch yaw inverse kinematics

ROBOT_CHAN_VIEW   = 'robot-vid-chan'
ROBOT_TIME_CHAN  = 'robot-time'
ROBOT_DIFF_DRIVE_CHAN   = 'robot-diff-drive'

r = ach.Channel(ROBOT_DIFF_DRIVE_CHAN)
r.flush()
v = ach.Channel(ROBOT_CHAN_VIEW)
v.flush()
t = ach.Channel(ROBOT_TIME_CHAN)
t.flush()


DataP1 = ipc.E_PY()
CHAN_1 = ach.Channel(ipc.EXAMPLE_CHAN_1)
CHAN_2 = ach.Channel(ipc.EXAMPLE_CHAN_2)
CHAN_2.flush()

Theta_init=np.array([0.0,0.0])
Pc=1
Yc=1
Goal=np.array([Pc,Yc])
Goal=np.array([1,1])
Initial = np.array([Pc,Yc])

DeltaTheta = 0.02 #in radian
Error = 0.02 #in radian
step = .01  #in radian


def 	GetFK(theta):
	P=theta[0]
	Y=theta[1]
	#z=0
	e=np.array([P,Y])
	#print "theta in jacobian = ", theta
	#print "fK = ", e
	return (e)	


def 	GetDist(CurrentPosition,GoalPosition):
	return np.sqrt((np.power((GoalPosition[0]-CurrentPosition[0]),2)+np.power((GoalPosition[1]-CurrentPosition[1]),2)))


def 	GetNextPointDelta(EndEffector,Goal,Step):
	Slope = Goal - EndEffector
	UnitVector = Slope/GetDist(Slope,np.array([0,0]))
	StepVector = 	UnitVector*Step
	return (StepVector)


def 	GetJacobian (DeltaTheta,CurrentTheta):
	CurrentEndEffector = GetFK(Theta)
#	print CurrentEndEffector
	ThetaTemp=CurrentTheta
	for j in range(0,len(Theta)):
#		print CurrentEndEffector
		ThetaNew = CurrentTheta
		ThetaNew[j] = ThetaTemp[j] + DeltaTheta
		for i in range(0,len(EndEffector)):		 
#			print CurrentEndEffector
			DeltaEndEffector = GetFK(ThetaNew)[i] - CurrentEndEffector[i]
			Jacobian[i,j]=DeltaEndEffector/DeltaTheta
#			print "DeltaEndEffector :",i,j,DeltaEndEffector		
		ThetaNew[j] = ThetaTemp[j] - DeltaTheta
	return Jacobian
 
Theta = Theta_init
#print "Theta_init is : ",Theta_init
EndEffector = GetFK(Theta_init)
#print "EndEffector  is : ",EndEffector
#print "Goal  is : ",Goal

Jacobian = np.zeros(shape=(2,2))
x=GetDist(EndEffector,Goal)
#print "Distance is : ",x
#print "Error is : ",Error
#print "started IK.py >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>:"
while np.all(x>Error):

	CHAN_1.get(DataP1,wait=True,last=True)
#	DataP2.Value[0] = 2.5*DataP1.Value[0]
#	CHAN_2.put(DataP2)
#        print "2.5*DataP1.Value[0] : ",2.5*DataP1.Value[0]
	#print "Pitch received = : ",DataP1.P[0]
	#print "Yaw received = : ",DataP1.Y[0]

	Theta =np.array([DataP1.P[0],DataP1.Y[0]])

	Jacobian=GetJacobian(DeltaTheta,Theta)
#	print 	Jacobian
	PseduInvertedJacobian=np.linalg.pinv(Jacobian)
	DeltaEndEffector=GetNextPointDelta(EndEffector,Goal,step)
	DeltaTheta2=np.dot(DeltaEndEffector,PseduInvertedJacobian)
	Theta = Theta + DeltaTheta2
	EndEffector = GetFK(Theta)

	ref.ref[0]=Theta[0]
	ref.ref[1]=Theta[1]
        PY_IK.P[0] = Theta[0]
        PY_IK.Y[0] = Theta[1]
	r.put(ref);
	CHAN_2.put(PY_IK);

	x=GetDist(EndEffector,Goal)
	#print "EndEffector is : ",EndEffector, "     Distance is : ",x	
	#time.sleep(0.01)
#print "FINAL THETA IS :  ", Theta
