import sys
import time
import math
from time import sleep
import numpy as np
pi = 3.141592654
Theta_init=np.array([pi,0.0,0.0,0.0,0.0,0.0])#represent the angles of the MDS robot, namely the Shoulder pitch, shoulder roll, shoulder yaw, elbow pitch, wrist yaw, wrist roll, respictivily.
Length = np.array([0.24551,0.282575,0.3127375,0.0635])#represent the lengths of the arm components. namely the shoulder, the arm, the forearm and the hand/fingers
l1 = Length[0]#the shoulder
l2 = Length[1]#the arm
l3 = Length[2]#the forearm
l4 = Length[3]#the hand/fingers

#new identity matrices to initiate the transformation matrices.
T01=np.identity(4)#for the shoulder pitch and translation
T12=np.identity(4)#for the shoulder roll
T23=np.identity(4)#for the shoulder yaw
T34=np.identity(4)#for the elbow pitch and translation 
T45=np.identity(4)#for the wrist yaw and translation
T56=np.identity(4)#for the wrist roll 
T67=np.identity(4)#for the hand translation

#here we change the matrices
def 	GetFK(theta):
#for the shoulder pitch and translation
	T01[0,0]=np.cos(theta[0])
	T01[2,2]=T01[0,0]
	T01[0,2]=np.sin(theta[0])
	T01[2,0]=-1*T01[0,2]
	T01[1,3]=l1
#for the shoulder roll
	T12[1,1]=np.cos(theta[1])
	T12[2,2]=T12[1,1]
	T12[1,2]=-1*np.sin(theta[1])
	T12[2,1]=-1*T12[0,2]
	T12[1,3]=0
#for the shoulder yaw
	T23[0,0]=np.cos(theta[2])
	T23[1,1]=T23[0,0]
	T23[0,1]=-1*np.sin(theta[2])
	T23[1,0]=-1*T23[0,1]
	T23[1,3]=0
#for the elbow pitch and translation
	T34[0,0]=np.cos(theta[3])
	T34[2,2]=T34[0,0]
	T34[0,2]=np.sin(theta[3])
	T34[2,0]=-1*T34[0,2]
	T34[2,3]=-1*l2
#for the wrist yaw and translation
	T45[0,0]=np.cos(theta[4])
	T45[1,1]=T45[0,0]
	T45[0,1]=-1*np.sin(theta[4])
	T45[1,0]=-1*T45[0,1]
	T45[2,3]=-1*l3
#for the wrist roll
	T56[1,1]=np.cos(theta[5])
	T56[2,2]=T56[1,1]
	T56[1,2]=-1*np.sin(theta[5])
	T56[2,1]=-1*T56[1,2]
	T56[1,3]=0
#for the hand translation
	T67[2,3]=-1*l4
#cross multiplying the matrices with each other to get the final transformation matrix
	T=np.dot(T01,T12)
	T=np.dot(T,T23)
	T=np.dot(T,T34)
	T=np.dot(T,T45)
	T=np.dot(T,T56)
	T=np.dot(T,T67)
	print "T01 is : "
	print T01
	print "T12 is : "
	print T12
	print "T23 is : "
	print T23
	print "T34 is : "
	print T34
	print "T45 is : "
	print T45
	print "T56 is : "
	print T56
	print "T67 is : "
	print T67
	print "T is : "
	print T
#calculating the x, y and z Angles of endeffector location 
	ThetaX= round(math.atan2(T[2,1],T[2,2]),1)*180/pi
	ThetaY= round(math.atan2(-1*T[2,0],math.pow(T[2,1],2)+math.pow(T[2,2],2)),1)*180/pi
	ThetaZ= round(math.atan2(T[1,0],T[0,0])*180/pi,1)
	Theta=np.array([ThetaX,ThetaY,ThetaZ])
#multipling the final coordination matrix with the coordinates of the hand effector in the last space to get the location vector T
	T=np.dot(T,np.array([[0],[0],[0],[1]]))
	x=round(T[0],3)#extracting x value of the end effector from the location vector
	y=round(T[1],3)#extracting y value of the end effector from the location vector
	z=round(T[2],3)#extracting z value of the end effector from the location vector
	e=np.array([x,y,z])
	return (e,Theta)	


#a function to calculate the distance between the end-effector and the origin
def 	GetDist(Endeffector,Goal):
	return np.sqrt((np.power((Endeffector[0]-Goal[0]),2)+np.power((Endeffector[1]-Goal[1]),2)+np.power((Endeffector[2]-Goal[2]),2)))


Theta = Theta_init
print "Theta_init is : ",Theta_init
EndEffector,Theta_endeffector = GetFK(Theta_init)#extracting information from hte FK function
print "EndEffector  is : ",EndEffector

Distance=GetDist(EndEffector,(np.array([0,0,0])))
print "Distance is : ",Distance

print "FINAL Theta of endeffector ThetaX,  ThetaY, ThetaZ IS :  ", Theta_endeffector
