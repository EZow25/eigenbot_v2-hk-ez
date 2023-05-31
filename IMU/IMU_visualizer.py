#!/usr/bin/env python3

from vpython import *
from time import *
import numpy as np
import math
import serial
#ad=serial.Serial('COM4')
ad=serial.Serial('/dev/ttyACM0')


sleep(1)


 
scene.range=6
scene.background=color.yellow
toRad=2*np.pi/360
toDeg=1/toRad
scene.forward=vector(-1,-1,-1)
 
scene.width=1200
scene.height=1080
 
# xarrow=arrow(length=2, shaftwidth=.1, color=color.red,axis=vector(1,0,0))
# yarrow=arrow(length=2, shaftwidth=.1, color=color.green,axis=vector(0,1,0))
# zarrow=arrow(length=4, shaftwidth=.1, color=color.blue,axis=vector(0,0,1))
 
frontArrow=arrow(length=4,shaftwidth=.1,color=color.red,axis=vector(1,0,0))
upArrow=arrow(length=1,shaftwidth=.1,color=color.green,axis=vector(0,1,0))
sideArrow=arrow(length=2,shaftwidth=.1,color=color.blue,axis=vector(0,0,1))
 
bBoard=box(length=6,width=2,height=.2,opacity=.8,pos=vector(0,0,0,))
#bn=box(length=1,width=.75,height=.1, pos=vector(-.5,.1+.05,0),color=color.blue)
#nano=box(length=1.75,width=.6,height=.1,pos=vector(-2,.1+.05,0),color=color.green)
#myObj=compound([bBoard,bn,nano])
myObj=compound([bBoard])
while (True):
    try:
        while (ad.inWaiting()==0):
            #print("pass in waiting")
            pass
        dataPacket=ad.readline()
        dataPacket=str(dataPacket,'utf-8')
        splitPacket=dataPacket.split(",")
        q0=float(splitPacket[0])
        q1=float(splitPacket[1])
        q2=float(splitPacket[2])
        q3=float(splitPacket[3])
 
        roll=math.atan2(2*(q0*q1+q2*q3),1-2*(q1*q1+q2*q2))
        pitch=-math.asin(2*(q0*q2-q3*q1))
        yaw=-math.atan2(2*(q0*q3+q1*q2),1-2*(q2*q2+q3*q3))
 
        rate(200)
        k=vector(cos(yaw)*cos(pitch), sin(pitch),sin(yaw)*cos(pitch))
        y=vector(0,1,0)
        s=cross(k,y)
        v=cross(s,k)
        vrot=v*cos(roll)+cross(k,v)*sin(roll)
 
        frontArrow.axis=k
        sideArrow.axis=cross(k,vrot)
        upArrow.axis=vrot
        myObj.axis=k
        myObj.up=vrot
        sideArrow.length=2
        frontArrow.length=4
        upArrow.length=1
        print("roll: %f, pitch: %f, yaw: %f, q0: %f, q1: %f, q2: %f, q3: %f" % (roll, pitch, yaw, q0, q1, q2, q3))
    except:
        print("pass in except")
        pass