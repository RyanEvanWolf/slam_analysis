#!/usr/bin/env python
import os
import cv2
import sys

import time

from cv_bridge import CvBridge

###common ros messages and imports
import rospy
import copy
import argparse
import os
import rosbag


import numpy as np
cvb=CvBridge()

from datetime import datetime 

import matplotlib.pyplot as plt
import matplotlib.style as sty

fileInput=sys.argv[1]

bagName=fileInput.split("/")[-1]
print(bagName)

loopName=bagName[:bagName.rfind(".")]
loopName=loopName[loopName.find("_"):]
print(loopName)

#######
###Calculates libviso results from a rosbag trajectory



####these essentially define Enum type values, but indicate the position in the array

CurrentIndex=0
PreviousIndex=1

LeftIndex=0
RightIndex=1

class Feature:
    def __init__(self,u,v,id):
        self.u=u
        self.v=v
        self.id=id


class singleFrame:
    def __init__(self,lmsg,rmsg,vmsg,frameN):
        self.left=cvb.imgmsg_to_cv2(lmsg)
        self.right=cvb.imgmsg_to_cv2(rmsg)
        self.viso=copy.deepcopy(vmsg)
        self.nNumber=frameN
    def getMatches(self):
        ###[LeftFeature,RightFeature]
        currentMatches=[]
        previousMatches=[]
        for index in self.viso.matches:
            currentMatches.append((Feature(index.u1c.data,index.v1c.data,index.i1c.data),
                                    Feature(index.u2c.data,index.v2c.data,index.i2c.data)))
            previousMatches.append((Feature(index.u1p.data,index.v1p.data,index.i1p.data),
                                    Feature(index.u2p.data,index.v2p.data,index.i2p.data)))
        return (currentMatches,previousMatches)
    def drawFeatures(self):
        data=copy.deepcopy(self.getMatches())
        leftColour=cv2.cvtColor(self.left,cv2.COLOR_GRAY2RGB)
        rightColour=cv2.cvtColor(self.right,cv2.COLOR_GRAY2RGB)
        Epi=np.hstack((leftColour, rightColour))

        for featureIndex in range(0,len(data[0])):
            ##draw current features on left,right, and epipolar Image
            cv2.circle(leftColour,(int(data[CurrentIndex][featureIndex][LeftIndex].u),int(data[CurrentIndex][featureIndex][LeftIndex].v)),2,(0,255,0))
            cv2.circle(rightColour,(int(data[CurrentIndex][featureIndex][RightIndex].u),int(data[CurrentIndex][featureIndex][RightIndex].v)),2,(0,255,0))
            cv2.circle(Epi,(int(data[CurrentIndex][featureIndex][LeftIndex].u),int(round(data[CurrentIndex][featureIndex][LeftIndex].v))),2,(25,255,120))
            cv2.circle(Epi,(int(data[CurrentIndex][featureIndex][RightIndex].u)+leftColour.shape[1],int(round(data[CurrentIndex][featureIndex][RightIndex].v))),2,(25,255,120))
            ##draw previous features on left,right, and epipolar Image
            cv2.circle(leftColour,(int(data[PreviousIndex][featureIndex][LeftIndex].u),int(data[PreviousIndex][featureIndex][LeftIndex].v)),2,(255,0,0))
            cv2.circle(rightColour,(int(data[PreviousIndex][featureIndex][RightIndex].u),int(data[PreviousIndex][featureIndex][RightIndex].v)),2,(255,0,0))
            ##draw Tracked Lines
            cv2.line(leftColour,(int(data[CurrentIndex][featureIndex][LeftIndex].u),int(data[CurrentIndex][featureIndex][LeftIndex].v)),
                                (int(data[PreviousIndex][featureIndex][LeftIndex].u),int(data[PreviousIndex][featureIndex][LeftIndex].v)),(255,0,0),1)
            cv2.line(rightColour,(int(data[CurrentIndex][featureIndex][RightIndex].u),int(data[CurrentIndex][featureIndex][RightIndex].v)),
                                (int(data[PreviousIndex][featureIndex][RightIndex].u),int(data[PreviousIndex][featureIndex][RightIndex].v)),(255,0,0),1)

            #draw Epi Lines
            cv2.line(leftColour,(int(data[0][featureIndex][0].u),int(data[0][featureIndex][0].v)),
                                (int(data[1][featureIndex][0].u),int(data[1][featureIndex][0].v)),(255,0,0),1)

            #draw Epi Image
            cv2.line(Epi,(int(data[CurrentIndex][featureIndex][LeftIndex].u),int(round(data[CurrentIndex][featureIndex][LeftIndex].v))),
                                (int(data[CurrentIndex][featureIndex][RightIndex].u)+leftColour.shape[1],int(round(data[CurrentIndex][featureIndex][RightIndex].v))),(255,0,0),1)
        return (leftColour,rightColour,Epi)
    def getEpiPolarError(self):
        data=copy.deepcopy(self.getMatches())
        EpiError=[]
        for featureIndex in range(0,len(data[0])):
            e=data[CurrentIndex][featureIndex][LeftIndex].v-data[CurrentIndex][featureIndex][RightIndex].v
            EpiError.append(e)
        return EpiError

   
    

inputBag=rosbag.Bag(sys.argv[1])

leftImages=[]
rightImages=[]
outputData=[]




for topic,msg,t in inputBag.read_messages(topics=['/viso_extractor/output','/bumblebee/left/ROI','/bumblebee/right/ROI']):
    if(topic=="/viso_extractor/output"):
        outputData.append(msg)
    if(topic=="/bumblebee/left/ROI"):
        leftImages.append(msg)
    if(topic=="/bumblebee/right/ROI"):
        rightImages.append(msg)

#########
##calculate motion Fail Statistics
##get Processing time
motionFail=[]
procTime=[]


for i in outputData:
    if(i.success):
        motionFail.append(True)
    else:
        motionFail.append(False)
    procTime.append(i.time.data)


RMS_EPI=[]
Matches=[]
inputBag.close()
for i in range(1,len(leftImages)):
    a=singleFrame(leftImages[i],rightImages[i],outputData[i],i)
    err=a.getEpiPolarError()
    RMS=0
    if(len(err)>0):
        for featureError in err:
            RMS+=np.power(featureError,2)
        RMS=np.sqrt(RMS/len(err))
    print(i,len(err))
    RMS_EPI.append(RMS)
    Matches.append(len(a.getMatches()[LeftIndex]))




sty.use("seaborn")

plt.figure(300)
plt.xlabel("Frame Number")
plt.ylabel("Success")
plt.title("Odometry Failure")
plt.plot(motionFail)

plt.figure(600)
plt.xlabel("Frame Number")
plt.ylabel("Processing Time (ms)")
plt.plot(procTime)

plt.figure(400)
plt.xlabel("Frame Number")
plt.ylabel("RMS Epipolar Error")
plt.plot(RMS_EPI)


plt.figure(500)
plt.xlabel("Frame Number")
plt.ylabel("Total Features Tracked")
plt.plot(Matches)

plt.show()
