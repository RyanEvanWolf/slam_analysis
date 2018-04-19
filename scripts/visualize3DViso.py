#!/usr/bin/env python
import os
import cv2
import sys

import time

from math import pi

from datetime import datetime 
from cv_bridge import CvBridge
import numpy as np

###common ros messages and imports
import rospy
import copy
import argparse

from geometry_msgs.msg import PoseStamped,PoseArray,Quaternion,Point,Pose
import tf

import rosbag
import numpy as np

from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA

import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt


cvb=CvBridge()


def getHomogZeros():
    out=np.zeros((4,1),dtype=np.float64)
    out[3,0]=1
    return out

def plotPose(graph,H,scale=0.2):
    ###
    centre=getHomogZeros()
    newCentre=H.dot(centre)
    xAxis=getHomogZeros()
    xAxis[0,0]=scale
    yAxis=getHomogZeros()
    yAxis[1,0]=scale
    zAxis=getHomogZeros()
    zAxis[2,0]=scale
    ##get relative points
    newX=H.dot(xAxis)
    newY=H.dot(yAxis)
    newZ=H.dot(zAxis)
    ###draw them
    graph.plot((newCentre[0,0],newX[0,0]),             
                (newCentre[1,0],newX[1,0]),
                (newCentre[2,0],newX[2,0]),
                color='red')
    graph.plot((newCentre[0,0],newY[0,0]),             
                (newCentre[1,0],newY[1,0]),
                (newCentre[2,0],newY[2,0]),
                color='green')
    graph.plot((newCentre[0,0],newZ[0,0]),             
                (newCentre[1,0],newZ[1,0]),
                (newCentre[2,0],newZ[2,0]),
                color='blue')


mpl.rcParams['legend.fontsize'] = 10

fig = plt.figure()
ax = fig.gca(projection='3d')
ax.set_xlabel('X axis')
ax.set_ylabel('Y axis')
ax.set_zlabel('Z axis')


ax.view_init(elev=0,azim=-90)
ax.legend()




def convertHomographyToMsg(homography):
    msg=Pose()
    quat=tf.transformations.quaternion_from_matrix(homography).reshape(1,4)
    msg.orientation=Quaternion(quat[0,0],quat[0,1],quat[0,2],quat[0,3])
    msg.position.x=homography[0,3]
    msg.position.y=homography[1,3]
    msg.position.z=homography[2,3]
    return msg

def deserialHomography(arrayIn):
    outHomography=np.zeros((4,4),dtype=np.float64)
    row=0
    col=0
    for row in range(0,4):
        for col in range(0,4):
            outHomography[row,col]=arrayIn[row*4+col]
    return outHomography

def getMotion(visoList):
    homographies=[]
    currentPosition=np.eye(4,dtype=np.float64)
    Poses=[]
    fails=[]
    for index in range(1,len(visoList)):
        if(visoList[index].success):
            homographies.append(deserialHomography(visoList[index].homography))
            fails.append(True)
        else:
            homographies.append(homographies[-1])
            fails.append(False)
        currentPosition=currentPosition.dot(homographies[-1])
        Poses.append(currentPosition)
    print(len(homographies),len(Poses),len(fails))
    return (homographies,Poses,fails)

class singleFrame:
    def __init__(self,lmsg,rmsg,vmsg,frameN):
        self.left=cvb.imgmsg_to_cv2(lmsg)
        self.right=cvb.imgmsg_to_cv2(rmsg)
        self.viso=copy.deepcopy(vmsg)
        self.nNumber=frameN
    def getHomography(self):
        return deserialHomography(self.viso.homography)

  
inDir=sys.argv[1]

print("Reading from "+inDir)

inputBag=rosbag.Bag(inDir)

leftImages=[]
rightImages=[]
outputData=[]


print("extracting Topic Data")



for topic,msg,t in inputBag.read_messages(topics=['/viso_extractor/output']):#,'/bumblebee/left/ROI','/bumblebee/right/ROI']):
    if(topic=="/viso_extractor/output"):
        outputData.append(msg)
        print(len(outputData))
    # if(topic=="/bumblebee/left/ROI"):
    #     leftImages.append(msg)
    # if(topic=="/bumblebee/right/ROI"):
    #     rightImages.append(msg)
inputBag.close()
##only start from the second frame (once a motion was calculated)

CurrentPose=np.eye(4,dtype=np.float64)
previousMotion=np.eye(4,dtype=np.float64)
displayArray=PoseArray()
displayArray.header.frame_id='sensor'



projec=[]

print("calculating Motion")

for index in range(1,200):#len(outputData)):
    print(index)
    #latestFrame=singleFrame(leftImages[index],rightImages[index],outputData[index],index)
    if(outputData[index].success):
        latestMotion=np.linalg.inv(deserialHomography(outputData[index].homography))
        plotPose(ax,latestMotion)
        CurrentPose=CurrentPose.dot(latestMotion)
        plotPose(ax,CurrentPose)
        previousMotion=latestMotion
    else:
        CurrentPose=CurrentPose.dot(previousMotion)
        plotPose(ax,CurrentPose)
print("published!")
plt.show()

print("Completed")