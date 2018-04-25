#!/usr/bin/env python
import os
import cv2
import sys

import time

from math import pi

from datetime import datetime 
from cv_bridge import CvBridge
import numpy as np

from Queue import Queue

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

from threading import Thread,Lock

import matplotlib as mpl
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D

from slam_analysis.srv import *


####
##always in sensor Frame
def getDefaultColours():
    colours={}
    colours["X"]=(1,0,0,1)
    colours["Y"]=(0,1,0,1)
    colours["Z"]=(0,0,1,1)
    return colours

def getUnitAxis(xcolour=(0,0,1,1),ycolour=(0,1,0,1),zcolour=(1,0,0,1)):
    c=getHomogZeros()
    x=getHomogZeros()
    x[0,0]=1
    y=getHomogZeros()
    y[1,0]=1
    z=getHomogZeros()
    z[2,0]=1
    result={}
    result["system"]="unit"
    result["Pose"]=np.eye(4,dtype=np.float64)
    result['X']={}
    result['Y']={}
    result['Z']={}
    result['centre']=c
    result['X']['coord']=x
    result['Y']['coord']=y
    result['Z']['coord']=z
    result['X']['colour']=xcolour
    result['Y']['colour']=ycolour
    result['Z']['colour']=zcolour
    return result

def getHomogZeros():
    out=np.zeros((4,1),dtype=np.float64)
    out[3,0]=1
    return out

class displayAxis:
    def __init__(self,H=np.eye(4,4,dtype=np.float64)):
        self.frameName="unit"
        self.Pose=H
        self.centre=getHomogZeros()
        self.x=getHomogZeros()
        self.x[0,0]=1
        self.xcolour=(1,0,0,1)
        self.y=getHomogZeros()
        self.ycolour=(0,1,0,1)
        self.y[1,0]=1
        self.z=getHomogZeros()
        self.zcolour=(0,0,1,1)
        self.z[2,0]=1
    # def getAxisCoordinates(self):
    #     Result={}

    #     c=self.Pose.dot(self.centre)
    #     x=self.Pose.dot(self.x)
    #     y=self.Pose.dot(self.y)
    #     z=self.Pose.dot(self.z)

    #     Result['centre']=self.sensor2World.dot(self.centre)
    #     Result['X']=self.sensor2World.dot(self.x)
    #     Result['Y']=self.sensor2World.dot(self.y)
    #     Result['Z']=self.sensor2World.dot(self.z)
    #     return Result


class viewingAxis:
    def __init__(self,H=np.eye(4,4,dtype=np.float64),H2=np.eye(4,4,dtype=np.float64)):
        self.Pose=H
        self.sensor2World=H2
        self.centre=getHomogZeros()
        self.x=getHomogZeros()
        self.x[0,0]=1
        self.y=getHomogZeros()
        self.y[1,0]=1
        self.z=getHomogZeros()
        self.z[2,0]=1
    def getAxisCoordinates(self):
        Result={}

        c=self.Pose.dot(self.centre)
        x=self.Pose.dot(self.x)
        y=self.Pose.dot(self.y)
        z=self.Pose.dot(self.z)

        Result['centre']=self.sensor2World.dot(self.centre)
        Result['X']=self.sensor2World.dot(self.x)
        Result['Y']=self.sensor2World.dot(self.y)
        Result['Z']=self.sensor2World.dot(self.z)
        return Result


class display2D:
    def __init__(self):
        self.initGraph()
        self.Q=Queue()
        self.data={}
        self.addNewPose("world",viewingAxis())
        self.addNewPose("pose",viewingAxis())
        #self.data,=self.ax.plot([se],[])
        self.ani=animation.FuncAnimation(self.fig,self.update,interval=15)
    def initGraph(self):
        self.fig,self.ax=plt.subplots()
        self.ax.set_xlabel('X axis')
        self.ax.set_ylabel('Z axis')
        self.ax.set_xlim(-10,10)
        self.ax.set_ylim(-10,10)
        self.ax.grid(color='black', linestyle='-', linewidth=0.5)
        self.ax.legend()
    def addNewPose(self,key,newPose):
            self.data[key]={}
            self.data[key]["coordinates"]=newPose.getAxisCoordinates()
            xData,=self.ax.plot((self.data[key]["coordinates"]['centre'][0,0],
                                self.data[key]["coordinates"]['X'][0,0]),
                                (self.data[key]["coordinates"]['centre'][2,0],
                                self.data[key]["coordinates"]['X'][2,0]),
                                color=(1,0,0,1))
            zData,=self.ax.plot((self.data[key]["coordinates"]['centre'][0,0],
                                self.data[key]["coordinates"]['Z'][0,0]),
                                (self.data[key]["coordinates"]['centre'][2,0],
                                self.data[key]["coordinates"]['Z'][2,0]),
                                color=(0,1,0,1))
            self.data[key]["pyplot"]=(xData,zData)
    def updatePose(self,key,coordinates):
        self.data[key]["pyplot"][0].set_data((coordinates['centre'][0,0],
                                coordinates['X'][0,0]),
                                (coordinates['centre'][2,0],
                                coordinates['X'][2,0]))
        self.data[key]["pyplot"][1].set_data((coordinates['centre'][0,0],
                        coordinates['Z'][0,0]),
                        (coordinates['centre'][2,0],
                        coordinates['Z'][2,0]))
    def updateData(self,req):
        print(req)
    def update(self,i):
        if(not self.Q.empty()):
            newPose=self.Q.get()
            self.updatePose("pose",newPose.getAxisCoordinates())


class display3D:
    def __init__(self,coordTransform=np.eye(4,dtype=np.float64)):
        self.coordTransform=coordTransform
        self.Q=Queue()

        self.data={}
        self.initGraph()
        #self.addAxis("world")
        self.ani=animation.FuncAnimation(self.fig,self.update,interval=500)
    def addAxis(self,key,colours=getDefaultColours()):
        self.data[key]={}
        self.data[key]["axis"]=displayAxis()
        self.data[key]["graph"]={}
        xAxis,=self.ax.plot3D((self.data[key]["axis"].centre[0,0],self.data[key]["axis"].x[0,0]),
                            (self.data[key]["axis"].centre[1,0],self.data[key]["axis"].x[1,0]),
                            (self.data[key]["axis"].centre[2,0],self.data[key]["axis"].x[2,0]),
                            color=colours["X"],label=key)
        yAxis,=self.ax.plot3D((self.data[key]["axis"].centre[0,0],self.data[key]["axis"].y[0,0]),
                            (self.data[key]["axis"].centre[1,0],self.data[key]["axis"].y[1,0]),
                            (self.data[key]["axis"].centre[2,0],self.data[key]["axis"].y[2,0]),
                            color=colours["Y"])
        zAxis,=self.ax.plot3D((self.data[key]["axis"].centre[0,0],self.data[key]["axis"].z[0,0]),
                            (self.data[key]["axis"].centre[1,0],self.data[key]["axis"].z[1,0]),
                            (self.data[key]["axis"].centre[2,0],self.data[key]["axis"].z[2,0]),
                            color=colours["Z"])
        self.data[key]["graph"]["X"]=xAxis
        self.data[key]["graph"]["Y"]=yAxis
        self.data[key]["graph"]["Z"]=zAxis
    def updateAxis(self,key,axis):
        #######
        ##transform into display Frame
        print(axis.Pose)
        print(self.coordTransform.dot(axis.Pose))
        poseCentre=axis.Pose.dot(axis.centre)
        poseX=axis.Pose.dot(axis.x)
        poseY=axis.Pose.dot(axis.y)
        poseZ=axis.Pose.dot(axis.z)
        if(key!="world"):
            newCentre=self.coordTransform.dot(poseCentre)
            newX=self.coordTransform.dot(poseX)
            newY=self.coordTransform.dot(poseY)
            newZ=self.coordTransform.dot(poseZ)
        else:
            newCentre=poseCentre
            newX=poseX
            newY=poseY
            newZ=poseZ
        self.data[key]["graph"]["X"].set_xdata((newCentre[0,0],newX[0,0]))
        self.data[key]["graph"]["X"].set_ydata((newCentre[1,0],newX[1,0]))
        self.data[key]["graph"]["X"].set_3d_properties((newCentre[2,0],newX[2,0]))
        self.data[key]["graph"]["Y"].set_xdata((newCentre[0,0],newY[0,0]))
        self.data[key]["graph"]["Y"].set_ydata((newCentre[1,0],newY[1,0]))
        self.data[key]["graph"]["Y"].set_3d_properties((newCentre[2,0],newY[2,0]))
        self.data[key]["graph"]["Z"].set_xdata((newCentre[0,0],newZ[0,0]))
        self.data[key]["graph"]["Z"].set_ydata((newCentre[1,0],newZ[1,0]))
        self.data[key]["graph"]["Z"].set_3d_properties((newCentre[2,0],newZ[2,0]))
    def update(self,i):
        if(not self.Q.empty()):
            newPose=self.Q.get()
            if(newPose[0] not in self.data):
                self.addAxis(newPose[0])
            self.updateAxis(newPose[0],newPose[1])
            #     xData,=self.ax.plot((self.data[key]["coordinates"]['centre'][0,0],
    #                         self.data[key]["coordinates"]['X'][0,0]),
    #                         (self.data[key]["coordinates"]['centre'][2,0],
    #                         self.data[key]["coordinates"]['X'][2,0]),
    #                         color=(1,0,0,1))
    #     zData,=self.ax.plot((self.data[key]["coordinates"]['centre'][0,0],
    #                         self.data[key]["coordinates"]['Z'][0,0]),
    #                         (self.data[key]["coordinates"]['centre'][2,0],
    #                         self.data[key]["coordinates"]['Z'][2,0]),
    #                         color=(0,1,0,1))
    # def addAxis(self,key):
    #     unit=getUnitAxis()
    #     ##convert to display Frame
    #     self.data[key]={}
    #     xAxis,=self.ax.plot((unit["centre"][0,0],unit["X"][0,0]),
    #                         (),
    #                         (),
    #                         )
    # def addNewPose(self,key,newPose):
    #     self.data[key]={}
    #     self.data[key]["coordinates"]=newPose.getAxisCoordinates()
    #     xData,=self.ax.plot((self.data[key]["coordinates"]['centre'][0,0],
    #                         self.data[key]["coordinates"]['X'][0,0]),
    #                         (self.data[key]["coordinates"]['centre'][2,0],
    #                         self.data[key]["coordinates"]['X'][2,0]),
    #                         color=(1,0,0,1))
    #     zData,=self.ax.plot((self.data[key]["coordinates"]['centre'][0,0],
    #                         self.data[key]["coordinates"]['Z'][0,0]),
    #                         (self.data[key]["coordinates"]['centre'][2,0],
    #                         self.data[key]["coordinates"]['Z'][2,0]),
    #                         color=(0,1,0,1))
    #     self.data[key]["pyplot"]=(xData,zData)
    def initGraph(self):
        self.fig = plt.figure()
        self.ax = self.fig.gca(projection='3d')
        self.ax.set_xlabel('X axis')
        self.ax.set_ylabel('Y axis')
        self.ax.set_zlabel('Z axis')
        self.ax.legend()


###3DD!!!!
# class display2D:
#     def __init__(self):
#         self.initGraph()
#         self.ani=animation.FuncAnimation(self.fig,self.update,interval=500)
#     def initGraph(self):
#         self.fig=plt.figure()
#         ax = self.fig.gca(projection='3d')
#         ax.set_xlabel('X axis')
#         ax.set_ylabel('Y axis')
#         ax.set_zlabel('Z axis')


#         ax.view_init(elev=0,azim=-90)
#         ax.legend()
#     def update(self,i):
#         pass

