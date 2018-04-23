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


def getHomogZeros():
    out=np.zeros((4,1),dtype=np.float64)
    out[3,0]=1
    return out

class viewingAxis:
    def __init__(self,H=np.eye(4,4,dtype=np.float64)):
        self.Pose=H
        self.centre=getHomogZeros()
        self.x=getHomogZeros()
        self.x[0,0]=1
        self.y=getHomogZeros()
        self.y[1,0]=1
        self.z=getHomogZeros()
        self.z[2,0]=1
    def getAxisCoordinates(self):
        Result={}
        Result['centre']=self.Pose.dot(self.centre)
        Result['X']=self.Pose.dot(self.x)
        Result['Y']=self.Pose.dot(self.y)
        Result['Z']=self.Pose.dot(self.z)
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
        self.ax.set_xlim(-5,5)
        self.ax.set_ylim(-5,5)
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

