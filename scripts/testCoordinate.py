#!/usr/bin/env python
import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import matplotlib.pyplot as plt

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
ax.set_xlim3d(0, 3)
ax.set_ylim3d(0,3)
ax.set_zlim3d(0,3)


newMotion=np.eye(4,dtype=np.float64)
plotPose(ax,newMotion)
plotPose(ax,newMotion)


ax.legend()
plt.show()

