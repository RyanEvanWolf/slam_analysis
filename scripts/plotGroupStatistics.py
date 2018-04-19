#!/usr/bin/env python
import os
import sys
import cv2
import Structures

import rospy
import rosbag
import pickle


import matplotlib.pyplot as plt
import matplotlib.style as sty
pickleDir=sys.argv[1] 


stats = pickle.load( open( pickleDir, "r" ))


print("Total Loops="+str(len(stats)))
################
###plot success 
sty.use("seaborn")

fig1, axMotion = plt.subplots()
axMotion.set_xlabel("Frame Number")
axMotion.set_ylabel("Success")
#axMotion.set_title("Odometry Failure")

fig2, axTime = plt.subplots()
axTime.set_xlabel("Frame Number")
axTime.set_ylabel("Processing Time (ms)")
#axTime.set_title("Odometry Processing Time")

fig3, axInlier = plt.subplots()
axInlier.set_xlabel("Frame Number")
axInlier.set_ylabel("Inlier Ratio")
#axOutlier.set_title("Outlier Ratio over Sequence 1")

fig4, axMatch = plt.subplots()
axMatch.set_xlabel("Frame Number")
axMatch.set_ylabel("Total Features Tracked")
#axMatch.set_title("Odometry Processing Time")

fig5, axRMS = plt.subplots()
axRMS.set_xlabel("Frame Number")
axRMS.set_ylabel("RMS stereo Epipolar Error")
#axRMS.set_title("Odometry Processing Time")


for i in stats:
    motionFail=[]
    procTime=[]
    rms=[]
    matches=[]
    inlier=[]
    for data in i["Data"]:
        motionFail.append(data["Success"])
        procTime.append(data["Time"])
        rms.append(data["RMS"])
        matches.append(data["nMatches"])
        inlier.append(data["Inlier"])
    
    axMotion.plot(motionFail,label=i["Name"][1:])
    axTime.plot(procTime,label=i["Name"][1:])
    axInlier.plot(inlier,label=i["Name"][1:])
    axMatch.plot(matches,label=i["Name"][1:])
    axRMS.plot(rms,label=i["Name"][1:])



axMotion.legend()
axTime.legend()
axInlier.legend()
axMatch.legend()
axRMS.legend()

plt.show()