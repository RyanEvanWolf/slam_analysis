#!/usr/bin/env python
import os
import cv2
import sys

import time

from datetime import datetime 

import matplotlib.pyplot as plt


###common ros messages and imports
import rospy

import argparse

parser = argparse.ArgumentParser(description='Given a Bumblebee dataset root folder, Analyse and plot the FPS information')
 
# Required positional argument
parser.add_argument('dataset_directory', type=str,
                    help='root folder of the dataset')

# Optional positional argument
parser.add_argument('outputFolder', type=str, nargs='?',
                    help='Location to store the outputInformation')


args=parser.parse_args()

if(args.dataset_directory==None):
    parser.error("A Root folder to the bumblebee dataset folder is required")

datasetName=args.dataset_directory[args.dataset_directory.rfind('/')+1:-1]

if(args.outputFolder==None):
    outputFolder="/home/ryan/git/Output/"+datasetName 
else:
    outputFolder=args.outputFolder


if(not os.path.isdir(outputFolder)):
    os.makedirs(outputFolder)


print("Configuration")
print("rootDirectory",args.dataset_directory)
print("outFolder",outputFolder)

###Load the filenames list with no extensions
data=[]


import collections
# a = [1,1,1,1,2,2,2,2,3,3,4,5,5]
# counter=collections.Counter(a)
# print(counter)
# # Counter({1: 4, 2: 4, 3: 2, 5: 2, 4: 1})
# print(counter.values())
# # [4, 4, 2, 1, 2]
# print(counter.keys())
# # [1, 2, 3, 4, 5]
# print(counter.most_common(3))
def getFloatTime(unixString):
    t=unixString.split('-')
    seconds=float(t[1][2:4])*60+float(t[1][4:6])
    outTime=seconds+(10**-3)*float(t[2])
    return outTime

a=[]
b=[]
for files in os.listdir(args.dataset_directory):
    if(files!='camera_setup.txt'):
        t=float(files.split("-")[1])
        second=float(files.split("-")[2][0:-4])
        a.append(t)
        b.append(second)
        print(t,second)
        #d=datetime.strptime('Jun 1 2005  1:33PM', '%b %d %Y %I:%M%p')
        #f=(files[0:-4],getFloatTime(files[0:-4]))
        #data.append(f)

c=collections.Counter(a)
plt.plot(c.values())
plt.show()
# data.sort(key=lambda x: getFloatTime)


# l=[]

# for i in data:
#     l.append(i[1])


# plt.scatter(range(0,len(l)),l)
# plt.show()

# ###get the time differences in milliseconds between frames
# diff=[]
# for i in range(1,len(a)):
#     difference=a[i]-a[i-1]
#     difference=difference*(10**3)
#     diff.append(difference)

# #plt.ylim(0,1000)
# #plt.scatter(range(0,len(diff)),diff,s=1)
# plt.plot(diff,lw=5)
# plt.show()


# print("Loaded")


#         self.rootDir=inDir
#         tempFiles=[]
#         for files in os.listdir(inDir):
#             tempFiles.append(self.rootDir+"/"+files)
#         self.filesList=sorted(tempFiles)
#         self.currentIndex=-1

# rospy.init_node("plotJitter") 

# rospy.spin()
