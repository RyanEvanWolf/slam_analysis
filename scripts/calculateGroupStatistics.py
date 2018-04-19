#!/usr/bin/env python
import os
import sys
import cv2
import Structures

import rospy
import rosbag
import pickle


mainDir=sys.argv[1]
outputDir=sys.argv[2]
if(not os.path.exists(outputDir)):
    os.makedirs(outputDir)

print("reading Bags From folder "+mainDir)

bagList=[]
#####get Bag information 
for files in os.listdir(mainDir):
    if(files[files.rfind("."):]==".bag"):
        bagData={}
        bagData["dir"]=mainDir+"/"+files
        bagData["file"]=files
        loopName=files.split(".")[0]
        loopName=loopName[loopName.find('_'):]
        bagData["name"]=loopName
        print(bagData)
        bagList.append(bagData)
################
####Analyse each Bag

StatisticData=[]




for metaInfo in bagList:
    singlebagData={"Name":metaInfo["name"],"Data":[]}
    bag=rosbag.Bag(metaInfo["dir"])
    print("Rosbag opened at "+metaInfo["dir"])
    leftImage=None
    rightImage=None
    visoData=None
    frames=[]
    #####extract messages from rosbag
    #############
    for topic,msg,t in bag.read_messages(topics=['/viso_extractor/output','/bumblebee/left/ROI','/bumblebee/right/ROI']):
        if(topic=="/viso_extractor/output"):
            visoData=msg
        if(topic=="/bumblebee/left/ROI"):
            leftImage=msg
        if(topic=="/bumblebee/right/ROI"):
            rightImage=msg
        if((leftImage!=None)and(rightImage!=None)and(visoData!=None)):
            frameData=Structures.singleFrame(leftImage,rightImage,visoData,len(frames))
            frames.append(frameData)
            leftImage=None
            rightImage=None
            visoData=None
            print("frames extracted "+str(len(frames)))
    bag.close()
    print("Rosbag Closed at "+metaInfo["dir"])
    ####################
    #####Calculate Statistics (skip the first frame always)
    for index in range(1,len(frames)):
        data={"RMS"}
        matches=Structures.getVisoMatches(frames[index].viso)
        epiError=Structures.getEpiPolarError(matches["current"])
        rms=Structures.getRMS(epiError)
        try:
            inlier=(frames[index].viso.nInliers.data/frames[index].viso.nMatches.data)
        except:
            inlier=0

        data={"RMS":rms,
            "Inlier":inlier,
            "nMatches":len(matches["current"]),
            "Success":frames[index].viso.success,
            "Time":frames[index].viso.time.data,
            "Frame":index}
        singlebagData["Data"].append(data)
        print(index)
    StatisticData.append(singlebagData)

pickleDir=outputDir+"/groupStatistics.p" 

pickle.dump(StatisticData,open(pickleDir, "w" ))




