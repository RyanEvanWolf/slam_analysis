import copy
import numpy as np
class visoFeature:
    def __init__(self,u,v,id):
        self.u=u
        self.v=v
        self.id=id


class visoBagStatistics:
    def __init__(self):
        self.data=0


class singleFrame:
    def __init__(self,l,r,v,frameN):
        self.left=copy.deepcopy(l)#cvb.imgmsg_to_cv2(lmsg)
        self.right=copy.deepcopy(r)#cvb.imgmsg_to_cv2(rmsg)
        self.viso=copy.deepcopy(v)
        self.nNumber=frameN


def getVisoMatches(visoOutput):
    outData={"current":[],"previous":[]}
    for match in visoOutput.matches:
        currentFeature={"left":{"u":match.u1c.data,"v":match.v1c.data,"id":match.i1c.data},"right":{"u":match.u2c.data,"v":match.v2c.data,"id":match.i2c.data}}
        previousFeature={"left":{"u":match.u1p.data,"v":match.v1p.data,"id":match.i1p.data},"right":{"u":match.u2p.data,"v":match.v2p.data,"id":match.i2p.data}}
        outData["current"].append(currentFeature)
        outData["previous"].append(previousFeature)
    return outData

def getVisoProcTime(setFrames):
    times=[]
    for i in setFrames:
        times.append(i.viso.time.data)
    return times

def getMotionSuccess(setFrames):
    success=[]
    for i in setFrames:
        if(i.viso.success):
            success.append(True)
        else:
            success.append(False)
        success.append(i.viso.success)
    return success

def getEpiPolarError(matches):
    error=[]
    for i in matches:
        e=i["left"]["v"]-i["right"]["v"]
        error.append(e)
    return error

def getRMS(errorList):
    RMS=0
    if(len(errorList)>0):
        for i in errorList:
            RMS+=np.power(i,2)
        RMS=np.sqrt(RMS/len(errorList))
    return RMS

