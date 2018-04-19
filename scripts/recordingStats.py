#!/usr/bin/env python
import os
import cv2
import sys

import time

from datetime import datetime 

import matplotlib.pyplot as plt

inputFile=sys.argv[1]
difference=[]
with open(inputFile, "r") as ins:
    ins.next()
    ins.next()
    for line in ins:
        msg=line.split(",")
        print(msg)
        try:
          difference.append(msg[1])
        except:
          pass
ins.close()

plt.plot(difference[1:-1])

plt.show()

