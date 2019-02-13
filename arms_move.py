robotIP = "nao.local"
PORT = 9559

import sys
from naoqi import ALProxy
import time
import math
motionProxy  = ALProxy("ALMotion", robotIP , PORT)
postureProxy = ALProxy("ALRobotPosture", robotIP, PORT)

def StiffnessOn(proxy):
  # Make robot stop moving 
  # We use the "Body" name to signify the collection of all joints
  pNames = "Body"
  pStiffnessLists = 1.0
  pTimeLists = 1.0
  proxy.stiffnessInterpolation(pNames, pStiffnessLists, pTimeLists)



# Turn on our stiffness
StiffnessOn(motionProxy)

# Goes to stand up postion
postureProxy.goToPosture("StandInit", 0.5)
time.sleep(10.0)

# Go to stand 
postureProxy.goToPosture("Stand", 0.5)
time.sleep(10.0)

