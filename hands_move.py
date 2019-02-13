robotIP = "nao.local"
PORT = 9559

import sys
from naoqi import ALProxy
import time
motionProxy = ALProxy("ALMotion", robotIP , PORT)

while(1):
  motionProxy.openHand('RHand')
  motionProxy.closeHand('RHand')
  time.sleep(1.0)
  motionProxy.openHand('LHand')
  motionProxy.closeHand('LHand')
  time.sleep(1.0)
