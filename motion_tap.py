# -*- encoding: UTF-8 -*- 

''' PoseZero: Set all the motors of the body to zero. '''

import sys
from naoqi import ALProxy
import time
def StiffnessOn(proxy):
    # We use the "Body" name to signify the collection of all joints
    pNames = "Body"
    pStiffnessLists = 1.0
    pTimeLists = 1.0
    proxy.stiffnessInterpolation(pNames, pStiffnessLists, pTimeLists)


def main(robotIP):
    # Init proxies.
    try:
        motionProxy = ALProxy("ALMotion", robotIP, 9559)
    except Exception, e:
        print "Could not create proxy to ALRobotPosture"
        print "Error was: ", e

    try:
        postureProxy = ALProxy("ALRobotPosture", robotIP, 9559)
    except Exception, e:
        print "Could not create proxy to ALRobotPosture"
        print "Error was: ", e

    # Set NAO in Stiffness On
    StiffnessOn(motionProxy)

    # Send NAO to Pose Init
    #postureProxy.goToPosture("Stand", 0.5)
    postureProxy.goToPosture("StandZero", 0.5)
    time.sleep(1.0)

    # We use the "Body" name to signify the collection of all joints and actuators
    pNames = "Body"

    # Get the Number of Joints
    numBodies = len(motionProxy.getBodyNames(pNames))
    print motionProxy.getBodyNames(pNames)

    # We prepare a collection of floats
    pTargetAngles = [0.0] * numBodies
    pTargetAngles[2] = 1.57
    pTargetAngles[20] = 1.57
    # We set the fraction of max speed
    pMaxSpeedFraction = 0.3

    print pNames
    # Ask motion to do this with a blocking call
    motionProxy.angleInterpolationWithSpeed(pNames, pTargetAngles, pMaxSpeedFraction)

    delta = 0.0
    pMaxSpeedFraction = 0.5
    while True:
      if delta > 0.0: 
        delta = 0.0
      else:
        delta = 0.2
      pTargetAngles[20] = 1.57 - delta
      motionProxy.angleInterpolationWithSpeed(pNames, pTargetAngles, pMaxSpeedFraction)
      print delta
      time.sleep(0.5)


if __name__ == "__main__":
    robotIp = "127.0.0.1"

    if len(sys.argv) <= 1:
        print "Usage python motion_poseZero.py robotIP (optional default: 127.0.0.1)"
    else:
        robotIp = sys.argv[1]

    main(robotIp)
