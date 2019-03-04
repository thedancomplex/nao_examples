import sys
from naoqi import ALProxy
import time
import math

class _GetchUnix:
    def __init__(self):
        import tty, sys

    def __call__(self):
        import sys, tty, termios
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

def StiffnessOn(proxy):
    # We use the "Body" name to signify the collection of all joints
    pNames = "Body"
    pStiffnessLists = 1.0
    pTimeLists = 1.0
    proxy.stiffnessInterpolation(pNames, pStiffnessLists, pTimeLists)

def main(robotIP):
    # Init proxies.
    charGetter = _GetchUnix()
    charGetter.__init__()
    pMaxSpeedFraction=0.3
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
    postureProxy.goToPosture("Stand", 0.5)# 50% speed
    time.sleep(2.0)
    angle = 45
    while(True):
	    t = charGetter.__call__()
	    if(t=='q'):
		break
	    angle = angle*-1
	    pTargetAnglesRad = [math.radians(angle)]

	    motionProxy.setAngles("HeadYaw", pTargetAnglesRad, pMaxSpeedFraction)
	    time.sleep(0.5)

    pTargetAnglesRad = [math.radians(0.0)]
    motionProxy.setAngles("HeadYaw", pTargetAnglesRad, pMaxSpeedFraction)
    time.sleep(1.0)

if __name__ == "__main__":
    robotIp = "127.0.0.1"

    if len(sys.argv) <= 1:
        print "Usage python motion_poseZero.py robotIP (optional default: 127.0.0.1)"
    else:
        robotIp = sys.argv[1]

    main(robotIp)
