# -*- encoding: UTF-8 -*- 

''' PoseZero: Set all the motors of the body to zero. '''

import sys
from naoqi import ALProxy
import time
import math
import forwardKinematics 
import qi

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
	session = qi.Session()
	session.connect("tcp://" + robotIP + ":" + str(9559))
        memory_service = session.service("ALMemory")
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
    #postureProxy.goToPosture("StandZero", 0.5)
    #time.sleep(2.0)

    # We use the "Body" name to signify the collection of all joints and actuators
    pRArm = "RArm"
    pLArm = "LArm"

    #[ 52.98565192 -26.52871757  50.91683225  73.35633152   0.        ]
    pTargetAngles0R = [ (90.0-79.65702954),-12.21749182,55.86600865, 40.51705609, 0 ]
    pTargetAngles1R = [ (90.0-60.95316587),-29.30072426,57.89848256,88.5,0]
    pTargetAngles0Rfk = [ (79.65702954),-12.21749182,55.86600865, 40.51705609, 0 ]
    pTargetAngles1Rfk = [ ( 60.95316587),-29.30072426,57.89848256,88.5,0]
    pTargetAnglesR = pTargetAngles0R

    pTargetAngles0L = [ (90.0-79.65702954),12.21749182,-55.86600865, -40.51705609, 0 ] 
    #[ (90-96.99678999),10.55418621,10.55418621,-40.52489326, 0 ]
    pTargetAngles1L = [ (90.0-60.95316587),29.30072426,-57.89848256,-88.5,0]
    #[ (90-99.08780408),23.39613756,23.39613756,-88.5,0  ]
    pTargetAngles0Lfk = [ (96.99678999),10.55418621,10.55418621,-40.52489326, 0 ]
    pTargetAngles1Lfk = [ (99.08780408),23.39613756,23.39613756,-88.5,0  ]

    pTargetAnglesL = pTargetAngles0L
    pTargetAnglesRadR = [0.0] * 6
    pTargetAnglesRadL = [0.0] * 6
    csvTiming = open("testTiming.csv","w")
    startingTime = time.time()
    count = 0
    while(True):
	    #pTargetAnglesPrime = [ 79.65702954,-12.21749182,55.86600865, 40.51705609,0]
	    if(count==10):
		if(pTargetAngles0R==pTargetAnglesR):
			pTargetAnglesR = pTargetAngles1R
			pTargetAnglesL = pTargetAngles1L
		        resultingT = forwardKinematics.createTransforms(pTargetAngles1Lfk,pLArm)
			#print("End Effector X FK:",resultingT[0,3])# x
			#print("End Effector Y FK:",(resultingT[1,3]))# y
			#print("End Effector Z FK:",resultingT[2,3])# z
		        resultingT = forwardKinematics.createTransforms(pTargetAngles1Rfk,pRArm)
			#print("End Effector X FK:",resultingT[0,3])# x
			#print("End Effector Y FK:",(resultingT[1,3]))# y
			#print("End Effector Z FK:",resultingT[2,3])# z
		else:
			pTargetAnglesR = pTargetAngles0R
			pTargetAnglesL = pTargetAngles0L	
		        resultingT = forwardKinematics.createTransforms(pTargetAngles0Lfk,pLArm)
			#print("End Effector X FK:",resultingT[0,3])# x
			#print("End Effector Y FK:",(resultingT[1,3]))# y
			#print("End Effector Z FK:",resultingT[2,3])# z
		        resultingT = forwardKinematics.createTransforms(pTargetAngles0Rfk,pRArm)
			#print("End Effector X FK:",resultingT[0,3])# x
			#print("End Effector Y FK:",(resultingT[1,3]))# y
			#print("End Effector Z FK:",resultingT[2,3])# z
        	batPerc = memory_service.getData("Device/SubDeviceList/Battery/Charge/Sensor/Value")
        	#print("battery%",batPerc)
		count = 0 
		csvTiming.write(str(time.time()-startingTime)+","+str(batPerc) + "\n")		
		count = 0
		
	    """pTargetAngles[0] = 52.98565192
	    pTargetAngles[1] = -26.52871757
	    pTargetAngles[2] = 50.91683225
	    pTargetAngles[3] = 73.35633152"""

	    # We set the fraction of max speed
	    pMaxSpeedFraction = 0.3

	    for a in range(len(pTargetAnglesR)):
		pTargetAnglesRadR[a] = math.radians(pTargetAnglesR[a])
		pTargetAnglesRadL[a] = math.radians(pTargetAnglesL[a])

	    #print "Angle values assigned in radians",pTargetAnglesRad

	    # Ask motion to do this with a blocking call

	    motionProxy.setAngles(pRArm, pTargetAnglesRadR, pMaxSpeedFraction)
	    motionProxy.setAngles(pLArm, pTargetAnglesRadL, pMaxSpeedFraction)

	    encoderAngles = motionProxy.getAngles(pRArm,True)
	    #print("Right arm encoders: ",encoderAngles)
	    encoderAngles = motionProxy.getAngles(pLArm,True)
	    #print("Left Arm encoders: ",encoderAngles)
	    #[ 46.09250502 -33.40408134  51.14905024  88.5          0.        ]
	    count += 1
	    time.sleep(.2)
	    #pTargetAnglesPrime = [ (60.95316587),-29.30072426,57.89848256,88.5,0]


    #print(encoderAngles)

    #rotationMatrixExample.createTransforms(encoderAngles,pNames)
    time.sleep(3.0)

if __name__ == "__main__":
    robotIp = "127.0.0.1"

    if len(sys.argv) <= 1:
        print "Usage python motion_poseZero.py robotIP (optional default: 127.0.0.1)"
    else:
        robotIp = sys.argv[1]

    main(robotIp)
