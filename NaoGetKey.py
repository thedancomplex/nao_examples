from naoqi import ALProxy


#getting keypress name and ask NAO to do sth
def NaoDirection(keyname): 
	# just remember NAO is standing in front of participants, (mirror)
	#PORT = 9559
	#robotIp = "nao.local"
	motionProxy = ALProxy("ALMotion", "nao.local", 9559)
	names = "HeadYaw"

        if keyname == "R":
            # here write codes for NAO to turn his head to the left and get back to the center
	     targetAngles = 1.0
        elif keyname == "L":
            # here write codes for NAO to turn his head to the right and get back to the center
	     targetAngles = -1.0

	maxSpeedFraction  = 0.2 # Using 20% of maximum joint speed
	motionProxy.angleInterpolationWithSpeed(names, targetAngles, maxSpeedFraction)
	#Send NAO to Pose Init
	postureProxy = ALProxy("ALRobotPosture", "nao.local", 9559)
	postureProxy.goToPosture("Stand", 0.5)
