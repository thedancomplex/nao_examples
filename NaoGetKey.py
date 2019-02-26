from naoqi import ALProxy
PORT = 9559
robotIp = "nao.local"

#getting keypress name and ask NAO to do sth
def NaoDirection(keyname): 
	# just remember NAO is standing in front of participants, (mirror)
	 
	motionProxy = ALProxy("ALMotion", robotIP, PORT)
        if keyname == "R":
            # here write codes for NAO to turn his head to the left and get back to the center
        elif keyname == "L":
            # here write codes for NAO to turn his head to the right and get back to the center
