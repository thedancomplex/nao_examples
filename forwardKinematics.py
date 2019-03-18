import numpy
import math
#import mds_ik

thetaZero=0.0
thetaOne=0
thetaTwo=0.0
thetaThree=0
thetaFour=0.0

l0=1
l1=1
l2=1
l3=1

UpperArmLength  =105.0
ElbowOffsetY    =15.0
LowerArmLength  =55.95
HandOffsetX 	=57.75
ShoulderOffsetY =98.0
HandOffsetZ     =12.31

# lengths is defined as an array [x,y,z] lengths for the translation matrix
def createTransformation(lengths,theta,axis):
	# different rotation matrices for each axis
	if(axis=='z'):
		Rtemp = numpy.array([[math.cos(math.radians(theta)),-math.sin(math.radians(theta)),0,0],
		[math.sin(math.radians(theta)),math.cos(math.radians(theta)),0,0],
		[0,0,1,0],
		[0,0,0,1]])
		Ptemp = numpy.array([[1,0,0,lengths[0]],[0,1,0,lengths[1]],[0,0,1,lengths[2]],[0,0,0,1]])
	elif(axis=='y'):
		Rtemp = numpy.array([[math.cos(math.radians(theta)),0,math.sin(math.radians(theta)),0],
		[0,1,0,0],
		[-math.sin(math.radians(theta)),0,math.cos(math.radians(theta)),0],
		[0,0,0,1]])
		Ptemp = numpy.array([[1,0,0,lengths[0]],[0,1,0,lengths[1]],[0,0,1,lengths[2]],[0,0,0,1]])
	elif(axis=='x'):
		Rtemp = numpy.array([[1,0,0,0],
		[0,math.cos(math.radians(theta)),-math.sin(math.radians(theta)),0],
		[0,math.sin(math.radians(theta)),math.cos(math.radians(theta)),0],
		[0,0,0,1]])
		Ptemp = numpy.array([[1,0,0,lengths[0]],[0,1,0,lengths[1]],[0,0,1,lengths[2]],[0,0,0,1]])

	return numpy.matmul(Rtemp,Ptemp)

# assumes: listOTs is order starting with Translation matrix 0:1 up to Translation matrix n-1:n
def matMullTransformation(listOTs):
	# takes in list of transformation matrices and computes the Cumulative transformation matrix
	TRes = []
	for a in range(len(listOTs)-1):
		if(a == 0):
			# need this first case to give TRes a first value to use
			TRes = numpy.dot(listOTs[a],listOTs[a+1])
		else:
			TRes = numpy.dot(TRes,listOTs[a+1])
	return TRes

# creates the transforms for either the Right Arm or the Left Arm
def createTransforms(targetAngles,body):
	temp = []
	if(body == "RArm"):
		# create base to shoulder transformation
		#RightShoulderPitch
		TRSP = createTransformation([0,-ShoulderOffsetY,0],targetAngles[0], axis='y')
		#RightShoulderRoll
		TRSR = createTransformation([0,0,0],targetAngles[1], axis='x')
		#RElbowYaw
		TREY = createTransformation([0,0,-UpperArmLength],targetAngles[2], axis='z')
		#RElbowRoll
		TRER = createTransformation([0,0,0],targetAngles[3],axis='x')
		#RWristYaw
		TRWY = createTransformation([0,0,-LowerArmLength],targetAngles[4],axis='z')
		#Transform from writst to hand(EndEffector)
		TWH = createTransformation([0.0,0,-HandOffsetX],0,axis='x')
		res = matMullTransformation([TRSP,TRSR,TREY,TRER,TRWY,TWH])
		#print(TWH)
		print("End Effector X:",res[0,3])# x
		print("End Effector Y:",(res[1,3]))# y
		print("End Effector Z:",res[2,3])# z
		return res

	elif(body=="LArm"):
		# create base to shoulder transformation
		#LeftShoulderPitch
		TLSP = createTransformation([0,ShoulderOffsetY,0],targetAngles[0], axis='y')
		#LeftShoulderRoll
		TLSR = createTransformation([0,0,0],targetAngles[1], axis='x')
		#LElbowYaw
		TLEY = createTransformation([0.0,0.0,-UpperArmLength],targetAngles[2], axis='x')
		#LElbowRoll
		TLER = createTransformation([0,0,0],targetAngles[3],axis='x')
		#LWristYaw
		TLWY = createTransformation([0.0,0,-LowerArmLength],targetAngles[4],axis='z')
		#Transform from writst to hand(EndEffector)
		TWH = createTransformation([0.0,0,-HandOffsetX],0,'x')

		res = matMullTransformation([TLSP,TLSR,TLEY,TLER,TLWY,TWH])
		#print(res)
		print("End Effector X:",res[0,3])# x
		print("End Effector Y:",(res[1,3]))# y
		print("End Effector Z:",res[2,3])# z
		return res

createTransforms([45,10,0,0,0,0],"LArm")
