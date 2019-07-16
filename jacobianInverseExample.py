import numpy
import math
import copy
import forwardKinematics

thetaOld = numpy.zeros(5)
OG = numpy.array([154.6,98,-25])
desiredPoint = numpy.array([154.6,98,-25]) # mmeters
jointSpaceStart = numpy.array([20,20,20,20,0])
print("desired: ",desiredPoint)
b=0.0 # this will need to change base on the new values when iterating

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
			TRes = numpy.matmul(listOTs[a],listOTs[a+1])
		else:
			TRes = numpy.matmul(TRes,listOTs[a+1])
	return TRes

def checkArmJoints(Arm_t,jointList):
	temp = jointList
	if(Arm_t == "RArm"):
		if(jointList[0] < -119.5):
			temp[0] = -119.5
		elif(jointList[0] > 119.5):
			temp[0] = 119.5
		if(jointList[1] > 18):
			temp[1] = 18
		elif(jointList[1] < -76):
			temp[1] = -76
		if(jointList[2] < -119.5):
			temp[2] = -119.5
		elif(jointList[2] > 119.5):
			temp[2] = 119.5
		if(jointList[3] < 2):
			jointList[3] = 2
		elif(jointList[3] > 88.5):
			jointList[3] = 88.5
		if(jointList[4] > 104.5):
			jointList[4] = 104.5
		elif(jointList[4] < -104.5):
			jointList[4] = -104.5
	elif(Arm_t == "LArm"):
		if(jointList[0] < -119.5):
			temp[0] = -119.5
		elif(jointList[0] > 119.5):
			temp[0] = 119.5
		if(jointList[1] > 76):
			temp[1] = 76
		elif(jointList[1] < -18):
			temp[1] = -18
		if(jointList[2] < -119.5):
			temp[2] = -119.5
		elif(jointList[2] > 119.5):
			temp[2] = 119.5
		if(jointList[3] < -88.5):
			jointList[3] = -88.5
		elif(jointList[3] > -2):
			jointList[3] = -2
		if(jointList[4] > 104.5):
			jointList[4] = 104.5
		elif(jointList[4] < -104.5):
			jointList[4] = -104.5
	return temp

def createJacobian(Arm_t,di):
	# need a 5x3 for nao arms
	dTheta = .001
	J = numpy.zeros((3,5))

	for a in range(3):
		for b in range(5):
			diPrime = copy.deepcopy(di)
			diPrime[b] = di[b] + dTheta
			A = forwardKinematics.createTransforms(di,Arm_t)
			APrime = forwardKinematics.createTransforms(diPrime,Arm_t)			
			J[a,b] = (APrime[a,3] - A[a,3])/dTheta
	return J

Arm_t = "LArm"
start = forwardKinematics.createTransforms(jointSpaceStart,Arm_t)
startingPoint = numpy.zeros(3)
startingPoint[0] = start[0,3]
startingPoint[1] = start[1,3]
startingPoint[2] = start[2,3]
J = createJacobian(Arm_t,jointSpaceStart)
Jinverse = numpy.linalg.pinv(J)
#print Jinverse
print(startingPoint)
currentPoint = startingPoint
thetaOld = jointSpaceStart
found  = 0
firstJointSet = None
for a in range(10000):
	J = createJacobian(Arm_t,thetaOld)
	print("joints",thetaOld)
	Jinverse = numpy.linalg.pinv(J)
	offset = desiredPoint - currentPoint;
	distanceOffset = numpy.sqrt((offset[0]*offset[0])+(offset[1]*offset[1])+(offset[2]*offset[2]))
	DEffector = (offset/distanceOffset) * .1
	#print(DEffector)

	dThetaJ = numpy.matmul(Jinverse,DEffector)
	#print "change in theta", dThetaJ
	thetaNew = thetaOld + dThetaJ
	thetaNew2 = checkArmJoints(Arm_t,thetaNew)
	#print thetaNew
	
	A = forwardKinematics.createTransforms(thetaNew2,Arm_t)
	
	newP = numpy.zeros(3)
	newP[0] = A[0,3]
	newP[1] = A[1,3]
	newP[2] = A[2,3]
	temp = desiredPoint - newP

	distanceOffset = numpy.sqrt(math.pow(temp[0],2) + math.pow(temp[1],2) + math.pow(temp[2],2))
	print "new points",newP
	print "distance offset",distanceOffset
	if(distanceOffset < 1 and found == 0):
		print "found it at: ",a
		firstJointSet = thetaNew
		print(thetaNew)
		desiredPoint = OG + numpy.array([0,0,-50])
		found += 1
	elif(distanceOffset<1 and found ==1):
		print "found it at: ",a
		print "found for ",Arm_t
		print(thetaNew)
		print(firstJointSet)
		print(desiredPoint)
		found += 1
		break;
	currentPoint = newP
	thetaOld = thetaNew2
