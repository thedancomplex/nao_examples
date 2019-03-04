import numpy
import math

l0,l1,l2 = 1.5,1.2,1.5
theta0,theta1,theta2=0,10,80 # starting theta
thetaOld = numpy.array([theta0,theta1])
startingPoint = 0,0
desiredPoint = 1.3,1.8 # meters
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

joint0 = createTransformation([l0,0,0],theta0,'z')
joint1 = createTransformation([l1,0,0],theta1,'z')
#joint2 = createTransformation([l2,0,0],theta2,'z')
base = matMullTransformation([joint0,joint1])
print base
startingPoint = base[0,3],base[1,3]
print startingPoint
def createJacobian():
	global joint0,joint1,joint2
	dTheta = .001 # degrees
	theta0Prime = theta0 + dTheta
	theta1Prime = theta1 + dTheta
	theta2Prime = theta2 + dTheta

	PPrime0 = createTransformation([l0,0,0],(theta0Prime),'z')
	PPrime1 = createTransformation([l1,0,0],theta1,'z')
	#PPrime2 = createTransformation([l2,0,0],theta2,'z')

	basePrime = matMullTransformation([PPrime0,PPrime1])
	#print("theta0Prime:")
	#print(basePrime)
	dexdTheta0 = (basePrime[0,3] - base[0,3])/dTheta
	deydTheta0 = (basePrime[1,3] - base[1,3])/dTheta

	PPrime0 = createTransformation([l0,0,0],theta0,'z')
	PPrime1 = createTransformation([l1,0,0],(theta1Prime),'z')
	#PPrime2 = createTransformation([l2,0,0],theta2,'z')

	basePrime = matMullTransformation([PPrime0,PPrime1])
	#print("theta1Prime:")
	#print(basePrime)
	dexdTheta1 = (basePrime[0,3] - base[0,3])/dTheta
	deydTheta1 = (basePrime[1,3] - base[1,3])/dTheta

	PPrime0 = createTransformation([l0,0,0],theta0,'z')
	PPrime1 = createTransformation([l1,0,0],theta1,'z')
	#PPrime2 = createTransformation([l2,0,0],(theta2Prime),'z')

	#basePrime = matMullTransformation([PPrime0,PPrime1,PPrime2])
	#print("theta2Prime:")
	#print(basePrime)
	#dexdTheta2 = (basePrime[0,3] - base[0,3])/dTheta
	#deydTheta2 = (basePrime[1,3] - base[1,3])/dTheta

	J = numpy.zeros((2,2))
	J = numpy.array([[dexdTheta0,dexdTheta1],[deydTheta0,deydTheta1]])

	#J = numpy.array([[dexdTheta0,dexdTheta1]#,dexdTheta2],
	#		[deydTheta0, deydTheta1]])#,deydTheta2]])

	#print(J)
	return J

J = createJacobian()
Jinverse = numpy.linalg.inv(J)
#print Jinverse
newPos = numpy.zeros(2)
newPos[0] = startingPoint[0]
newPos[1] = startingPoint[1]

print(newPos)
for a in range(4000):
	diffY,diffX = (desiredPoint[1]-newPos[1]),(desiredPoint[0]-newPos[0])
	slope=abs(diffY/diffX)
	print "slope",slope

	if(diffY > diffX):
		if(diffY >0):
			newY = .001
		else:
			newY = -.001
		if(diffX >0):
			newX = (newY-b)/(slope)
		else:
			newX = -(newY-b)/(slope)

	else:
		if(diffX >0):
			newX = .001
		else:
			newX = -.001
		if(diffY > 0):
			newY = (slope)*newX + b
		else:
			newY = -(slope)*newX + b

	#print newX, newY
	newPos[0] = newX# - newPos[0]
	newPos[1] = newY# - newPos[1]
	print newPos
	dThetaJ = numpy.matmul(Jinverse,newPos)
	#print "change in theta", dThetaJ
	thetaNew = thetaOld + dThetaJ
	#print thetaNew
	joint0 = createTransformation([l0,0,0],thetaNew[0],'z')
	joint1 = createTransformation([l1,0,0],thetaNew[1],'z')
	#joint2 = createTransformation([l2,0,0],theta2,'z')
	baseP = matMullTransformation([joint0,joint1])

	newP = numpy.zeros(2)
	newP[0] = baseP[0,3] # set the x
	newP[1] = baseP[1,3] # set the y
	print "new points",newP
	if(math.sqrt(math.pow(desiredPoint[0]-newP[0],2)+math.pow(desiredPoint[1]-newP[1],2))<.01):
		print "found it at: ",a
		print(thetaNew)
		break
	newPos = newP
	thetaOld = thetaNew
