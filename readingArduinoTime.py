import serial
import syslog
import time
import struct
import os
from datetime import datetime
import sys
import numpy as np
import pickle as pickleRick

leftB = np.array([])
rightB= np.array([])
port = "/dev/ttyACM1"
record_location = str(os.getcwd())+"/logs"
print '---------'
if (len(sys.argv) > 1):
  port = sys.argv[1]
  print sys.argv[1]
if (len(sys.argv) > 2):
  record_location = sys.argv[2]
  print record_location
print '---------'
ard = serial.Serial(port,115200,timeout=5)
startTime = time.time()
#ard.flush()
record_location += "/"
f = open(record_location+str(datetime.now().isoformat())+".log","w")
f.write(str(time.time())+"\r\n")
os.system('./recordAudio.sh -n '+record_location+str(datetime.now().isoformat())+ '& echo $! > /tmp/pid')

timeout = 0

timeDiff = 0

tDiff = 0

played1 = False # use these variables to play the tones only once

played2 = False

played3 = False

def cleanUp():
        print("clean up")
	pickleRick.dump(leftB,open(record_location+str(datetime.now().isoformat())+"Left.dat","w"))
	
	pickleRick.dump(rightB,open(record_location+str(datetime.now().isoformat())+"Right.dat","w"))

        f = open("/tmp/pid","r")

        pid = int(f.readline())
        pid2 = pid + 1

        os.system("kill -9 " + str(pid))
        os.system("kill -9 " + str(pid2))
	ard.close()

def unpackULong(msg,negate):
	global tDiff
	res = 0
	msg = msg[1:len(msg)]
	countD = len(msg)
	countU = 0
	res = (float(int(msg,16))/1000000) # convert to int then to float then to seconds from micro
	if(tDiff==1):
		if(negate):	
			print -res
		else:
			print res
	return res

try:
	ard.flush()
	while(True):
		if(time.time() > (startTime + 20) and played1 == False):
			os.system("aplay beep-06.wav &")
			played1 = True # so tone doesn't play every loop
		if(time.time() > (startTime + 40) and played2 == False):
			os.system("aplay beep-06.wav &")
			played2 = True # so tone doesn't play every loop
		if(time.time() > (startTime + 60) and played3 == False):
			os.system("aplay beep-06.wav &")
			played3 = True
		if(ard.inWaiting()):
			msg = (ard.read(3))
			
			buf = struct.unpack('>B', msg[0])
			buf2 = struct.unpack('>B', msg[1])
			buf3 = struct.unpack('>B', msg[2])
			if(buf[0] == 255 and buf2[0]==255):
                		if(buf3[0] == 2):
					# this is the small button at the front
					print("breaking out")
					break
				if(buf3[0] == 3):
                    			tDiff = 1
                		else:
                    			tDiff = 0
				if(buf3[0] == 0):
					ard.read(1)
					curT = unpackULong(ard.read(8),0)
					print(curT)
					f.write("H " + str(curT) + " right"+"\r\n")
					rightB = np.append(rightB,curT)
				elif(buf3[0] == 1):
					ard.read(1)
					curT = unpackULong(ard.read(8),0)
					print(curT)
					f.write("H "+str(curT) + " left"+"\r\n")
					leftB = np.append(leftB,curT)
				elif(buf3[0] == 5):
					timeout = 1
					buf4 = 0
				elif(buf3[0] == 7):# this case will no longer be used, but will keep
                                        f.write(str(time.time())+" zero"+"\r\n")
					leftB = np.append(leftB,0)
					rightB = np.append(rightB,0)

				elif(buf3[0] == 3):
					buf4 = struct.unpack('>B', ard.read(1))[0]
					msg = ard.read(8)
					timeDiff = unpackULong(msg,buf4)

					if(buf4 == 1):
						f.write("D -"+str(timeDiff)+" diff "+"\r\n")
					elif(buf4 == 0):
						f.write("D "+str(timeDiff)+" diff "+"\r\n")

					timeout = 0
			else:
				ard.close()
				ard = serial.Serial(port,115200,timeout=5)
		time.sleep(.001)
	cleanUp()

except any:#KeyboardInterrupt:
	cleanUp()
