import serial
import syslog
import time
import struct
import os
from datetime import datetime
import sys

port = "/dev/ttyACM1"
record_location = os.system('pwd')
print '---------'
if (len(sys.argv) > 1):
  port = sys.argv[1]
  print sys.argv[1]
if (len(sys.argv) > 2):
  record_location = sys.argv[2]
  print record_location
print '---------'
ard = serial.Serial(port,115200,timeout=5)

ard.flush()
record_location += "/"
f = open(record_location+str(datetime.now().isoformat())+".log","w")
f.write(str(time.time())+"\r\n")
os.system('./recordAudio.sh -n '+record_location+str(datetime.now().isoformat())+ '& echo $! > /tmp/pid')

timeout = 0

timeDiff = 0

tDiff = 0

def unpackULong(msg,neg):
	global tDiff
	res = 0
	#print int(msg[0])
	msg = msg[1:len(msg)]
	countD = len(msg)
	countU = 0
	res = (float(int(msg,16))/1000000) # convert to int then to float then to seconds from micro
	if(tDiff==1):
		if(neg):	
			print -res
		else:
			print res
	return res

try:
	while(True):
		if(ard.inWaiting()):
			msg = (ard.read(3))
			buf = struct.unpack('>B', msg[0])
			buf2 = struct.unpack('>B', msg[1])
			buf3 = struct.unpack('>B', msg[2])
			if(buf[0] == 255 and buf2[0]==255):
				#print("ID",buf3[0]) # 0 for N button 1 for L button and 3 for diff, 2 for top, 5 for timeout
                		if(buf3[0] == 3):
                    			tDiff = 1
                		else:
                    			tDiff = 0
				if(buf3[0] == 5):
					#print("tapping timeout")
                    			print("9999999")
					timeout = 1
					buf4 = 0
				else:
					buf4 = struct.unpack('>B', ard.read(1))[0]
					#print("negative phase",buf4)
					msg = ard.read(8)
					timeDiff = unpackULong(msg,buf4)

				if(buf3[0] == 3):
					if(buf4 == 1):
						f.write(str(time.time())+" -"+str(timeDiff)+" "+str(timeout)+"\r\n")
					elif(buf4 == 0):
						f.write(str(time.time())+" "+str(timeDiff)+" "+str(timeout)+"\r\n")

					timeout = 0
		time.sleep(.001)

except any:#KeyboardInterrupt:

	# need to catch so that the audio recording is killed

	f = open("/tmp/pid","r")

	pid = int(f.readline())
	pid2 = pid + 1

	os.system("kill -9 " + str(pid))
	os.system("kill -9 " + str(pid2))
