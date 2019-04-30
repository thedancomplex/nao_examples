import serial
import syslog
import time
import struct
import os
import datetime
import sys

port = "/dev/ttyACM1"
record_location = os.system('pwd')
print '---------'
if (len(sys.argv) > 1):
  port = sys.argv[1]
  print sys.argv[1]
print '---------'
ard = serial.Serial(port,115200,timeout=5)

ard.flush()

f = open(str(datetime.datetime.now().isoformat())+".log","w")
f.write("test")

os.system('./recordAudio.sh -n '+record_location+str(datetime.datetime.now().isoformat())+ '& echo $! > /tmp/pid')

def unpackULong(msg):
	res = 0
	print int(msg[0])
	msg = msg[1:len(msg)]
	countD = len(msg)
	countU = 0
	print(float(int(msg,16))/1000000)# convert to int then to float then to seconds from micro

	
try:
	while(True):
		if(ard.inWaiting()):
			msg = (ard.read(3))
			hi = struct.unpack('>B', msg[0])
			hi2 = struct.unpack('>B', msg[1])
			hi3 = struct.unpack('>B', msg[2])
			if(hi[0] == 255 and hi2[0]==255):
				print("ID",hi3[0]) # 0 for N button 1 for L button and 3 for diff, 2 for top, 5 for timeout
				if(hi3[0] == 5):
					print("tapping timeout")
				else:
					hi4 = struct.unpack('>B', ard.read(1))
					print("negative phase ",hi4)
					msg = ard.read(8)
			
					unpackULong(msg)
		time.sleep(.001)

except KeyboardInterrupt:

	# need to catch so that this is executed

	f = open("/tmp/pid","r")

	pid = int(f.readline())
	pid2 = pid + 1

	os.system("kill -9 " + str(pid))
	os.system("kill -9 " + str(pid2))
	
