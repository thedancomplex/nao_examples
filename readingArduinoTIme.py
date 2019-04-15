import serial
import syslog
import time
import struct

port = "/dev/ttyACM1"

ard = serial.Serial(port,115200,timeout=5)

ard.flush()

def unpackULong(msg):
	res = 0
	print int(msg[0])
	msg = msg[1:len(msg)]
	countD = len(msg)
	countU = 0
	print(float(int(msg,16))/1000000)# convert to int then to float then to seconds from micro
	

while(True):
	if(ard.inWaiting()):
		msg = (ard.read(3))
		hi = struct.unpack('>B', msg[0])
		hi2 = struct.unpack('>B', msg[1])
		hi3 = struct.unpack('>B', msg[2])
		if(hi[0] == 255 and hi2[0]==255):
			print("ID",hi3[0]) # 0 for N button 1 for L button and 3 for diff, 2 for top
			hi4 = struct.unpack('>B', ard.read(1))
			print("negative",hi4)
			msg = ard.read(8)
			

			unpackULong(msg)
		#print((msg[1]))
	time.sleep(.001)
