import serial
import time

port = serial.Serial(port = '////', baudrate = 9600);

if (!port.isOpen)
	port.open();

port.write("Enter commands")
while True:	
	port.timeout = 2
	received = port.read(1)	# only takes one byte (one letter input)
	port.write("You sent: " + repr(received))
	

port.close()

