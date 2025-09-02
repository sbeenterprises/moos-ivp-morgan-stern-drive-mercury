import serial


portname = '/dev/serial0'
baudrate = 115200

def main():
	# open serial port
	port = serial.Serial(portname, baudrate, timeout=1, write_timeout=1)
	
	# read 10 times
	for i in range(10):
		print(port.readline())
	
	# pause async outputs
	print('$VNASY,0*XX\r\n'.encode())
	port.write('$VNASY,0*XX\r\n'.encode())
	
	# read 5 times (should timeout each time)
	for i in range(5):
		print(port.readline())
	
	# resume async outputs
	port.write('$VNASY,1*XX\r\n'.encode())
	
	# read 10 times
	for i in range(10):
		print(port.readline())
	port.close()

if __name__ == "__main__":
	main()