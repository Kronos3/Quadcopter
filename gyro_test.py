from gyro import Gyro
import sys

test = Gyro(0x68)
index = int(sys.argv[1])

while True:
	data = test.pid_read()
	sys.stdout.write("\r%s" % data[index])
