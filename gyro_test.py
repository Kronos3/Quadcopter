from gyro import Gyro
import sys
import time
from clock import BaseClock


index = int(sys.argv[1])

start_time = time.time()
i = 0

while True:
	try:
		i += 1
		data = test.pid_read()
		sys.stdout.write("\r%.4f" % data[index])
	except KeyboardInterrupt:
		break

total_time = time.time() - start_time
print("Hz = %s" % (i / total_time))
