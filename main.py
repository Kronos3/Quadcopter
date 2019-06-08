#!/usr/bin/env python

from motor import MotorControl
from gyro import Gyro

def main(args):
	gyro = Gyro(0x68)
	
	mc = MotorControl(gyro, 1, [2,3,4,5])
	mc.start()
	
	
	return 0

if __name__ == '__main__':
	import sys
	sys.exit(main(sys.argv))
