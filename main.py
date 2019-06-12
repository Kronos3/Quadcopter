#!/usr/bin/env python

from PID import PIDControl
from gyro import Gyro

def main(args):
	gyro = Gyro(0x68)
	
	# Rotational Ks
	rkp = 0
	rki = 0
	rkd = 0
	
	# Translation Ks
	tkp = 0
	tki = 0
	tkd = 0
	
	mc = PIDControl((rkp, rki, rkd), (tkp, tki, tkd), gyro, (2,3,4,5))
	
	mc.desired_height = 0.5
	
	mc.start()
	
	return 0

if __name__ == '__main__':
	import sys
	sys.exit(main(sys.argv))
