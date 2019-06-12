import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)

def out(pin):
	print("PIN %d OUT" % pin)
	GPIO.setup(pin, GPIO.OUT, 0)

out(17)

pwm = GPIO.