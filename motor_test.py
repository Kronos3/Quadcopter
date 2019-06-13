import RPi.GPIO as GPIO
from PID import Motor

GPIO.setmode(GPIO.BCM)

PWM_PIN = 17
test = Motor(pwm_pin=PWM_PIN, mode_scale=None)

while True:
	try:
		test.set_speed(int(input("dc: ")))
	except KeyboardInterrupt:
		break

test.stop()
GPIO.cleanup()