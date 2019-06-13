import RPi.GPIO as GPIO
from PID import Motor

GPIO.setmode(GPIO.BCM)

PWM_PIN = 17
ENABLE_PIN = 4

GPIO.setup(ENABLE_PIN, GPIO.OUT)
GPIO.output(ENABLE_PIN, True)

test = Motor(pwm_pin=PWM_PIN, mode_scale=None)

while True:
	try:
		test.set_speed(int(input("dc: ")))
	except KeyboardInterrupt:
		break

test.stop()
GPIO.cleanup()