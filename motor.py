import RPi.GPIO as GPIO
from clock import BaseClock

class MotorControl(BaseClock):
	motors = []
	enable_pin = -1
	gyro = None
	
	def __init__(self, gyro, enable_pin, motor_pins):
		super(MotorControl, self).__init__(tick_hz=6)
		
		self.motors = []
		self.enable_pin = enable_pin
		self.gyro = gyro
		
		
		print("ENABLE pin_%d" % self.enable_pin)
		GPIO.setmode(GPIO.BOARD)
		GPIO.setup(self.enable_pin, GPIO.OUT)
		GPIO.output(self.enable_pin, 1)
		
		for pin in motor_pins:
			motor = Motor(pin)
			motor.start()
			self.motors.append(motor)
	
	def read(self):
		return (self.gyro.get_gyro_data(), self.gyro.get_accel_data())
	
	def tick(self, dt):
		print("dt = %f" % dt)
		
		gyro, accel = self.read()
		

class Motor:
	pwm_pin = -1
	pwm = None
	frequency = -1
	current_speed = -1
	
	def __init__(self, x_scale, y_scale, pwm_pin, frequency=10000):
		self.pwm_pin = pwm_pin
		self.current_speed = 0
		self.frequency = frequency
		
		print("PWM pin_%d (%d kHz)" % (self.pwm_pin, self.frequency/1000))
		
		GPIO.setup(self.pwm_pin, GPIO.out)
		self.pwm = GPIO.PWM(self.pwm_pin, frequency)
	
	def start(self, duty_cycle=0):
		self.current_speed = duty_cycle
		self.pwm.start(duty_cycle)
	
	def stop(self):
		self.pwm.stop()
		pass
	
	def set_speed(self, duty_cycle):
		self.current_speed = duty_cycle
		self.pwm.ChangeDutyCycle(self.current_speed)
	
	def angle(self, x_angle, y_angle):
		scale = 
		
		self.current_speed *= scale
		self.pwm.ChangeDutyCycle(self.current_speed)
