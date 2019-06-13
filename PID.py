from collections import namedtuple
from clock import BaseClock
import RPi.GPIO as GPIO

PID_mode = namedtuple("PID_mode", "pitch roll yaw")
PID_scale = namedtuple("PID_scale", "pitch_c roll_c yaw_c")


class PIDControl(BaseClock):
	"""
	Proportional Integral Derivative Control
	"""
	
	def __init__(self, k_rot, k_trans, gyro, motor_pins):
		PWM.setup()
		
		self.gyro = gyro
		self.modes = {
			"stabilize": PID_mode(0, 0, 0),
			"ascend": PID_mode(0, 0, 0),
			"descend": PID_mode(0, 0, 0)
		}
		
		self.current_mode = self.modes["stabilize"]
		
		self.eulerZ = 0
		self.euler_velZ = 0
		
		self.pid_pitch = PID(*k_rot)
		self.pid_roll = PID(*k_rot)
		self.pid_yaw = PID(*k_rot)
		self.pid_lift = PID(*k_trans)
		
		self.throttle = 0
		self.desired_height = -1
		
		self.motor_1 = Motor(motor_pins[0], PID_scale(1, 1, -1))
		self.motor_2 = Motor(motor_pins[1], PID_scale(1, -1, 1))
		self.motor_3 = Motor(motor_pins[2], PID_scale(-1, -1, -1))
		self.motor_4 = Motor(motor_pins[4], PID_scale(-1, 1, 1))
	
	def set_mode(self, mode):
		self.current_mode = self.modes[mode]
	
	def tick(self, dt):
		gyro = self.gyro.get_gyro_data()
		z_accel = self.gyro.get_z_accel()
		
		pid_pitch = self.pid_pitch.get(dt, gyro[0], self.current_mode.pitch)
		pid_roll = self.pid_roll.get(dt, gyro[1], self.current_mode.roll)
		pid_yaw = self.pid_yaw.get(dt, gyro[2], self.current_mode.yaw)
		
		self.eulerZ += self.euler_velZ * dt
		self.euler_velZ += z_accel * dt
		
		if self.desired_height != -1:
			self.throttle = self.pid_lift.get(dt, self.eulerZ, self.desired_height)
		
		self.motor_1.tick(self.throttle, pid_pitch, pid_roll, pid_yaw)
		self.motor_2.tick(self.throttle, pid_pitch, pid_roll, pid_yaw)
		self.motor_3.tick(self.throttle, pid_pitch, pid_roll, pid_yaw)
		self.motor_4.tick(self.throttle, pid_pitch, pid_roll, pid_yaw)


class PID:
	def __init__(self, Kp, Ki, Kd):
		self.prev_error = 0
		self.integral = 0
		
		self.Kp = Kp
		self.Ki = Ki
		self.Kd = Kd
	
	def get(self, dt, sensor, desired):
		error = desired - sensor
		
		p = self.Kp * error
		i = self.integral + (error * dt)
		d = (error - self.prev_error) / dt
		
		self.integral = i
		self.prev_error = error
		
		return p + i + d


class Motor:
	pwm_pin = -1
	current_speed = -1
	current_throttle = -1
	
	pwm = None
	
	# PID related PIV
	mode_scale = None
	
	def __init__(self, pwm_pin, mode_scale: PID_scale):
		self.pwm_pin = pwm_pin
		self.current_speed = 0
		
		GPIO.setup(pwm_pin, GPIO.OUT)
		
		self.pwm = GPIO.PWM(pwm_pin, 8000)
		self.mode_scale = mode_scale
		self.current_throttle = 0
		
		self.pwm.start(0)
	
	def stop(self):
		self.set_speed(0)
		self.pwm.stop()
	
	def set_speed(self, percent):
		self.current_speed = percent
		self.pwm.ChangeDutyCycle(percent)
	
	def set_throttle(self, percent):
		self.current_throttle = percent
	
	def tick(self, throttle, pid_pitch, pid_roll, pid_yaw):
		roll_offset = self.mode_scale.roll_c * pid_roll
		pitch_offset = self.mode_scale.pitch_c * pid_pitch
		yaw_offset = self.mode_scale.pitch_c * pid_yaw
		
		self.current_throttle = throttle
		speed = self.current_throttle + roll_offset + pitch_offset + yaw_offset
		self.set_speed(speed)
