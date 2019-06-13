import time


class BaseClock:
	tick_hz = -1
	dt = 0
	period = 0
	
	def tick(self, dt):
		pass
	
	def start(self):
		old_time = time.time()
		while True:
			try:
				current_time = time.time()
				self.tick(old_time - current_time)
				old_time = current_time
			except KeyboardInterrupt:
				break