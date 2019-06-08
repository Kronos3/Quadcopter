import sched, time

class BaseClock:
	tick_hz = -1
	dt = 0
	period = 0
	
	def __init__(self, tick_hz):
		self.tick_hz = tick_hz
		self.last_tick = None
		self.scheduler = sched.scheduler(time.time, time.sleep)
		self.dt = 0
		self.period = 1.0 / self.tick_hz
		
		self.register()
	
	def tick(self):
		pass
	
	def start(self):
		self.old_time = time.time()
		self.scheduler.run()
	
	def reload(self, old_time):
		new_time = time.time()
		dt = new_time - old_time
		
		self.tick(dt)
		self.scheduler.enter(self.period, 0, self.reload, [new_time])
	
	def register(self):
		self.scheduler.enter(self.period, 0, self.reload, [time.time()])
