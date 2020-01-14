#!/usr/bin/env python

import time

class Timer:
	stime = time.time()
	def start(self):
		self.stime = time.time()
	def elapsed_time(self):
		return time.time() - self.stime
