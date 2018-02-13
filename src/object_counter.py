#!/usr/bin/env python

# ObjectCounter.py:
#	Defines the ObjectCounter class, used to count observations of the object

class ObjectCounter:
	# max_count: max number of observations of the same object
	max_count = 13

	# ids is a class member for each new object will own a different, unique, new id
	ids = 0

	def __init__(self, x, y):
		self.x = x
		self.y = y
		self.count = 1
		ObjectCounter.ids += 1
		self.id = ObjectCounter.ids

	def addObservation(self, x, y):
		
		# Weighted average of previous positions and added position
		newX = (self.x * self.count + x)/(1.0*(self.count+1))
		newY = (self.y * self.count + y)/(1.0*(self.count+1))

		self.x = newX
		self.y = newY

		self.count=min(self.count+1, ObjectCounter.max_count)