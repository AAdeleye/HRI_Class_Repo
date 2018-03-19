""" 
A class to hold the components necessary to filter the Pozyx data
"""

from collections import deque

class FilterData(object):

	def __init__(self, num_points, buff, method="MovingAverage"):
		# Initialize empty deques to hold points
		self.__point_dqs = {"x": deque(maxlen=num_points),
							"y": deque(maxlen=num_points),
							"z": deque(maxlen=num_points)}
		
		self.__prev_points = {}

		self.__buff = buff
		self.__method = method

		# Dictionary to help switch between filtering types
		self.__filter = {
			"MovingAverage": self.__moving_average,
			"Median": self.__median
		}

	def filt(self, position):
		ret = [0]*3
		for i, axis in enumerate("xyz"):
			# Ignore data that is obviously wrong (i.e. outside of self.__buff)
			if axis in self.__prev_points:
				if abs(self.__prev_points[axis] - position[i]) < self.__buff:
					self.__point_dqs[axis].append(position[i])
				self.__prev_points[axis] = position[i]
				# Applies the desired filtering method
				ret[i] = self.__filter[self.__method](self.__point_dqs[axis])
			else:
			# First run; just return the given distance
				self.__point_dqs[axis].append(position[i])
				self.__prev_points[axis] = position[i]
				ret[i] = position[i]
		return ret

	def __moving_average(self, dq):
	# Returns the average of the distances in dq
		return int(sum(dq)/len(dq))

	def __median(self, dq):
	# Returns the median of the distances in dq
		i = int(len(dq)/2) - 1
		return sorted(dq)[i]