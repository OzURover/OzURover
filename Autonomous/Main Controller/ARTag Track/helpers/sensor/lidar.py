import rospy
import math
from sensor_msgs.msg import LaserScan


class LiDAR:

	def __init__(self):
		self.ranges = []
		self.MARGIN = 7.33

	def start(self):
		rospy.Subscriber("scan", LaserScan, self.process, queue_size=1)

	def reset(self):
		self.ranges = []

	def get_ranges(self):
		temp = [x*100.0 for x in self.ranges]
		self.ranges = []
		return temp

	def get_min(self):
		if len(self.ranges) > 0:
			span = 10 / (270.0 / 1080)
			r = self.get_ranges()
			return min(r[(540 - int(span)):(540 + int(span))])

	def get_accurate(self, approx):
		if len(self.ranges) > 0:
			deg = 90 - math.degrees(math.atan(approx/self.MARGIN))
			count = deg / (270.0 / 1080)
			r = self.get_ranges()

			f = r[540 + int(count)]
			s = r[540 + int(count) + 1]
			if abs(f - s) > 20:
				return min([f, s])
			else:
				return (f + s) / 2.0
		return -1

	def process(self, data):
		self.ranges = data.ranges
