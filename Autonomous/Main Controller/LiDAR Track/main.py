import rospy
import math
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import matplotlib.gridspec as gridspec


class LiDAR:

	def __init__(self):
		self.ranges = []

	def start(self):
		rospy.Subscriber("scan", LaserScan, self.process, queue_size=100)

	def reset(self):
		self.ranges = []

	def get_ranges(self):
		if len(self.ranges) == 10:
			altered = []
			for i in range(len(self.ranges)):
				temp = [x*100.0 if x < 100.0 else 100.0 for x in self.ranges[i]]
				altered.append(temp)
			self.ranges = []
			return altered
		else:
			return []

	def process(self, data):
		self.ranges.append(data.ranges)

def update():
	r = LIDAR.get_ranges()
	if len(r) > 0:
		

		fig.gca().relim()
		fig.gca().autoscale_view()

	rospy.rostime.wallsleep(0.05)

if __name__ == "__main__":
	LIDAR = LiDAR()
	LIDAR.start()



	try:
		while not rospy.core.is_shutdown():
			plt.clf()
			update()
			plt.draw()
			rospy.rostime.wallsleep(0.1)
	except KeyboardInterrupt:
		rospy.core.signal_shutdown('keyboard interrupt')
