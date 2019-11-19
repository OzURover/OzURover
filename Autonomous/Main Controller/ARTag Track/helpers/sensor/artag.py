
import rospy
import math
from ar_track_alvar_msgs.msg import AlvarMarkers


class ARTag:

	def __init__(self):
		self.CONSTANT = 73 / 0.789879958527
		self.markers = []

	def start(self):
		rospy.Subscriber("ar_pose_marker", AlvarMarkers, self.process, queue_size=5)

	def reset(self):
		self.markers = []

	def get_markers(self):
		temp = self.markers
		self.markers = []
		return temp

	def process(self, data):
		markers = []
		if len(data.markers) > 0:
			for i in range(len(data.markers)):
				if abs(data.markers[i].pose.pose.orientation.w) > 0.2:
					continue

				x = data.markers[i].pose.pose.position.x
				y = data.markers[i].pose.pose.position.y
				z = data.markers[i].pose.pose.position.z
				real_z = (z * self.CONSTANT)
				real_x = (x * self.CONSTANT)
				id = data.markers[i].id
				dist = math.sqrt(real_x**2 + real_z**2)
				hangle = math.degrees(math.atan(x/z))
				vangle = math.degrees(math.atan(-y/z))
				markers.append((id, dist, hangle, vangle))
			self.markers = markers
