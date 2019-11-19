import rospy
import threading
import time
import math
from ar_track_alvar_msgs.msg import AlvarMarkers


class ARTag:

	def __init__(self):
		self.CONSTANT = 0.969919815855  # 30 cm

	def start(self):
		rospy.init_node("ARTagMarkers")
		rospy.Subscriber("ar_pose_marker", AlvarMarkers, self.process)
		rospy.spin()

	def process(self, data):
		if len(data.markers) > 0:
			z = data.markers[0].pose.pose.position.z
			x = data.markers[0].pose.pose.position.x
			self.dist = z * 100 / self.CONSTANT
			self.angle = math.atan(x/z)
			print(self.dist, self.angle)

	def get_position(self):
		return self.dist, self.angle

if __name__ == "__main__":
	AR = ARTag()
	AR.start()
