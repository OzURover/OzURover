import rospy
import time
import message_filters
from std_msgs.msg import Float32
from geometry_msgs.msg import PointStamped

class GyroDriverMain:

	init_val = 0

	def __init__(self):
		# Initialize the node.
		rospy.init_node('autonomy_controller', anonymous = True);
		test = message_filters.Subscriber("bno_gyro", PointStamped)
		# Use synchonizer to get 4 encoder datas and bno data simultaneously.
		time_synch = message_filters.ApproximateTimeSynchronizer([test], 1, 0.1, allow_headerless=True)
		time_synch.registerCallback(self.print_sensor_data)
		rospy.spin()

	first = True

	def print_sensor_data(self, test):
		one_cm = 0.0115955 #data

		if self.first:
			#print("We have this" + str(test.point.x))
			self.init_val = test.point.x
			self.first = False
		else:
			print("diff: " + str(test.point.x - self.init_val))
		#print(gyro_data.point.x)
		#print("RM: " + str(test.point.x * -1 / one_cm) + " cm")
		#print("RB: " + str(test.point.x / one_cm) + " cm")
		#print("RB: " + str(test.point.x / one_cm) + " cm")
		#print("LB: " + str(test.point.x * -1 / one_cm) + " cm")

if __name__ == "__main__":
	DRIVER = GyroDriverMain()


# LM 1 turn -> 0.482372
# RB 1 turn -> 0.464739
# RM 1 turn -> -0.479325
# LB 1 turn -> -1.733848
# 50.265482 circumference


