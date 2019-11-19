import rospy
import time
import sys
import message_filters
from std_msgs.msg import Float32
from geometry_msgs.msg import PointStamped

class GyroDriverMain:

	constants = {
		"LM": 0.482372,
		"LB": -1.733848,
		"RM": -0.479325,
		"RB": 0.464739,
		"CR": 50.265482
	} # x * CR/LM

	zero =  {
		"LM": 0,
		"LB": 0,
		"RM": 0,
		"RB": 0,
		"reset": False
	}

	current = {
		"pos": 0,
		"prev": 0
	}

	targets = [{
		"x": 200,
		"deg": 0.0,
		"reached": False
	}]

	def __init__(self):
		self.curr_index = 0

	def start(self):
		# Initialize the node.
		rospy.init_node('autonomy_controller', anonymous = True);
		# Publisher for pwm.
		self.rotate_pwm_publisher = rospy.Publisher("joyinputy", Float32, queue_size=5)
		self.forward_pwm_publisher = rospy.Publisher("joyinputx", Float32, queue_size=5)
		# Gyro subscriber.
		bno_subscriber = message_filters.Subscriber("bno_gyro", PointStamped)
		# Subscriber for wheel encoders.
		right_mid_encoder_sub = message_filters.Subscriber("RM_info", PointStamped)
		left_mid_encoder_sub = message_filters.Subscriber("LM_info", PointStamped)
		right_back_encoder_sub = message_filters.Subscriber("RB_info", PointStamped)
		left_back_encoder_sub = message_filters.Subscriber("LB_info", PointStamped)
		# Use synchonizer to get 4 encoder datas and bno data simultaneously.
		time_synch = message_filters.ApproximateTimeSynchronizer([bno_subscriber, right_mid_encoder_sub, right_back_encoder_sub, left_mid_encoder_sub, left_back_encoder_sub], 1, 0.1, allow_headerless=True)
		time_synch.registerCallback(self.loop)
		rospy.spin()

	def move(self, pwm, rotate=False):
		if rotate:
			self.rotate_pwm_publisher.publish(pwm)
		else:
			self.forward_pwm_publisher.publish(pwm)	

	def loop(self, gyro_data, RM, LM, RB, LB):
			if self.curr_index is len(self.targets): exit(0)
			if not self.targets[self.curr_index]["reached"]:
				RM_v = ((RM.point.x - self.zero["RM"]) * self.constants["CR"] / self.constants["RM"])
				LM_v = ((LM.point.x - self.zero["LM"]) * self.constants["CR"] / self.constants["LM"])
				RB_v = ((RB.point.x - self.zero["RB"]) * self.constants["CR"] / self.constants["RB"])
				LB_v = ((LB.point.x - self.zero["LB"]) * self.constants["CR"] / self.constants["LB"])
				VEL = (RM.point.y + LM.point.y + RB.point.y + LB.point.y)/4
				ACC = (RM.point.z + LM.point.z + RB.point.z + LB.point.z)/4
				DEG = gyro_data.point.x

				if not self.zero["reset"]:
					self.zero["RM"] = RM.point.x
					self.zero["LM"] = LM.point.x
					self.zero["RB"] = RB.point.x
					self.zero["LB"] = LB.point.x
					self.zero["reset"] = True
				else:
					if not self.is_target_angle_reached(DEG):
						target_angle = self.targets[self.curr_index]["deg"]
						if DEG - ((DEG + target_angle) % 360) > 0:
							pwm = -1
						else:
							pwm = 1
						self.move(rotate=True, pwm=pwm)
					elif not self.is_target_reached():
						if self.targets[self.curr_index]["x"] > (self.current["pos"]):
							pwm = 0.50
						else:
							pwm = -0.50
						self.move(rotate=False, pwm=pwm)
					else:
						self.move(pwm=0)
						self.current["pos"] = 0
						self.current["prev"] = 0
						self.targets[self.curr_index]["reached"] = True
						self.curr_index = self.curr_index + 1
						self.zero["RM"] = RM.point.x
						self.zero["LM"] = LM.point.x
						self.zero["RB"] = RB.point.x
						self.zero["LB"] = LB.point.x
						return 0

					if not self.is_target_angle_reached(DEG):
						self.zero["RM"] = RM.point.x
						self.zero["LM"] = LM.point.x
						self.zero["RB"] = RB.point.x
						self.zero["LB"] = LB.point.x
					else:
						AVG_DIST = (RM_v + LM_v + RB_v + LB_v)/4
						if AVG_DIST < self.current["pos"]:
							self.current["prev"] = self.current["pos"]
						self.current["pos"] = AVG_DIST + self.current["prev"]

					sys.stdout.write("Target number: {} => {} cm travelled. {} deg ||\r".format(self.curr_index, self.current["pos"], DEG))
					sys.stdout.flush()

	def is_target_reached(self):
		return abs(self.current["pos"] - self.targets[self.curr_index]["x"]) < 10
		# return self.current["pos"] > self.targets[self.curr_index]["x"]

	def is_target_angle_reached(self, deg):
		return abs(self.targets[self.curr_index]["deg"] - deg) < 3

if __name__ == "__main__":
	DRIVER = GyroDriverMain()
	try:
		DRIVER.start()
	except (KeyboardInterrupt):
		DRIVER.move(pwm=0)
		raise
