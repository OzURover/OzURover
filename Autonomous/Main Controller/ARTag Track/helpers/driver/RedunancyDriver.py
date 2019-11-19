import rospy
import time
import sys
import message_filters
from std_msgs.msg import Float32
from geometry_msgs.msg import PointStamped


class Driver:

	zero = {
		"LM": 0,
		"LB": 0,
		"RM": 0,
		"RB": 0,
		"reset": True
	}

	current = {
		"pos": 0,
		"prev": 0,
		"prev_set": False
	}

	def __init__(self):
		self.curr_index = 0
		self.targets = []

	def start(self):
		# Publisher for pwm.
		self.rotate_pwm_publisher = rospy.Publisher(
			"joyinputy", Float32, queue_size=5)
		self.forward_pwm_publisher = rospy.Publisher(
			"joyinputx", Float32, queue_size=5)
		# Gyro subscriber.
		bno_subscriber = message_filters.Subscriber("bno_gyro/s", PointStamped)
		# Subscriber for wheel encoders.
		right_mid_encoder_sub = message_filters.Subscriber(
			"RM_info/s", PointStamped)
		left_mid_encoder_sub = message_filters.Subscriber(
			"LM_info/s", PointStamped)
		right_back_encoder_sub = message_filters.Subscriber(
			"RB_info/s", PointStamped)
		left_back_encoder_sub = message_filters.Subscriber(
			"LB_info/s", PointStamped)
		# Use synchonizer to get 4 encoder datas and bno data simultaneously.
		time_synch = message_filters.ApproximateTimeSynchronizer(
			[bno_subscriber, right_mid_encoder_sub, right_back_encoder_sub, left_mid_encoder_sub, left_back_encoder_sub], 1, 0.1, allow_headerless=True)
		time_synch.registerCallback(self.loop)

	def new_target(self, dist, deg, now=True):
		if now:
			self.curr_index = 0
			self.targets = [{
				"x": dist,
				"deg": deg,
				"reached": False
			}]
		else:
			self.targets.append({
				"x": dist,
				"deg": deg,
				"reached": False
			})

	state = -1
	stateNext = -1
	def move(self, pwm, rotate=False):
		if not self.state is self.stateNext:
			if self.state is 0:
				self.rotate_pwm_publisher.publish(0)
			elif self.state is 1:
				self.forward_pwm_publisher.publish(0)
			self.stateNext = self.state
			print("Mode is changing...")
			time.sleep(2)
		
		if rotate:
			self.rotate_pwm_publisher.publish(pwm)
		else:
			self.forward_pwm_publisher.publish(pwm)

	def loop(self, gyro_data, RM, RB, LM, LB):
		if self.is_waiting():
			pass
		elif not self.targets[self.curr_index]["reached"]:
			if RM.point.z or RB.point.z or LM.point.z or LB.point.z:
				self.zero["RM"] = RM.point.x
				self.zero["LM"] = LM.point.x
				self.zero["RB"] = RB.point.x
				self.zero["LB"] = LB.point.x
				self.current["prev_set"] = False

			if self.zero["reset"]:
				self.current["pos"] = 0
				self.current["prev"] = 0
				self.zero["RM"] = RM.point.x
				self.zero["LM"] = LM.point.x
				self.zero["RB"] = RB.point.x
				self.zero["LB"] = LB.point.x
				self.zero["reset"] = False
				time.sleep(2)
			else:
				RM_v = RM.point.x - self.zero["RM"]
				LM_v = LM.point.x - self.zero["LM"]
				RB_v = RB.point.x - self.zero["RB"]
				LB_v = LB.point.x - self.zero["LB"]
				# VEL = (RM.point.y + LM.point.y + RB.point.y + LB.point.y)/4
				DIST = (RM_v + LM_v + RB_v + LB_v)/4
				DEG = gyro_data.point.x

				if not self.is_target_angle_reached(DEG):
					left = self.target_angle_diff(DEG)[1]
					pwm = 0.3 if left else -0.3

					self.state = 1
					self.move(rotate=True, pwm=pwm)
					# Continuously reset encoder data
					self.zero["RM"] = RM.point.x
					self.zero["LM"] = LM.point.x
					self.zero["RB"] = RB.point.x
					self.zero["LB"] = LB.point.x
					self.current["prev_set"] = False
				elif not self.is_target_reached():
					m = 0.375 if abs(self.targets[self.curr_index]["x"] - (self.current["pos"])) < 30 else 1
					if self.targets[self.curr_index]["x"] > (self.current["pos"]):
						pwm = 1 * m
					else:
						pwm = -1 * m
					self.state = 0
					self.move(rotate=False, pwm=round(pwm, 2))
				else:
					self.move(pwm=0)
					self.zero["reset"] = True
					self.targets[self.curr_index]["reached"] = True
					self.curr_index = self.curr_index + 1
					return 0

				if self.is_target_angle_reached(DEG):
					if DIST < self.current["pos"] and not self.current["prev_set"]:
						self.current["prev"] = self.current["pos"]
						self.current["prev_set"] = True
					self.current["pos"] = DIST + self.current["prev"]

				sys.stdout.write("Travelled: " + str(self.current["pos"]) + " Current degree: " + str(DEG) + " Target degree: " + str(self.targets[self.curr_index]["deg"]) + " Target Angle Difference: " + str(self.target_angle_diff(DEG)[0]) + " PWM: " + str(pwm)+ "             \r")
				sys.stdout.flush()

	def is_target_reached(self):
		return abs(self.current["pos"] - self.targets[self.curr_index]["x"]) < 8
		# return self.current["pos"] > self.targets[self.curr_index]["x"]

	def is_target_angle_reached(self, deg):
		tolerance_angle = 5
		target_angle = self.targets[self.curr_index]["deg"]
		diff = abs(deg - target_angle) % 360 if abs(deg -
                                              target_angle) % 360 < 180 else abs((360 - deg) + target_angle) % 360
		diff = 360 - diff if diff > 180 else diff
		return diff <= tolerance_angle

	def target_angle_diff(self, deg):
		target_angle = self.targets[self.curr_index]["deg"]
		diff = abs(deg - target_angle) % 360 if abs(deg -
                                              target_angle) % 360 < 180 else abs((360 - deg) + target_angle) % 360
		diff = 360 - diff if diff > 180 else diff
		left = (diff + deg) % 360 != 0
		return diff, left

	def is_waiting(self):
		return self.curr_index is len(self.targets)
