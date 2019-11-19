
from std_msgs.msg import Float32MultiArray

class AutonomousDriveController:
	# DONT MASK MY FUCKING KOKORU AND ITS NOT AN EMU MAN! BUT STILL SHE'S BEAUTY
	def __init__(self):
		self.slow_down_threeshold = 2 # in meters. If remaining distance is lower than threeshold, then the car will slow down.
		self.forward_pwm_arr = Float32MultiArray()
		self.rotate_pwm_arr = Float32MultiArray()
		self.tmp_arr = Float32MultiArray()

		self.tmp_arr.data = [0, 0, 0, 0, 0, 0]
		self.forward_pwm_arr.data = [0.5, 0.5, 0.5, 0.5, 0.5, 0.5]
		self.rotate_pwm_arr_arr = [0.4, 0.4, 0.4, 0.4, 0.4, 0.4]

		self.rotate_pwm_publisher = rospy.Publisher("joyinputy", Float32MultiArray, queue_size=5)
		self.forward_pwm_publisher = rospy.Publisher("joyinputx", Float32MultiArray, queue_size=5)

		halt()

	def update_forward_pwm_arr(self,new_forward_pwm_arr): # new_forward_pwm_arr -> normal bir array
		#kokoru read
		self.forward_pwm_arr.data = new_forward_pwm_arr

	def update_rotate_pwm_arr(self,new_rotate_pwm_arr): # new_forward_pwm_arr -> normal bir array
		#kokoru read
		self.rotate_pwm_arr.data = new_rotate_pwm_arr

	def forward(self):
		#kokoru run.
		self.forward_pwm_publisher.publish(self.forward_pwm_arr)

	def reverse(self):
		#kokoru come back
		self.tmp_arr.data = - self.forward_pwm_arr.data
		self.forward_pwm_publisher.publish(self.tmp_arr)

	def rotate_right(self):
		#kokoru right!
		self.rotate_pwm_publisher.publish(self.rotate_pwm_arr)

	def smooth_forward(self,  distance_remaining , pwm_err):
		if pwm_err  >  0.5:
			pwm_err = 0.5
		elif pwm_err < -0.5:
			pwm_err = -0.5

		if distance_remaining < self.slow_down_threeshold: # slowing down linearly remaining path
			pwm_constant = 0.25 + (distance_remaining/self.slow_down_threeshold)*0.25
		else:
			pwm_constant = 0.5

		self.forward_pwm_arr.data = [pwm_constant - pwm_err, pwm_constant - pwm_err, pwm_constant - pwm_err, pwm_constant + pwm_err , pwm_constant + pwm_err, pwm_constant + pwm_err]
		self.forward_pwm_publisher.publish(self.forward_pwm_arr)

 	def rotate_left(self):
		#kokoru left??
		self.tmp_arr.data =  - self.rotate_pwm_arr.data
		self.rotate_pwm_publisher.publish(self.tmp_arr)

	def halt(self):
		#kokoru slow slow stop!
		self.tmp_arr.data = [0, 0, 0, 0, 0, 0]
		self.forward_pwm_publisher.publish(self.tmp_arr)
		self.rotate_pwm_publisher.publish(self.tmp_arr)
