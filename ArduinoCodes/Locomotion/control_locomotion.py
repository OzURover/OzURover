import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import Joy

def drive(data):
	if(data.buttons[4] != 0):
		if(data.axes[4] < -0.1 or data.axes[4] > 0.1):
			pub.publish(data.axes[4])
	if(data.buttons[5] != 0):
		if(data.axes[3] < -0.1 or data.axes[3] > 0.1):
			pub.publish(data.axes[3])

def start():
	global pub
	global pub1
	rospy.init_node("PWM2Arduino")
	pub = rospy.Publisher("joyinputx", Float32, queue_size=5)
	pub1 = rospy.Publisher("joyinputy", Float32, queue_size=5)
	rospy.Subscriber("joy", Joy, drive)
	rospy.spin()

if __name__ == '__main__':
	start()
