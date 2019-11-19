import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import Joy

def drive(data):
	axis_x = data.axes[1]
	axis_y = data.axes[4]
	pub.publish(axis_x)
	pub1.publish(axis_y)

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

