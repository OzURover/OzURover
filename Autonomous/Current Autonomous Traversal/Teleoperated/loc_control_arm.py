import rospy
from std_msgs.msg import Float32, Float32MultiArray
from sensor_msgs.msg import Joy

def drive(data):
	if(data.axes[4] < -0.1 or data.axes[4] > 0.1):
		arr = Float32MultiArray()
		pwm_value = data.axes[4] * 1.0 / 4
		arr.data = [pwm_value, pwm_value, pwm_value, pwm_value, pwm_value, pwm_value]
		pub.publish(arr)
	elif(data.axes[2] < 1):
		arr = Float32MultiArray()
		pwm_value = - (1.0 - data.axes[2]) / 8
    		arr.data = [pwm_value, pwm_value, pwm_value, pwm_value, pwm_value, pwm_value]
		pub1.publish(arr)
	elif(data.axes[5] < 1):
		arr = Float32MultiArray()
		pwm_value = (1.0 - data.axes[5]) / 8
    		arr.data = [pwm_value, pwm_value, pwm_value, pwm_value, pwm_value, pwm_value]
		pub1.publish(arr)
	else:
		arr = Float32MultiArray()
		arr.data = [0, 0, 0, 0, 0, 0]
		pub.publish(arr)


def start():
	global pub
	global pub1
	rospy.init_node("PWM2Arduino")
	pub = rospy.Publisher("joyinputx", Float32MultiArray, queue_size=2)
	pub1 = rospy.Publisher("joyinputy", Float32MultiArray, queue_size=2)
	rospy.Subscriber("joy", Joy, drive)
	rospy.spin()

if __name__ == '__main__':
	start()
