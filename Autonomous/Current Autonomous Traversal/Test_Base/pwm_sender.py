import rospy
from std_msgs.msg import Float32, Float32MultiArray
from geometry_msgs.msg import PointStamped

def publish(RM_info):
    arr = Float32MultiArray()
    arr.data = [0.3, 0.3, 0.3, 0.2, 0.2, 0.2]
    forward_pwm_publisher.publish(arr)

def setup():
	global forward_pwm_publisher
        rospy.init_node("Pwm_publisher")
	# Pwm publishers for every single motor.
	forward_pwm_publisher = rospy.Publisher("joyinputx", Float32MultiArray, queue_size=5)
    	rospy.Subscriber("RM_info", PointStamped, publish)
    	rospy.spin()

if __name__ == '__main__':
	setup()
