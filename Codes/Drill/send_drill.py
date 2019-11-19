import rospy
from std_msgs.msg import Int32
from std_msgs.msg import Float32
from sensor_msgs.msg import Joy

def drillCallback(data):
    message = Int32()
    if data.buttons[8] == 1:
        if data.buttons[3] == 1:
            message.data = 1
            actuator_pub.publish(message)
        elif data.buttons[0] == 1:
            message.data = 0
            actuator_pub.publish(message)
        elif data.axes[1] > 0.1:
            message.data = 1
            motor_pub.publish(message)
        elif data.axes[1] < -0.1:
            message.data = -1
            motor_pub.publish(message)
        elif data.axes[4] > 0.1:
            message.data = 1
            motor2_pub.publish(message)
        elif data.axes[4] < -0.1:
            message.data = -1
            motor2_pub.publish(message)
        else:
            message.data = 0
            motor_pub.publish(message)
            motor2_pub.publish(message)
    else:
        message.data = 0
        motor_pub.publish(message)
        motor2_pub.publish(message)

def loadcellCallback(data):
	print(data.data)

def start():
    global actuator_pub, motor_pub, motor2_pub
    rospy.init_node("Drill_control")
    actuator_pub = rospy.Publisher("drillActuator", Int32, queue_size = 5)
    motor_pub = rospy.Publisher("drillMotor", Int32, queue_size = 5)
    motor2_pub = rospy.Publisher("drillMotor2", Int32, queue_size = 5)
    rospy.Subscriber("joy", Joy, drillCallback)
	rospy.Subscriber("drillLoadcell", Float32, loadcellCallback)
    rospy.spin()

if __name__ == "__main__":
    start()
