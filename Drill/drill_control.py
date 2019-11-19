import numpy as np
import rospy
from std_msgs.msg import Int32
from sensor_msgs.msg import Joy
import time

def callback(data):
    message = Int32()
    if data.axes[1] > 0.1:
        message.data = 1
        motor_publisher.publish(message)
    elif data.axes[1] < -0.1:
        message.data = -1
        motor_publisher.publish(message)
    elif data.buttons[3] == 1:
        message.data = 1
        actuator_publisher.publish(message)
    elif data.buttons[0] == 1:
        message.data = -1
        actuator_publisher.publish(message)
    else:
        message.data = 0
        motor_publisher.publish(message)
        actuator_publisher.publish(message)

def start():
    global actuator_publisher, motor_publisher
    rospy.init_node("Drill_control")
    rospy.Subscriber("joy", Joy, callback)
    actuator_publisher = rospy.Publisher('drillActuator', Int32, queue_size = 5)
    motor_publisher = rospy.Publisher('drillMotor',Int32, queue_size = 5)
    rospy.spin()

if __name__ == '__main__':
	start()
