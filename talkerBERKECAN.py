#!/usr/bin/env python
import rospy	#imported ros library
from std_msgs.msg import Float32 #choosed data type 
from sensor_msgs.msg import Joy  #imported joystick datas

def drive(data): #created publising method
	pub.publish(data.buttons[4]) #published data

def start():
	global pub #described pub variable 
	rospy.init_node("PWM2Arduino") #authorized node name
	pub = rospy.Publisher("joyinputx", Float32, queue_size=5) #choosed chanel and connected with data type
	rospy.Subscriber("joy", Joy, drive) #subscribered joystic channel
	rospy.spin()

if __name__ == '__main__':
	start()

#B3RK3C4N
