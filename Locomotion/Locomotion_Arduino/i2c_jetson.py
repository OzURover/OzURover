#!/usr/bin/env python
import rospy
import smbus
from std_msgs.msg import Float32

bus = smbus.SMBus(0)
address = 0x08
cmd = 0x01

def joy_x(data):
	bus.write_byte(address, 103)
	send_arduino(data)

def joy_y(data):
	bus.write_byte(address, 104)
	send_arduino(data)

def send_arduino(data):
	data = str(data)[6:]

	if data[0] == "-":
		bus.write_byte(address, 100)

	if data[0] == "1" or data[1] == "1":
		bus.write_byte(address, 101)

	else:
		data = data.split(".")[-1]
		counter = 0
	
		while counter<len(data):
			try:
				bus.write_byte(address, int(data[counter:counter+2]))
			except:
				bus.write_byte(address, int(data[-1]))
			counter += 2
	
	bus.write_byte(address, 102)

def start():
	rospy.init_node("PWM2Arduino")
	rospy.Subscriber("joyinputx", Float32, joy_x)
	rospy.Subscriber("joyinputy", Float32, joy_y)
	rospy.spin()

if __name__ == "__main__":
	start()
