#!/usr/bin/env python

import dynamixel
import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayDimension
from std_msgs.msg import Float32
from std_msgs.msg import Int32

#THIS CODE WILL RUN INSIDE TX2
#Servo IDs:
#Gripper Pitch:1
#Gripper Yaw:2
#Sample Box:3
#Camera Horizontal:4
#Camera Vertical:5

# Establish a serial connection to the dynamixel network.
# This usually requires a USB2Dynamixel
serial = dynamixel.SerialStream(port='/dev/ttyUSB4', baudrate=1000000, timeout=1)

# Instantiate our network object
net = dynamixel.DynamixelNetwork(serial)

# Get connected dynamixels
net.scan(1,5)

dxl_count=0
motors=[None]*5

print "Searching Dynamixel motors..."

for dyn in net.get_dynamixels():
	motors[dyn.id-1]=dyn
	dxl_count=dxl_count+1

print "Found "+str(dxl_count)+" Dynamixel motors. Those are:"
for d in motors:
	if(d != None):
		print d.id
	else:
		print None

#Callback Functions

def callback(data):
	if(data.data == 50 or data.data == 52):
		yaw((data.data%10)-1)
	elif(data.data == 60 or data.data == 62):
		pitch((data.data%10)-1)
	elif(data.data == 70 or data.data == 72):
		claw((data.data%10)-1)

def kinecb(data):
	pos = motors[0].current_position
	print "Motor 1-UpDown: "+str(pos)
	if(data.data <= 4096 and data.data >= 0):
		motors[0].moving_speed = 1023
		motors[0].torque_limit = 1023 
		motors[0].max_torque = 1023
		motors[0].goal_position = int(data.data)
		net.synchronize()

def sampleBox(data):
	if(data.data == 0):
		sample_box_close()
	elif(data.data == 1):
		sample_box_open()

def moveCamera(data):
	if(data.data == 1.0):
		cam_ver(1)
	elif(data.data == 2.0):
		cam_ver(-1)
	elif(data.data == 3.0):
		cam_hor(1)
	elif(data.data == 4.0):
		cam_hor(-1)

#Moving Servo Functions

def yaw(d):
	pos = motors[1].current_position
	print "Motor 2-Roll: "+str(pos)
	if(pos+50*d <= 4096 and pos+50*d >= 0):
		motors[1].moving_speed = 1023
		motors[1].torque_limit = 1023
		motors[1].max_torque = 1023
		motors[1].goal_position = int(pos+50*d)
		net.synchronize()

def pitch(d):
	pos = motors[0].current_position
	print "Motor 1-UpDown: "+str(pos)
	if(pos+10*d <= 4096 and pos+10*d >= 0):
		motors[0].moving_speed = 1023
		motors[0].torque_limit = 1023 
		motors[0].max_torque = 1023
		motors[0].goal_position = int(pos+30*d)
		net.synchronize()
		pub1.publish(int(pos+30*d))

def sample_box_close():
	print "Motor 3 - Sample Box Close"
	motors[0].moving_speed = 1023
	motors[0].torque_limit = 1023 
	motors[0].max_torque = 1023
	motors[0].goal_position = 220
	net.synchronize()

def sample_box_open():
	print "Motor 3 - Sample Box Open"
	motors[0].moving_speed = 1023
	motors[0].torque_limit = 1023 
	motors[0].max_torque = 1023
	motors[0].goal_position = 441
	net.synchronize()

def cam_hor(d):
	pos = motors[3].current_position
	print "Motor 4: "+str(pos)
	if(pos+8*d <= 1023 and pos+8*d >= 0):
		motors[3].moving_speed = 1023
		motors[3].torque_limit = 1023
		motors[3].max_torque = 1023
		motors[3].goal_position = int(pos+50*d)
		net.synchronize()

def cam_ver(d):
	pos = motors[4].current_position
	print "Motor 5: "+str(pos)
	if(pos+8*d <= 1023 and pos+8*d >= 0):
		motors[4].moving_speed = 1023
		motors[4].torque_limit = 1023
		motors[4].max_torque = 1023
		motors[4].goal_position = int(pos+32*d)
		net.synchronize()

def start():
	global pub, pub1
	rospy.init_node('DxlControl')
	pub = rospy.Publisher('dxl_angles', Float32MultiArray, queue_size = 5, latch = True)
	pub1 = rospy.Publisher('pitch_angle', Int32, queue_size=5)
	rospy.Subscriber('dxlinput', Float32, callback)
	rospy.Subscriber('servokine', Float32, kinecb)
	rospy.Subscriber('camera_move', Float32, moveCamera)
	rospy.Subscriber('sample_box', Int32, sampleBox)
	rospy.spin()

if __name__ == '__main__':
	start()
