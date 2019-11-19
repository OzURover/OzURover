#!/usr/bin/env python
import dynamixel
from time import sleep

#THIS CODE WILL RUN INSIDE TX2
#Servo IDs:
#Gripper Pitch:1
#Gripper Yaw:2
#Sample Box:3
#Camera Horizontal:4
#Camera Vertical:5

# Establish a serial connection to the dynamixel network.
# This usually requires a USB2Dynamixel
serial = dynamixel.SerialStream(port='/dev/ttyUSB0', baudrate=1000000, timeout=1)

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
		print "ID: "+str(d.id)+" Found"

def get_pos(pos):
	return (pos - 1024/2) / (1024/300.0)

# Moving Servo Functions
def cam_ver(d):
	d = 1024/2 + d*(1024/300.0)
	if(d >= 376 and d <= 585):
		motors[3].moving_speed = 1023
		motors[3].torque_limit = 1023
		motors[3].max_torque = 1023
		motors[3].goal_position = int(d)
		net.synchronize()

def cam_hor(d):
	d = 1024/2 + d*(1024/300.0)
	d = 1023 if d == 1024 else d
	if(d <= 1023):
		motors[4].moving_speed = 1023
		motors[4].torque_limit = 1023
		motors[4].max_torque = 1023
		motors[4].goal_position = int(d)
		net.synchronize()

def start():
	# Here
	while True:
		print("hor: " + str(motors[4].current_position), str(get_pos(motors[4].current_position)))
		print("ver: " + str(motors[3].current_position), str(get_pos(motors[3].current_position)))
		cam_hor(float(input("h: ")))
		cam_ver(float(input("v: ")))


if __name__ == '__main__':
	start()
