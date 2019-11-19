#!/usr/bin/env python

import cv2 as cv
from time import gmtime, strftime
import numpy as np
import math
import sys
from PyQt4 import QtGui, QtCore
from std_msgs.msg import Float32, Int32
import math
import rospy
import subprocess

class GUI(QtGui.QWidget):
	def __init__(self):
		super(GUI, self).__init__()
		self.initUI()

	def initUI(self):
		#Camera
		self.cam_label = QtGui.QLabel('Camera Move',self)
		self.cam_label.move(50, 25)

		self.cam_button_up = QtGui.QPushButton('^', self)
		self.cam_button_down = QtGui.QPushButton('v', self)
		self.cam_button_left = QtGui.QPushButton('<', self)
		self.cam_button_right = QtGui.QPushButton('>', self)

		self.cam_button_up.resize(35,25)
		self.cam_button_down.resize(35,25)
		self.cam_button_left.resize(35,25)
		self.cam_button_right.resize(35,25)

		self.cam_button_up.move(75,50)
		self.cam_button_down.move(75,100)
		self.cam_button_left.move(45,75)
		self.cam_button_right.move(105,75)

		self.cam_button_up.clicked.connect(self.cam_button_up_s)
		self.cam_button_down.clicked.connect(self.cam_button_down_s)
		self.cam_button_left.clicked.connect(self.cam_button_left_s)
		self.cam_button_right.clicked.connect(self.cam_button_right_s)

		#Sample Box
		self.sample_label = QtGui.QLabel('Sample Box Control',self)
		self.sample_label.move(25, 175)

		self.sample_box_button_open = QtGui.QPushButton('Sample Box Open', self)
		self.sample_box_button_close = QtGui.QPushButton('Sample Box Close', self)

		self.sample_box_button_open.resize(150,25)
		self.sample_box_button_close.resize(150,25)

		self.sample_box_button_open.move(25,200)
		self.sample_box_button_close.move(25,240)

		self.sample_box_button_open.clicked.connect(self.sample_box_open)
		self.sample_box_button_close.clicked.connect(self.sample_box_close)

		#Sample Box Load Sensor
		self.box_load_label = QtGui.QLabel('Sample Box Load Sensor',self)
		self.box_load_label.move(225,25)

		self.box_load_button = QtGui.QPushButton('Get Weight', self)
		self.box_load_button.resize(100,25)
		self.box_load_button.move(225, 60)
		self.box_load_button.clicked.connect(self.get_box_weight)

		self.box_load_weight = QtGui.QLabel('xx',self)
		self.box_load_weight.resize(50,25)
		self.box_load_weight.move(350, 65)

		#Drill Load Sensor
		self.drill_load_label = QtGui.QLabel('Drill Load Sensor',self)
		self.drill_load_label.move(225,110)

		self.drill_load_button = QtGui.QPushButton('Get Weight', self)
		self.drill_load_button.resize(100,25)
		self.drill_load_button.move(225,145)
		self.drill_load_button.clicked.connect(self.get_drill_weight)

		self.drill_load_weight = QtGui.QLabel('xx',self)
		self.drill_load_weight.resize(50,25)
		self.drill_load_weight.move(350,150)

		#Photo Capture
		self.photo_capture = QtGui.QPushButton('Photo Capture', self)
		self.photo_capture.resize(150, 25)
		self.photo_capture.move(230, 200)
		self.photo_capture.clicked.connect(self.capture)

		#Misc Buttons
		#Science
		self.science = QtGui.QLabel('Science Task', self)
		self.science.move(450,25)

		self.auto_contact = QtGui.QPushButton('Auto Contact with Surface', self)
		self.auto_contact.resize(250,25)
		self.auto_contact.move(450,50)
		self.auto_contact.clicked.connect(self.auto_contact_s)

		self.auto_sampling = QtGui.QPushButton('Auto Sampling', self)
		self.auto_sampling.resize(250,25)
		self.auto_sampling.move(450,75)
		self.auto_sampling.clicked.connect(self.auto_sampling_s)

		self.auto_place = QtGui.QPushButton('Auto Placement into Sample Box 1', self)
		self.auto_place.resize(250,25)
		self.auto_place.move(450,100)
		self.auto_place.clicked.connect(self.auto_place_s)

		self.auto_place2 = QtGui.QPushButton('Auto Placement into Sample Box 1', self)
		self.auto_place2.resize(250,25)
		self.auto_place2.move(450,125)
		self.auto_place2.clicked.connect(self.auto_place_s2)

		#Maintenance
		self.maintenance = QtGui.QLabel('Maintenance Task', self)
		self.maintenance.move(450,175)

		self.auto_state = QtGui.QPushButton('Auto State Change', self)
		self.auto_state.resize(250,25)
		self.auto_state.move(450,200)
		self.auto_state.clicked.connect(self.auto_state_s)

		#Arm Home Pos
		self.home_pos = QtGui.QPushButton('Arm Home Pos', self)
		self.home_pos.resize(250,25)
		self.home_pos.move(450,250)
		self.home_pos.clicked.connect(self.go_home_pos)

		# Main Frame
		self.setGeometry(750, 300, 750, 300)
		self.setWindowTitle('OzU Rover Team - Theia')
		self.show()

	def cam_button_up_s(self):
		print 'Cam Up'
		camera_pub.publish(2)

	def cam_button_down_s(self):
		print 'Cam Down'
		camera_pub.publish(1)

	def cam_button_left_s(self):
		print 'Cam Left'
		camera_pub.publish(3)

	def cam_button_right_s(self):
		print 'Cam Right'
		camera_pub.publish(4)

	def sample_box_open(self):
		print 'Sample Box Open'
		sample_box_pub.publish(1)

	def sample_box_close(self):
		print 'Sample Box Close'
		sample_box_pub.publish(0)

	def get_box_weight(self):
		self.box_load_weight.setText(str(sample_box_weight))

	def get_drill_weight(self):
		self.drill_load_weight.setText(str(drill_weight))

	def capture(self):
		flag = 1
		while(flag):
			#Change the IP to the camera you want to use.
			vcap = cv.VideoCapture("rtsp://192.168.88.90:554/user=admin&password=&channel=1&stream=0.sdp?")
			ret, frame = vcap.read()
		
			time=strftime("%d-%m-%Y | %H.%M.%S", gmtime())
			time1=strftime("%Y-%m-%d-%H-%M-%S", gmtime())
			img_new=cv.copyMakeBorder(frame,0,60,0,0,cv.BORDER_CONSTANT,value=[0,0,0])
			cv.putText(img_new, "OzU Rover Team - Theia", (0,740), cv.FONT_HERSHEY_SIMPLEX, 0.75, (255,255,255),1) 
		
			cv.putText(img_new, "-- Capture Time: "+time, (0,770), cv.FONT_HERSHEY_SIMPLEX, 0.75, (255,255,255),1) 

			cv.imwrite('/home/ozurover/Desktop/rover_pic/photo_'+time1+'.jpeg', img_new)
		
			print "Done!"
		
			flag=0

	def auto_contact_s(self):
		auto_arm_pub.publish(1)

	def auto_sampling_s(self):
		auto_arm_pub.publish(2)

	def auto_place_s(self):
		auto_arm_pub.publish(3)

	def auto_place_s2(self):
		auto_arm_pub.publish(4)

	def auto_state_s(self):
		auto_arm_pub.publish(5)

	def go_home_pos(self):
		auto_arm_pub.publish(6)
		
def box_load_cell(data):
	sample_box_weight = data.data

def drill_load_cell(data):
	drill_weight = data.data

def main():
	global gui, camera_pub, sample_box_pub, auto_arm_pub, sample_box_weight, drill_weight

	sample_box_weight = 0.0
	drill_weight = 0.0

	camera_pub = rospy.Publisher("camera_move", Float32, queue_size=5)
	sample_box_pub = rospy.Publisher("sample_box", Int32, queue_size=5)
	auto_arm_pub = rospy.Publisher("auto_arm", Int32, queue_size=5)

	rospy.Subscriber("sample_box_load_cell", Float32, box_load_cell)
	rospy.Subscriber("drill_load_cell", Float32, drill_load_cell)

	rospy.init_node('GUI', anonymous=True)
	app = QtGui.QApplication(sys.argv)
	gui = GUI()
	sys.exit(app.exec_())

if __name__ == '__main__':
	main()
