import sys
from PyQt4 import QtGui, QtCore
from std_msgs.msg import Float32, Float32MultiArray
import math
import rospy
import subprocess

class GUI(QtGui.QWidget):
	def __init__(self):
        	super(GUI, self).__init__()
		self.initUI()

	def initUI(self):
		self.goal_pos0 = QtGui.QLabel('Goal Position for Joint 0', self)
		self.goal_pos2 = QtGui.QLabel('Goal Position for Joint 2', self)
		self.goal_pos1 = QtGui.QLabel('Goal Position for Joint 1', self)
		self.goal_pos0.move(70, 20)
		self.goal_pos2.move(70, 80)
		self.goal_pos1.move(70, 50)

		self.goal_pos0_textedit = QtGui.QLineEdit(self)
		self.goal_pos0_textedit.resize(70,22)
		self.goal_pos0_textedit.move(260, 20)
		self.goal_pos2_textedit = QtGui.QLineEdit(self)
		self.goal_pos2_textedit.resize(70,22)
		self.goal_pos2_textedit.move(260, 80)
		self.goal_pos1_textedit = QtGui.QLineEdit(self)
		self.goal_pos1_textedit.resize(70,22)
		self.goal_pos1_textedit.move(260, 50)

		self.set_goal_pos0_btn = QtGui.QPushButton('Send Goal Pos 1', self)
		self.set_goal_pos0_btn.move(350, 20)
		self.set_goal_pos0_btn.pressed.connect(self.send_goalPos1)
		self.set_goal_pos2_btn = QtGui.QPushButton('Send Goal Pos 3', self)
		self.set_goal_pos2_btn.move(350, 80)
		self.set_goal_pos2_btn.pressed.connect(self.send_goalPos3)
		self.set_goal_pos1_btn = QtGui.QPushButton('Send Goal Pos 2', self)
		self.set_goal_pos1_btn.move(350, 50)
		self.set_goal_pos1_btn.pressed.connect(self.send_goalPos2)


		self.pid_cons_j0 = QtGui.QLabel('PID Constants for Joint 0', self)
		self.j0_kp = QtGui.QLineEdit(self)
		self.j0_ki = QtGui.QLineEdit(self)
		self.j0_kd = QtGui.QLineEdit(self)
		self.pid_j0_btn = QtGui.QPushButton('Send', self)

		self.pid_cons_j0.move(70, 110)
		self.j0_kp.resize(70,20)
		self.j0_kp.move(70, 140)
		self.j0_ki.resize(70,20)
		self.j0_ki.move(70, 170)
		self.j0_kd.resize(70,20)
		self.j0_kd.move(70, 200)
		self.pid_j0_btn.move(70, 230)
		self.pid_j0_btn.pressed.connect(self.send_pid_cons_j0)

		self.pid_cons_j1 = QtGui.QLabel('PID Constants for Joint 1', self)
		self.j1_kp = QtGui.QLineEdit(self)
		self.j1_ki = QtGui.QLineEdit(self)
		self.j1_kd = QtGui.QLineEdit(self)
		self.pid_j1_btn = QtGui.QPushButton('Send', self)

		self.pid_cons_j1.move(260, 110)
		self.j1_kp.resize(70,20)
		self.j1_kp.move(260, 140)
		self.j1_ki.resize(70,20)
		self.j1_ki.move(260, 170)
		self.j1_kd.resize(70,20)
		self.j1_kd.move(260, 200)
		self.pid_j1_btn.move(260, 230)
		self.pid_j1_btn.pressed.connect(self.send_pid_cons_j1)


		self.pid_cons_j2 = QtGui.QLabel('PID Constants for Joint 2', self)
		self.j2_kp = QtGui.QLineEdit(self)
		self.j2_ki = QtGui.QLineEdit(self)
		self.j2_kd = QtGui.QLineEdit(self)
		self.pid_j2_btn = QtGui.QPushButton('Send', self)

		self.pid_cons_j2.move(450, 110)
		self.j2_kp.resize(70,20)
		self.j2_kp.move(450, 140)
		self.j2_ki.resize(70,20)
		self.j2_ki.move(450, 170)
		self.j2_kd.resize(70,20)
		self.j2_kd.move(450, 200)
		self.pid_j2_btn.move(450, 230)
		self.pid_j2_btn.pressed.connect(self.send_pid_cons_j2)

		# Main Frame
		self.setGeometry(0, 0, 750, 300)
		self.setWindowTitle('Theia Arm PID Test')
		self.show()

	def send_goalPos3(self):
		global pub
		message = float(self.goal_pos2_textedit.text())
		pub.publish(message)

	def send_goalPos2(self):
		global pub1
		message = float(self.goal_pos1_textedit.text())
		pub1.publish(message)

	def send_goalPos1(self):
		global pub4
		message = float(self.goal_pos0_textedit.text())
		pub4.publish(message)

	def send_pid_cons_j0(self):
		global pub5
		cons = []
		cons.append(float(self.j0_kp.text()))
		cons.append(float(self.j0_ki.text()))
		cons.append(float(self.j0_kd.text()))
		cons_pub = Float32MultiArray(data = cons)
		pub5.publish(cons_pub)


	def send_pid_cons_j1(self):
		global pub2
		cons = []
		cons.append(float(self.j1_kp.text()))
		cons.append(float(self.j1_ki.text()))
		cons.append(float(self.j1_kd.text()))
		cons_pub = Float32MultiArray(data = cons)
		pub2.publish(cons_pub)

	def send_pid_cons_j2(self):
		global pub3
		cons = []
		cons.append(float(self.j2_kp.text()))
		cons.append(float(self.j2_ki.text()))
		cons.append(float(self.j2_kd.text()))
		cons_pub = Float32MultiArray(data = cons)
		pub3.publish(cons_pub)

def main():
	global gui, pub, pub1, pub2, pub3, pub4, pub5

	pub = rospy.Publisher('goal_pos3', Float32, queue_size=5)
	pub1 = rospy.Publisher('goal_pos2', Float32, queue_size=5)
	pub2 = rospy.Publisher('constants1', Float32MultiArray, queue_size=10)
	pub3 = rospy.Publisher('constants2', Float32MultiArray, queue_size=10)
	pub4 = rospy.Publisher('goal_pos1', Float32, queue_size=25)
	pub5 = rospy.Publisher('constants0', Float32MultiArray, queue_size=10)

	rospy.init_node('Arm_GUI', anonymous=True)
	app = QtGui.QApplication(sys.argv)
	gui = GUI()
	sys.exit(app.exec_())

if __name__ == '__main__':
	main()
