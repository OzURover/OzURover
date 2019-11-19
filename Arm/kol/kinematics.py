import numpy as np
import rospy
from std_msgs.msg import Float32MultiArray, Float32, Int32
from sensor_msgs.msg import Joy
import time

#lengths of links
d0 = 6.0
d1 = 40.0
d2 = 34.0
d3 = 2.2
d4 = 21.1

def rotateYaw(a):
	Rx = np.matrix([[np.cos(a), -np.sin(a), 0],
		[np.sin(a), np.cos(a), 0],
		[0, 0, 1]])
	return Rx

def transYaw(a, d):
	R = rotateYaw(a)
	T = np.matrix([[R[0,0], R[0,1], R[0,2], 0],
			[R[1,0], R[1,1], R[1,2], 0],
			[R[2,0], R[2,1], R[2,2], d],
			[0,0,0,1]])
	return T


def rotatePitch(a):
	Ry = np.matrix([[np.cos(a), 0, -np.sin(a)],
		[0,1,0],
		[np.sin(a), 0, np.cos(a)]])
	return Ry

def transPitch(a, d):
	R = rotatePitch(a)
	T = np.matrix([[R[0,0], R[0,1], R[0,2], 0],
			[R[1,0], R[1,1], R[1,2], d],
			[R[2,0], R[2,1], R[2,2], 0],
			[0,0,0,1]])
	return T


def rotateX(a):
	Rz = np.matrix([[1,0,0],
		[0, np.cos(a), -np.sin(a)],
		[0, np.sin(a), np.cos(a)]])
	return Rz
	
def transX(a, d):
	R = rotateX(a)
	T = np.matrix([[R[0,0], R[0,1], R[0,2], 0],
			[R[1,0], R[1,1], R[1,2], d*np.cos(a)],
			[R[2,0], R[2,1], R[2,2], d*np.sin(a)],
			[0,0,0,1]])
	return T


def fkine(q):
	T1 = transYaw(q[0,0], 0)
	T2 = transPitch(q[1,0], d0)
	T3 = transPitch(q[2,0], d1)
	T4 = transPitch(q[3,0], d2)
	T5 = transX(q[4,0], d3)
	T6 = np.matrix([[1,0,0,0],[0,1,0,0],[0,0,1,d4],[0,0,0,1]])
	return np.matmul(np.matmul(np.matmul(np.matmul(np.matmul(T1,T2),T3),T4),T5),T6)
	
#the Jacobian inverse matrix is calculated with matlab
def jacobianInv(q):
	q0 = q[0,0]
	q1 = q[1,0]
	q2 = q[2,0]
	q3 = q[3,0]
	q4 = q[4,0]

	J = np.matrix([[-np.sin(q0)/(d2*np.sin(q1+q2)+d1*np.sin(q1)+d3*np.sin(q1+q2+q3)+d4*np.sin(q1+q2+q3)),
				np.cos(q0)/(d2*np.sin(q1+q2)+d1*np.sin(q1)+d3*np.sin(q1+q2+q3)+d4*np.sin(q1+q2+q3)),0,0,0],
			[(np.sin(q1+q2)*np.cos(q0))/(d1*np.sin(q2)),(np.sin(q1+q2)*np.sin(q0))/(d1*np.sin(q2)),np.cos(q1+q2)/(d1*np.sin(q2)),
				-(np.sin(q0)*np.sin(q3)*(d3+d4))/(d1*np.sin(q2)),(np.cos(q0)*np.sin(q3)*(d3+d4))/(d1*np.sin(q2))],
			[-(np.cos(q0)*(d2*np.sin(q1+q2)+ d1*np.sin(q1)))/(d1*d2*np.sin(q2)),
				-(np.sin(q0)*(d2*np.sin(q1+q2)+d1*np.sin(q1)))/(d1*d2*np.sin(q2)),
				-(d2*np.cos(q1+q2) + d1*np.cos(q1))/(d1*d2*np.sin(q2)),
				(np.sin(q0)*(d3+d4)*(d1*np.sin(q2+q3)+d2*np.sin(q3)))/(d1*d2*np.sin(q2)),
				(np.cos(q0)*(d3 + d4)*(d1*np.sin(q2 + q3) + d2*np.sin(q3)))/(d1*d2*np.sin(q2))],
			[(np.cos(q0)*np.sin(q1))/(d2*np.sin(q2)),(np.sin(q0)*np.sin(q1))/(d2*np.sin(q2)),np.cos(q1)/(d2*np.sin(q2)),
				-(np.sin(q0)*(d3*np.sin(q2+q3)+d4*np.sin(q2+q3)+d2*np.sin(q2)))/(d2*np.sin(q2)), 
				(np.cos(q0)*(d3*np.sin(q2+q3)+d4*np.sin(q2+q3)+d2*np.sin(q2)))/(d2*np.sin(q2))],
			[0,0,0,np.cos(q0)/np.sin(q1+q2+q3),np.sin(q0)/np.sin(q1 + q2 + q3)]])

	return J

def ikine(coord):
	global qko, xko
	Jinv = jacobianInv(qko)
	xk = np.matrix([[coord[0]], [coord[1]], [coord[2]], [0.0001], [0.0001]])
	qk = qko + np.matmul(Jinv,(xk - xko))
	qko = qk
	xko = xk
	return qk

def boundryCheck(x, y, z):
	r = np.sqrt(np.power(x, 2) + np.power(y,2) + np.power(z - 6.9, 2))
	r_max = 85
	if x > 85 :
		print('x out of boundry')
		return False
	if y > 85 :
		print('y out of boundry')
		return False
	if z > 85 - 6.9 :
		print('z out of boundry')
		return False
	if(r > r_max):
		print('out of sphere boundry')
		return False
	else:
		return True

def publishAngles(q):
	pub1.publish(q[0,0])
	pub2.publish(q[1,0])
	pub3.publish(q[2,0])
	pub4.publish(q[3,0])
	print('Forward Kinematics:')
	print(np.around(fkine(q),decimals=2))
	print('Current Position:')
	print(np.around(xko, decimals=2))
	print('Current Angles:')
	qko_degrees = (180 / np.pi)*qko
	print(qko_degrees.astype(int))
	arr = Float32MultiArray()
	arr.data = [qko[0,0], qko[1,0], qko[2,0], qko[3,0]]
	pub8.publish(arr)

def xySquare(length, coord):
	#x y square
	for i in range(0,length*10):
		coord[1] = coord[1] + 0.1
		q = ikine(coord)
		publishAngles(q)
		time.sleep(0.04)

	for i in range(0,length*10):
		coord[1] = coord[1] - 0.1
		q = ikine(coord)
		publishAngles(q)
		time.sleep(0.04)

	for i in range(0,length*10):
		coord[0] = coord[0] + 0.1
		q = ikine(coord)
		publishAngles(q)
		time.sleep(0.04)

	for i in range(0,length*10):
		coord[0] = coord[0] - 0.1
		q = ikine(coord)
		publishAngles(q)
		time.sleep(0.04)

def yzSquare(length, coord):
	#y z square

	for i in range(0,length*10):
		coord[1] = coord[1] + 0.1
		q = ikine(coord)
		publishAngles(q)
		time.sleep(0.04)

	for i in range(0,length*10):
		coord[2] = coord[2] + 0.1
		q = ikine(coord)
		publishAngles(q)
		time.sleep(0.04)

	for i in range(0,length*10):
		coord[1] = coord[1] - 0.1
		q = ikine(coord)
		publishAngles(q)
		time.sleep(0.04)

	for i in range(0,length*10):
		coord[2] = coord[2] - 0.1
		q = ikine(coord)
		publishAngles(q)
		time.sleep(0.04)

def xzSquare(length, coord):
	#x z square
	for i in range(0,length*10):
		coord[2] = coord[2] - 0.1
		q = ikine(coord)
		publishAngles(q)
		time.sleep(0.03)

	for i in range(0,length*10):
		coord[0] = coord[0] + 0.1
		q = ikine(coord)
		publishAngles(q)
		time.sleep(0.03)

	for i in range(0,length*10):
		coord[2] = coord[2] + 0.1
		q = ikine(coord)
		publishAngles(q)
		time.sleep(0.03)

	for i in range(0,length*10):
		coord[0] = coord[0] - 0.1
		q = ikine(coord)
		publishAngles(q)
		time.sleep(0.03)


def armDistance(data):
	global arm_distance
	if(data.data == 0.0):
		arm_distance = 99999.9
	else:
		arm_distance = data.data

def pitchManualMove(data):
	global xko, qko
	pitchAngle = ((2983.0 - data.data) / 2048.0) * np.pi * (1.5/2.0)
	print('pitch angle')
	print(pitchAngle)
	print('old qko')
	print(qko)
	qko[3,0] = pitchAngle
	forwardKinematics = fkine(qko)
	new_x = forwardKinematics[0,3]
	new_y = forwardKinematics[1,3]
	new_z = forwardKinematics[2,3]
	coord = [new_x, new_y, new_z]
	xko = np.matrix([[new_x], [new_y], [new_z], [0.000001], [0.0000001]])
	print('qko')
	qko_degrees = (180/np.pi)*qko
	print(qko_degrees.astype(int))
	print('xko')
	print(np.around(xko,decimals=2))
	print('fkine coord')
	print(np.around(coord,decimals=2))
	arr = Float32MultiArray()
	arr.data = [qko[0,0], qko[1,0], qko[2,0], qko[3,0]]
	pub8.publish(arr)

def callback(data):
	global xko, pub1, pub2, pub3, pub4

	#y
	if(data.buttons[5] == 1):
		coord = [xko[0,0], xko[1,0], xko[2,0]]
		if(data.axes[0] > 0.1 or data.axes[0] < -0.1):
			if(boundryCheck(coord[0], coord[1] + data.axes[0]*0.05, coord[2])):
				coord[1] = coord[1] + data.axes[0]*0.05
				q = ikine(coord)
				publishAngles(q)

	#x
	if(data.buttons[6] == 1):
		coord = [xko[0,0], xko[1,0], xko[2,0]]
		if(data.axes[1] > 0.1 or data.axes[1] < -0.1):
			if(boundryCheck(coord[0] + data.axes[1]*0.05, coord[1], coord[2])):
				coord[0] = coord[0] + data.axes[1]*0.05
				q = ikine(coord)
				publishAngles(q)

	#z
	if(data.buttons[4] == 1):
		coord = [xko[0,0], xko[1,0], xko[2,0]]
		if(data.axes[1] > 0.1 or data.axes[1] < -0.1):
			if(boundryCheck(coord[0], coord[1], coord[2] + data.axes[1]*0.05)):
				coord[2] = coord[2] + data.axes[1]*0.05
				q = ikine(coord)
				publishAngles(q)

	if(data.buttons[7] == 1):
		coord = [xko[0,0], xko[1,0], xko[2,0]]

def start():
	global qko, xko, pub1, pub2, pub3, pub4, pub8, arm_distance
	qko = np.matrix([[0.000001],[np.pi/4],[np.pi/2],[0.000001],[0.000001]])
	print(fkine(qko))
	xko = np.matrix([[65.4074],[0.0],[-3.8388],[0.0001],[0.0001]])
	ikine([65.4074, 0.0, -3.8388])
	print(qko)

	arm_distance = 99999.9
		
	rospy.init_node('Kinematics_Node')
	rospy.Subscriber("joy", Joy, callback)
	rospy.Subscriber("pitch_angle", Int32, pitchManualMove)
	rospy.Subscriber("arm_distance", Float32, armDistance)
	pub1 = rospy.Publisher('goal_pos1', Float32, queue_size=10)
	pub2 = rospy.Publisher('goal_pos2', Float32, queue_size=10)
	pub3 = rospy.Publisher('goal_pos3', Float32, queue_size=10)
	pub4 = rospy.Publisher('goal_pos4', Float32, queue_size=10)
	pub8 = rospy.Publisher('arm_angle_array', Float32MultiArray, queue_size=10)	
	rospy.spin()

if __name__ == '__main__':
	start()
