import rospy
import roslib

from mpl_toolkits import mplot3d

import math
import numpy as np
import matplotlib.pyplot as plt

from std_msgs.msg import Float32MultiArray

class Arm3DPlotter():
	def __init__(self):

		rospy.init_node("Arm_3D_Plotter")
		self.angle_arr = [0,0,0,0]
		self.dof_lenghts = [40,34,18.5]
		self.dof_stPoints = [[0,0,0],[0,0,0],[0,0,0],[0,0,0]]
		self.dof_enPoints = [[0,0,0],[0,0,0],[0,0,0],[0,0,0]]

		self.rate = 10

		rospy.Subscriber("arm_angle_array", Float32MultiArray, self.updateAngles)

		self.fig = plt.figure()

		self.ax = self.fig.gca(projection='3d')

		plt.show()

	def updateGraphics(self):
		##draw 4 vectors
		plt.clf()
		self.ax.quiver(0,0,0,0,0,1, length = 5)
		x_st = 0; y_st = 0; z_st = 5
		for jnt in range(0,3):
			x_end,y_end,z_end = self.calculateEndPoint(sPoint = [x_st,y_st,z_st],angle_radians = [self.angle_radians[0], self.angle_radians[jnt]], length = self.dof_lenghts[jnt])
			u,v,w = self.calculateUnitVector(sPoint = [x_st,y_st,z_st], ePoint = [x_end, y_end, z_end])
			dir_vector = self.calculateUnitVector(   sPoint = [x_st, y_st, z_st], ePoint = [x_end, y_end, z_end])
			self.ax.quiver( x_st,y_st,z_st,u,v,w, length = self.dof_lenghts[jnt])
			x_st = x_end; y_st = y_end; z_st = z_end
		self.fig = plt.figure();
		plt.draw()

	def calculateEndPoint(self, sPoint, angle_radians, length):
		## angle_radians[1]--> angle in xz in radians
		## angle_radians[0]--> angle in xy in radians
		dx = length*math.cos(angle_radians[1])*math.cos(angle_radians[0])
		dy = length*math.cos(angle_radians[1])*math.sin(angle_radians[0])
		dz = length*math.sin(angle_radians[1])
		return [sPoint[0]+dx, sPoint[1]+dy, sPoint[2]+dz]

	def calculateUnitVector(self, sPoint, ePoint):
		u = ePoint[0] - sPoint[0] #direction vector of x
		v = ePoint[1] - sPoint[1] #direction vector of y
		w = ePoint[2] - sPoint[2] #direction vector of z
		vector_lenght = math.pow(math.pow(u,2) + math.pow(v,2) + math.pow(w,2),1/3)
		return [u/vector_lenght, v/vector_lenght, w/vector_lenght]

	def updateAngles(self, msg):
		self.angle_radians = [msg.data[0], msg.data[1], msg.data[2], msg.data[3]]
		self.updateGraphics()


if __name__ == '__main__':
    """ main """
    try:
        inst_Arm3DPlotter = Arm3DPlotter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
