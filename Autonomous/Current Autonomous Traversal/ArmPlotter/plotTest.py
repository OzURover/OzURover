import rospy
import roslib

import math
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
import numpy as np
import tf

class Arm3DPlotter():
    def __init__(self):
        #initializing the variables and objects
        self.seq = 0
        self.marker_array = MarkerArray()
        rospy.init_node("Arm_3D_Plotter")
        self.angle_arr = [0, 0.78, 1.58, 1.58]
        self.dof_lenghts = [5,40,34,18.5]
        self.start_points = [Point()]
        self.end_points = [Point(), Point() , Point(), Point()] # first element is the starting point of the next joint

        ##setting starting positions
        self.setPointCoordinates(point = self.end_points[0], x = 0 , y = 0 , z = 5)
        self.setPointCoordinates(point = self.end_points[1], x = 6.32455532 , y = 0 , z = 11.32455532)
        self.setPointCoordinates(point = self.end_points[2], x = 12.155507215 , y = 0 , z = 5.493603425)
        self.setPointCoordinates(point = self.end_points[2], x = 16.456669849 , y = 0 , z = 1.192440791)

        self.setPointCoordinates(point = self.start_points[0], x = 0 , y = 0 , z = 0)
        self.start_points.append(self.end_points[0])
        self.start_points.append(self.end_points[1])
        self.start_points.append(self.end_points[2])

        #first joint it is constantly vertical to the base, however, the angle may be changed.
        #instant of markers
        self.Joint0 = Marker()
        self.Joint1 = Marker()
        self.Joint2 = Marker()
        self.Joint3 = Marker()
        ##setting default configurations to set arm to zero position
        ##publishers and subscribers

        rospy.Subscriber("arm_angle_array", Float32MultiArray, self.updateAngles)
        self.marker_pub = rospy.Publisher("arm_joint_markers", MarkerArray, queue_size = 10)
        print("Publishers and Subscriber are created!")

        self.setUpMarkers( self.Joint0 , self.dof_lenghts[0] , 0 , self.start_points[0] )
        self.setUpMarkers( self.Joint1 , self.dof_lenghts[1] , 1 , self.start_points[1] )
        self.setUpMarkers( self.Joint2 , self.dof_lenghts[2] , 2 , self.start_points[2] )
        self.setUpMarkers( self.Joint3 , self.dof_lenghts[3] , 3 , self.start_points[3] )

        print("The markers are set!")

        self.marker_array.markers.append(self.Joint0)
        self.marker_array.markers.append(self.Joint1)
        self.marker_array.markers.append(self.Joint2)
        self.marker_array.markers.append(self.Joint3)
        self.marker_pub.publish(self.marker_array)
        print("The main is finished")

    def setUpMarkers(self, Joint, length, array_index , st_point):

        Joint.header.frame_id = "/arm_graphics"
        Joint.id = array_index
        Joint.type = Marker.ARROW
        Joint.action = Marker.ADD
        Joint.scale.x = length
        Joint.scale.y = 0.7
        Joint.scale.z = 0.7
        Joint.color.a = 1.0
        Joint.color.r = 1.0
        Joint.color.g = 1.0
        Joint.color.b = 0.0
        if array_index is 0:
            self.setOrientation(joint = Joint , yaw = 0 , pitch = 1.58)
        else:
            self.setOrientation(joint = Joint , yaw = self.angle_arr[0], pitch = self.angle_arr[array_index])
        Joint.pose.position = st_point
        return Joint

    def updateEndPoints(self):
        #angle_arr[0] = yaw

        for i in range (1,4):
            yaw = -self.angle_arr[0]
            pitch = -self.angle_arr[i]
            ix = self.end_points[i-1].x #ix -> initial x
            iy = self.end_points[i-1].y #iy -> initial y
            iz = self.end_points[i-1].z #iz -> initial z
            #finally set the end point of the dof that calculated initial value + projection_axis value
            self.setPointCoordinates( point = self.end_points[i] , x = ix + self.dof_lenghts[i]*math.cos(pitch)*math.cos(yaw),
                                                                     y = iy + self.dof_lenghts[i]*math.cos(pitch)*math.sin(yaw),
                                                                     z = iz + self.dof_lenghts[i]*math.sin(-pitch) )

    def updateMarkers(self):

        for i in range(1,4):
            self.setOrientation(joint = self.marker_array.markers[i] , yaw = self.angle_arr[0], pitch = self.angle_arr[i])
        #self.setPointCoordinates(self.endefactor_point , )


    def setOrientation(self, joint, yaw, pitch):

        quaternion = tf.transformations.quaternion_from_euler( 0 , -pitch , -yaw) # parameters roll, pitch, yaw

        joint.pose.orientation.x = quaternion[0]
        joint.pose.orientation.y = quaternion[1]
        joint.pose.orientation.z = quaternion[2]
        joint.pose.orientation.w = quaternion[3]

    def setColor(self, joint, r_new, g_new, b_new):
        joint.color.a = 1.0
        joint.color.r = r_new
        joint.color.g = g_new
        joint.color.b = b_new

    def updateAngles(self, msg):
        pi = np.pi
        self.angle_arr = [msg.data[0], pi/2 - msg.data[1] , pi/2 - msg.data[2] - msg.data[1],  np.pi/2 - msg.data[3] - msg.data[2] - msg.data[1]]
        print(self.angle_arr)
        self.updateEndPoints()
        self.updateMarkers()
        if(self.angle_arr[1] >= 1.64):
            self.setColor(self.marker_array.markers[1] , 1.0 , 0 , 0)
        else:
            self.setColor(self.marker_array.markers[1] , 1.0 , 1.0 , 0)
        self.marker_pub.publish(self.marker_array)

    def setPointCoordinates(self, point, x , y, z):
        point.x = x
        point.y = y
        point.z = z

if __name__ == '__main__':
    """ main """
    try:
        inst_Arm3DPlotter = Arm3DPlotter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
