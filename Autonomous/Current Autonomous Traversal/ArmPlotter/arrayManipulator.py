
import rospy
import roslib
from std_msgs.msg import Float32MultiArray

import math

rospy.init_node("Arm_Data_Manipulator")

manipulator = rospy.Publisher('arm_angle_array', Float32MultiArray, queue_size=10)

arr = Float32MultiArray()
arr.data = [ 0 , 0.78 , 1.56 , 1.56]
counter = 0

arr.data[1] = 0.7

manipulator.publish(arr)

rospy.spin()
