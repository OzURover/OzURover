#!/usr/bin/env python
import rospy
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

def callback_vel(data, arg):
    print (arg + " Turtle1 Speeds: Linear: {} | Angular: {}".format(data.linear.x, data.angular.z))

def callback_pose(data, arg):
    print arg[0], arg[1], data.x, data.y, data.theta
    
def listener():
    rospy.init_node('turtlesim_listener', anonymous=True)

    msg = rospy.wait_for_message("/turtle1/cmd_vel", Twist)
    print msg.linear.x, msg.angular.z

    rospy.Subscriber("/turtle1/cmd_vel", Twist, callback_vel, "Hello")
    rospy.Subscriber("/turtle1/pose", Pose, callback_pose, ("Hello", 1))

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()