#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def write(data):
    print(data.data)

def read():
    rospy.init_node("TempMoisChannel", anonymous=True)
    rospy.Subscriber("TempMois", String, write)
    rospy.spin()

if __name__ == '__main__':
    read()
