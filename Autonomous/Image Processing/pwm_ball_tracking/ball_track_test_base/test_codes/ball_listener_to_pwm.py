#!/usr/bin/env python

import rospy
import sys
from std_msgs.msg import String
from std_msgs.msg import Float32

def rotate(message):
    if(message.data == 'Found a ball'):
        pub.publish(1)
    elif(message.data == 'Found nothing'):
        pub.publish(-1)
    else:
        pub.publish(0)

def start():
    global pub
    rospy.init_node('BallTrack2PWM')
    pub = rospy.Publisher('rotate_info', Float32, queue_size=5)
    rospy.Subscriber('detect_info', String, rotate)
    rospy.spin()

def main(args):
    start()

if __name__ == '__main__':
    main(sys.argv)
