#!/usr/bin/env python
import rospy
from std_msgs.msg import String

dataCounter = 0
datas = [0,0,0,0]

def write(data):
    global dataCounter, datas
    
    if "+" in data.data:
        datas[dataCounter] = data.data.split("+")[1]
    else:
        datas[dataCounter] = data.data

    dataCounter += 1
    if dataCounter == 4:
        dataCounter = 0
    
    if dataCounter == 0:
        print(datas)

def read():
    rospy.init_node("ScienceSensorsChannel", anonymous=True)
    rospy.Subscriber("AllSensors", String, write)
    rospy.spin()

if __name__ == '__main__':
    read()
