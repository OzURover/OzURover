#!usr/bin/env python
import rospy
from std_msgs.msg import Float32

counter = 0
datas = [0,0]

def write(data):
	global counter
	datas[counter] = data.data
	counter += 1
	if counter ==2:
		counter = 0

	print(datas)

def start():
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("gpsPub", Float32, write)

    rospy.spin()

if __name__ == "__main__":
    start()
