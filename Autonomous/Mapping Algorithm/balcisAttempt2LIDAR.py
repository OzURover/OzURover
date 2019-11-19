import rospy
from sensor_msgs.msg import PointCloud
import matplotlib.pyplot as plt
import pickle
from math import floor

def callback(data):
	print 'In Callback Function !'
	pts = data.points
	thresholdedPtsX, thresholdedPtsY  = [], []
	for i in range(0, len(pts)):
		if(round(pts[i].z, 1) == 1.4):
			thresholdedPtsX.append(pts[i].x)
			thresholdedPtsY.append(pts[i].y)
	print 'Done Thresholding', len(thresholdedPtsX)
	output = open('data.pkl', 'wb')
    	pickle.dump(thresholdedPtsX, output)
    	pickle.dump(thresholdedPtsY, output)
    	output.close()
	print 'Saved Coordiantes to plot!'

if __name__ == '__main__':	
	rospy.init_node('bbLIDAR')
	rospy.Subscriber('assembled_cloud', PointCloud, callback)
	rospy.spin()


