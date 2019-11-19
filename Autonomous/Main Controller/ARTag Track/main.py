import rospy
import datetime
import sys

from helpers.sensor.artag import ARTag
from helpers.driver.RedunancyDriver import Driver

if __name__ == "__main__":
	# Initialize the node.
	rospy.init_node('autonomy_controller', anonymous=True)
	DRIVER = Driver()
	DRIVER.start()
	DRIVER.new_target(300.0, 0.0, True)
	DRIVER.new_target(300.0, 270.0, False)
	DRIVER.new_target(300.0, 180.0, False)
	DRIVER.new_target(300.0, 90.0, False)
	DRIVER.new_target(0.0, 0.0, False)
	rospy.spin()
