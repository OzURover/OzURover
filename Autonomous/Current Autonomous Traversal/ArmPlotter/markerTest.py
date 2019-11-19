
import roslib; roslib.load_manifest('visualization_marker_tutorials')
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import tf
import rospy
import math

topic = 'visualization_marker_array'
publisher = rospy.Publisher(topic, MarkerArray)

rospy.init_node('register')

markerArray = MarkerArray()

count = 0
MARKERS_MAX = 100

marker = Marker()
marker.header.frame_id = "/neck"
marker.id = 0
marker.type = marker.ARROW
marker.action = marker.ADD
marker.scale.x = 5
marker.scale.y = 1
marker.scale.z = 1
marker.color.a = 1.0
marker.color.r = 1.0
marker.color.g = 1.0
marker.color.b = 0.0

quaternion = tf.transformations.quaternion_from_euler(0, -1.58, 0 )

marker.pose.orientation.x = quaternion[0]
marker.pose.orientation.y = quaternion[1]
marker.pose.orientation.z = quaternion[2]
marker.pose.orientation.w = quaternion[3]
markerArray.markers.append(marker)

marker2 = Marker()
marker2.header.frame_id = "/neck"
marker2.id = 1
marker2.type = marker.ARROW
marker2.action = marker.ADD
marker2.scale.x = 40
marker2.scale.y = 1
marker2.scale.z = 1
marker2.color.a = 1.0
marker2.color.r = 0
marker2.color.g = 1.0
marker2.color.b = 1.0

quaternion = tf.transformations.quaternion_from_euler(0, -0.78 , 0)

marker2.pose.orientation.x = quaternion[0]
marker2.pose.orientation.y = quaternion[1]
marker2.pose.orientation.z = quaternion[2]
marker2.pose.orientation.w = quaternion[3]

marker2.pose.position.x = 0
marker2.pose.position.y = 0
marker2.pose.position.z = 5
markerArray.markers.append(marker2)

while not rospy.is_shutdown():


   # We add the new marker to the MarkerArray, removing the oldest
   # marker from it when necessary
   #marker2.pose.orientation.x = math.cos(count / 50)
   #marker2.pose.orientation.y = math.sin(count / 50)
   #marker2.pose.orientation.z = 1
   """
   if(count > MARKERS_MAX):
       markerArray.markers.pop(0)

   markerArray.markers.append(marker)
   """
   # Renumber the marker IDs

   # for m in markerArray.markers:
   #     m.id = id
   #     id += 1

   # Publish the MarkerArray
   publisher.publish(markerArray)
   rospy.sleep(0.01)
