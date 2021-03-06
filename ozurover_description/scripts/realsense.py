#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image as msg_Image
from cv_bridge import CvBridge, CvBridgeError


class ImageListener:
    def __init__(self, topic):
        self.topic = topic
        self.bridge = CvBridge()
        self.sub = rospy.Subscriber(topic, msg_Image, self.imageDepthCallback)

    def imageDepthCallback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
            cv2.imshow("image",cv_image)
            pix = (data.width/2, data.height/2)
            print('%s: Depth at center(%d, %d): %f(mm)\r' % (self.topic, pix[0], pix[1], cv_image[pix[1], pix[0]]))
        except CvBridgeError as e:
            print(e)
            return

def main():
    topic = '/camera/depth/image_raw'
    listener = ImageListener(topic)
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node("deneme",anonymous=True)
    main()