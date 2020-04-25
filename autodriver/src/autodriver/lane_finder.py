#!/usr/bin/env python
import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class LaneFinder:
    """
    Subscribes to ROS Image messages on topic.
    Converts messages to OpenCV image data for lane detection.
    """

    def __init__(self, topic):
        self._listener = rospy.Subscriber(topic, Image, self.callback)
        self._bridge = CvBridge()

    def callback(self, data):
        try:
            image = self._bridge.imgmsg_to_cv2(data, 'bgr8')
            rospy.loginfo('Img received')
            cv2.imshow("Frame", image)
            cv2.waitKey(0)
        except CvBridgeError as e:
            rospy.logerr('LaneFinder error: %s', e)


def init():
    finder = LaneFinder('image_data')
    rospy.init_node('lane_finder')
    rospy.spin()


if __name__ == '__main__':
    init()
