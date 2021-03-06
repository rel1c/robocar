#!/usr/bin/env python
import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from models.ros_subscriber import ROSSubscriber


class LaneFinder(ROSSubscriber):
    def __init__(self, name, topic):
        super(LaneFinder, self).__init__(name, topic, Image)
        self.converter = CvBridge()

    def start(self):
        super(LaneFinder, self).start()

    def callback(self, data):
        try:
            image = self.converter.imgmsg_to_cv2(data, 'bgr8')
            rospy.loginfo('Img received')
            # cv2.imshow("Frame", image)
            # cv2.waitKey(0)

        except CvBridgeError as e:
            rospy.logerr('Lane Finder error: %s', e)
            # figure out how to exit from node (can't just call return)


if __name__ == '__main__':
    subscriber = LaneFinder('lane_finder', 'image_data')
    subscriber.start()
