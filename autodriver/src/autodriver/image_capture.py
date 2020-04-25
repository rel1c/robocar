#!/usr/bin/env python
import cv2
from picamera.array import PiRGBArray
from picamera import PiCamera
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class ImageCapture:
    """
    Captures and converts openCV image data to ROS Image messages.
    Publishes on topic with a frequency of frame_rate.
    """

    def __init__(self, topic, width, height, frame_rate=32, rotation=180):
        self.camera = PiCamera()
        self.camera.resolution = (width, height)
        self._sender = rospy.Publisher(topic, Image, queue_size=frame_rate)
        self._bridge = CvBridge()

    def __del__(self):
        self.camera.close()

    def take_snapshot(self):
        raw = PiRGBArray(self.camera, size=self.camera.resolution)

        for frame in self.camera.capture_continuous(raw, format='bgr', use_video_port=True):
            # grab the raw NumPy array representing the image, then initialize the timestamp
            # and occupied/unoccupied text
            image = frame.array
            msg = self._bridge.cv2_to_imgmsg(image, 'bgr8')
            try:
                self._sender.publish(msg)
            except CvBridgeError as e:
                rospy.logerr('Image Capture error: %s', e)

            # clear the stream in preparation for the next frame
            raw.truncate(0)


def init():
    camera = ImageCapture('image_data', 320, 240)
    rospy.init_node('image_capture')
    while not rospy.is_shutdown():
        camera.take_snapshot()


if __name__ == '__main__':
    try:
        init()
    except rospy.ROSInterruptException:
        pass
