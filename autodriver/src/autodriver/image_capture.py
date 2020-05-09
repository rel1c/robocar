#!/usr/bin/env python
import cv2
from picamera.array import PiRGBArray
from picamera import PiCamera
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from models.ros_publisher import ROSPublisher


class ImageCapture(ROSPublisher):
    """Captures and converts openCV image data to ROS Image messages."""

    def __init__(self, name, topic, width, height, frame_rate=32, rotation=180):
        super(ImageCapture, self).__init__(name, topic, Image, frame_rate)
        self.lens = PiCamera(resolution=(width, height), framerate=frame_rate)
        self.lens.rotation = rotation
        self._raw = PiRGBArray(self.lens, size=self.lens.resolution)
        self._converter = CvBridge()

    def start_node(self):
        super(ImageCapture, self).start_node()

    def _cleanup(self):
        super(ImageCapture, self)._cleanup()
        self.lens.close()

    def publish(self):
        while not rospy.is_shutdown():
            self.lens.capture(self._raw, format='bgr', use_video_port=True)
            image = self._raw.array
            msg = self._converter.cv2_to_imgmsg(image, 'bgr8')
            try:
                # publish up to frame_rate msgs/sec
                self._sender.publish(msg)
            except CvBridgeError as e:
                rospy.logerr('Image Capture error: %s', e)
                break
            # clear stream for next frame
            self._raw.truncate(0)


if __name__ == '__main__':
    camera = ImageCapture('image_capture', 'image_data', 320, 240)
    camera.start_node()
    camera.publish()
