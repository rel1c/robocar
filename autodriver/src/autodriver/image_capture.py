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
        self.converter = CvBridge()

    def start(self):
        super(ImageCapture, self).start()

    def publish(self):
        raw = PiRGBArray(self.lens, size=self.lens.resolution)

        while not rospy.is_shutdown():
            for frame in self.lens.capture_continuous(raw, format='bgr', use_video_port=True):
                # grab numpy representation of image, initialize the timestamp and occupied/unoccupied text
                image = frame.array
                msg = self.converter.cv2_to_imgmsg(image, 'bgr8')
                try:
                    self.sender.publish(msg)
                except CvBridgeError as e:
                    rospy.logerr('Image Capture error: %s', e)
                    # figure out how to exit from node (can't just call return)

                # clear stream for next frame
                raw.truncate(0)

    def __del__(self):
        self.lens.close()


if __name__ == '__main__':
    camera = ImageCapture('image_capture', 'image_data', 320, 240)
    rospy.loginfo('Image capture started')
    camera.start()
    camera.publish()
