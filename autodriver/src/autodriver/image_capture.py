#!/usr/bin/env python
import rospy
import cv2
import threading
from picamera.array import PiRGBArray
from picamera import PiCamera
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from models.ros_publisher import ROSPublisher
from autodriver.srv import GetCameraResolution, GetCameraResolutionResponse


class ImageCapture(ROSPublisher):
    """Captures and converts openCV image data to ROS Image messages."""

    def __init__(self, name, topic, srv_topic, resolution, frame_rate=32, rotation=180):
        super(ImageCapture, self).__init__(name, topic, Image, frame_rate)
        self._res = resolution
        self.lens = PiCamera(resolution=(self._res), framerate=frame_rate)
        self.lens.rotation = rotation
        self._raw = PiRGBArray(self.lens, size=self._res)
        self._converter = CvBridge()
        self._srv = rospy.Service(
            srv_topic, GetCameraResolution, self._send_camera_resolution)

    # def start_node(self):
    #     super(ImageCapture, self).start_node()

    def _send_camera_resolution(self, req):
        """GetCameraResolution service response for node in need"""
        return self._res

    def _cleanup(self):
        super(ImageCapture, self)._cleanup()
        async_closer = threading.Thread(target=self.lens.close)
        async_closer.start()
        async_closer.join()

    def publish(self):
        # publish up to frame_rate msgs/sec
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            self.lens.capture(self._raw, format='bgr', use_video_port=True)
            image = self._raw.array
            msg = self._converter.cv2_to_imgmsg(image, 'bgr8')
            try:
                self._sender.publish(msg)
            except CvBridgeError as e:
                rospy.logerr('Image Capture error: %s', e)
                break
            # clear stream for next frame
            self._raw.truncate(0)
            rate.sleep()


if __name__ == '__main__':
    camera = ImageCapture('image_capture', 'image_data',
                          'get_camera_resolution', (640, 480))
    camera.start_node()
    camera.publish()
