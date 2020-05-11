#!/usr/bin/env python
import cv2
import rospy
import image_proc
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from models.ros_publisher import ROSPublisher
from models.ros_subscriber import ROSSubscriber
from autodriver.msg import ControlState
from autodriver.srv import GetCameraResolution, GetCameraResolutionRequest, GetCameraResolutionResponse
from models.exceptions import ImageProcessingError


# debug param used for viewing
DEBUG = False


class LaneFinder(ROSSubscriber, ROSPublisher):
    def __init__(self, name, sub_topic, pub_topic, srv_topic, pub_qsize):
        ROSSubscriber.__init__(self, name, sub_topic,
                               Image, delay_callback=True)
        ROSPublisher.__init__(self, None, pub_topic, ControlState, pub_qsize)
        self._converter = CvBridge()
        self._srv_topic = srv_topic
        self._msg = ControlState()
        self._cam_res = None

    def _set_camera_resolution(self):
        """Retrieve client service response"""

        rospy.loginfo('Waiting to receive camera resolution...')
        rospy.wait_for_service(self._srv_topic)
        try:
            get_resolution = resolution = rospy.ServiceProxy(
                self._srv_topic, GetCameraResolution)()
            self._cam_res = (resolution.width, resolution.height)
        except rospy.ServiceException as e:
            rospy.logerr(
                'Unable to get resolution, failed to initialize %s: %s', self.name, e)
            # re-raise to force termination; unable to continue
            raise

    def start_node(self):
        # for now constant reverse state and speed
        # self._msg.reverse = 0
        # self._msg.motor_pct = 10

        # receive service response before starting rospy.spin()
        super(LaneFinder, self).start_node(
            final_init=self._set_camera_resolution)

    # def _cleanup(self):
    #     super(LaneFinder, self)._cleanup()

    def publish(self, angle):
        rospy.loginfo('%d', angle)
        self._msg.heading = angle
        self._sender.publish(self._msg)

    def _callback(self, data):
        try:
            image = self._converter.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as e:
            rospy.logerr('Image conversion error: %s', e)
            # implement early node termination

        # transform image
        img_birds = image_proc.birds_eye_view(image)
        # isolate blue areas in the bottom half of the transform
        width, height = self._cam_res
        #width, height = 640, 480
        img_half = img_birds[(height / 2): height, 0: width]
        img_blues = image_proc.isolate_blue(img_half)
        img_erode = image_proc.erode_image(img_blues)
        # find lines in ROI
        lines = image_proc.find_lines(img_erode)

        # find average angle from lines
        try:
            angle = image_proc.average_angle(lines)
        except ImageProcessingError as e:
            rospy.logerr('Image processing error: %s', e)
            self.publish(90)
        else:
            self.publish(angle)

        if DEBUG:
            # draw Hough lines on bird's eye view image
            img_lines = draw_lines(img_birds, lines, 0, height / 2)
            # show the frame
            cv2.imshow("Camera View", image)
            cv2.imshow("Bird's Eye View with Lane Lines", img_birds)
            cv2.waitKey(1)  # required to display images, use Ctrl+C to quit


if __name__ == '__main__':
    lf = LaneFinder("lane_finder", "image_data",
                    "control_data", "get_camera_resolution", 10)
    lf.start_node()
