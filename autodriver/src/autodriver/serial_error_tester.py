#!/usr/bin/env python
import rospy
from random import randint
from models.ros_publisher import ROSPublisher
from autodriver.msg import ControlState
from serialcomm.params import STEERING_MIN, STEERING_MAX


class Test(ROSPublisher):
    def __init__(self):
        ROSPublisher.__init__(self, 'test', 'control_data', ControlState, 10)
        self._msg = ControlState()

    def publish(self, hz):
        rate = rospy.Rate(hz)
        while not rospy.is_shutdown():
            angle = randint(STEERING_MIN, STEERING_MAX)
            rospy.loginfo('%d', angle)
            self._msg.heading = angle
            self._sender.publish(self._msg)
            rate.sleep()


if __name__ == '__main__':
    tester = Test()
    tester.start_node()
    tester.publish(15)
