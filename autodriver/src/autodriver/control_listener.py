#!/usr/bin/env python
import rospy
import autodriver.serialcomm.controller as driver
from autodriver.msg import ControlState


def callback(data):
    rospy.loginfo('motor percent: %f, heading: %d, reverse: %d' %
                  (data.motor_pct, data.heading, data.reverse))
    # implement only updating fields that have changed
    # maybe wrap these three function calls into a parent helper function in controller.py
    driver.motor(data.motor)
    driver.steer(data.heading)
    driver.reverse(data.reverse)
    driver.read_serial()


def control_listener():
    if not driver.connect():
        return

    rospy.init_node('control_listener')
    rospy.Subscriber('control_data', ControlState, callback)
    rospy.spin()


if __name__ == '__main__':
    control_listener()
