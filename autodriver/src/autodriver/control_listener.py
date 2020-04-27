#!/usr/bin/env python
import rospy
from models.ros_subscriber import ROSSubscriber
import autodriver.serialcomm.controller as driver
from autodriver.msg import ControlState


class ConnectionError(RuntimeError):
    def __init__(self, arg):
        self.args = arg


class ControlListener(ROSSubscriber):
    """Listens for control state change and updates hardware."""

    def __init__(self, name, topic):
        super(ControlListener, self).__init__(name, topic, ControlState)

    def start(self):
        if not driver.connect():
            raise ConnectionError('Arduino connection failed')
        super(ControlListener, self).start()

    def callback(self, data):
        rospy.loginfo('motor percent: %f, heading: %d, reverse: %d',
                      data.motor_pct, data.heading, data.reverse)
        driver.motor(data.motor_pct)
        driver.read_serial()
        driver.steer(data.heading)
        driver.read_serial()
        driver.reverse(data.reverse)
        driver.read_serial()


if __name__ == '__main__':
    command_handler = ControlListener('control_listener', 'control_data')
    try:
        command_handler.start()
    except ConnectionError as e:
        rospy.logerr('%s', e)
        pass
