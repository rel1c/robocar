#!/usr/bin/env python
from enum import Enum
import rospy
from models.ros_subscriber import ROSSubscriber
import autodriver.serialcomm.controller as driver
from autodriver.msg import ControlState


class ConnectionError(RuntimeError):
    def __init__(self, arg):
        self.args = arg


class DriveState(Enum):
    """Named indices of node state array"""
    SPEED = 0
    DIRECTION = 1
    IN_REVERSE = 2


class ControlListener(ROSSubscriber):
    """Listens for control state change and updates hardware."""

    def __init__(self, name, topic):
        super(ControlListener, self).__init__(name, topic, ControlState)
        # speed, direction, in_reverse
        self.state = [0.0, 90, 0]

    def start(self):
        if not driver.connect():
            raise ConnectionError('Arduino connection failed')
        super(ControlListener, self).start()

    def set_state(self, typeof, new_state, action):
        if new_state != self.state[typeof.value]:
            self.state[typeof.value] = new_state
            action(new_state)
            driver.read_serial()

    def callback(self, data):
        rospy.loginfo('motor percent: %f, heading: %d, reverse: %d',
                      data.motor_pct, data.heading, data.reverse)
        self.set_state(DriveState.SPEED, data.motor_pct, driver.motor)
        self.set_state(DriveState.DIRECTION, data.heading, driver.steer)
        self.set_state(DriveState.IN_REVERSE, data.reverse, driver.reverse)

    def __del__(self):
        driver.reset()


if __name__ == '__main__':
    command_handler = ControlListener('control_listener', 'control_data')
    try:
        command_handler.start()
    except ConnectionError as e:
        rospy.logerr('%s', e)
        pass
