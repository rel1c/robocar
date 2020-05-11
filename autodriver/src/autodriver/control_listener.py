#!/usr/bin/env python
import rospy
import threading
from enum import Enum
import autodriver.serialcomm.controller as driver
from autodriver.msg import ControlState
from models.exceptions import ConnectionError
from models.ros_subscriber import ROSSubscriber


class DriveState(Enum):
    """Named indices of node state array."""
    SPEED = 0
    DIRECTION = 1
    IN_REVERSE = 2


class ControlListener(ROSSubscriber):
    """Listens for control state change and updates hardware."""

    def __init__(self, name, topic):
        super(ControlListener, self).__init__(name, topic, ControlState)
        # speed, direction, in_reverse
        self._state = [0.0, 90, 0]

    def start_node(self):
        # ConnectionError raised on failure
        driver.connect()
        super(ControlListener, self).start_node()

    def _cleanup(self):
        super(ControlListener, self)._cleanup()
        async_closer = threading.Thread(target=driver.reset)
        async_closer.start()
        async_closer.join()

    def _set_state(self, typeof, new_state, action):
        if new_state != self._state[typeof.value]:
            self._state[typeof.value] = new_state
            action(new_state)
            # driver.read_serial()

    def _callback(self, data):
        rospy.loginfo('motor percent: %f, heading: %d, reverse: %d',
                      data.motor_pct, data.heading, data.reverse)
        self._set_state(DriveState.SPEED, data.motor_pct, driver.motor)
        self._set_state(DriveState.DIRECTION, data.heading, driver.steer)
        self._set_state(DriveState.IN_REVERSE, data.reverse, driver.reverse)


if __name__ == '__main__':
    command_handler = ControlListener('control_listener', 'control_data')
    try:
        command_handler.start_node()
    except ConnectionError as e:
        rospy.logerr('Connecction error: %s', e)
