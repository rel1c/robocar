import rospy
from abc import ABCMeta, abstractmethod
from ros_node import ROSNode


class ROSSubscriber(ROSNode):
    """A base class for all ROS Nodes with subscribing functionality."""

    __metaclass__ = ABCMeta

    def __init__(self, name, topic, msg_type, q_size=None):
        super(ROSSubscriber, self).__init__(name)
        self._listener = rospy.Subscriber(
            topic, msg_type, self._callback, queue_size=q_size)

    def start_node(self):
        super(ROSSubscriber, self).start_node()
        rospy.spin()

    @abstractmethod
    def _callback(self, data):
        pass
