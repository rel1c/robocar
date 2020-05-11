import rospy
from abc import ABCMeta, abstractmethod
from ros_node import ROSNode


class ROSPublisher(ROSNode):
    """A base class for all ROS Nodes with publishing functionality."""

    __metaclass__ = ABCMeta

    def __init__(self, name, topic, msg_type, q_size):
        super(ROSPublisher, self).__init__(name)
        self._sender = rospy.Publisher(topic, msg_type, queue_size=q_size)

    @abstractmethod
    def publish(self, arg=None):
        pass
