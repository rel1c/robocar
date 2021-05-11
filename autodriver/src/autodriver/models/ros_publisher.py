import rospy
from abc import abstractmethod
from ros_node import ROSNode


class ROSPublisher(ROSNode):
    """A base class for all ROS Nodes with publishing functionality."""

    def __init__(self, name, topic, msg_type, q_size):
        super(ROSPublisher, self).__init__(name)
        self.sender = rospy.Publisher(topic, msg_type, queue_size=q_size)

    @abstractmethod
    def publish(self):
        pass
