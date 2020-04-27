import rospy
from abc import abstractmethod
from ros_node import ROSNode


class ROSSubscriber(ROSNode):
    """A base class for all ROS Nodes with subscribing functionality."""

    def __init__(self, name, topic, msg_type, q_size=None):
        super(ROSSubscriber, self).__init__(name)
        self.listener = rospy.Subscriber(
            topic, msg_type, self.callback, queue_size=q_size)

    def start(self):
        super(ROSSubscriber, self).start()
        rospy.spin()

    @abstractmethod
    def callback(self, data):
        pass
