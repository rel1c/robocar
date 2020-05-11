import rospy
import threading
from abc import ABCMeta, abstractmethod
from ros_node import ROSNode


class ROSSubscriber(ROSNode):
    """A base class for all ROS nodes with subscriber functionality"""

    __metaclass__ = ABCMeta

    def __init__(self, name, topic, msg_type, q_size=None, delay_callback=False):
        """
        Arguments:
            name {str} -- name of the created node
            topic {str} -- topic that is subscribed to
            msg_type {Message} -- type of messages that are received

        Keyword Arguments:
            q_size {int} -- number of messsages to keep in a queue if callback is busy 
            (default: {None})

            delay_callback {bool} -- if True, callback is not initialized until self._listener
            is called (default: {False})
        """

        ROSNode.__init__(self, name)
        self._has_delay = delay_callback
        if self._has_delay:
            self._listener = lambda: rospy.Subscriber(
                topic, msg_type, self._callback, queue_size=q_size)
        else:
            self._listener = rospy.Subscriber(
                topic, msg_type, self._callback, queue_size=q_size)

    def start_node(self, final_init=None):
        super(ROSSubscriber, self).start_node()
        if final_init is not None:
            # ensure node has all required dependencies before starting callback and/or spin
            final_init()
        if self._has_delay:
            # start callback function
            self._listener = self._listener()
        rospy.spin()

    @abstractmethod
    def _callback(self, data):
        pass
