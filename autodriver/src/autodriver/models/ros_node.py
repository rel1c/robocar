import rospy


class ROSNode(object):
    """ A base class for all ROS Nodes."""

    def __init__(self, name):
        # disallow duplicate creation of a node
        if hasattr(self, 'name'):
            return
        self.name = name

    def start_node(self):
        rospy.init_node(self.name)
        rospy.loginfo('%s started', self.name)
        rospy.on_shutdown(self._cleanup)

    def _cleanup(self):
        rospy.loginfo('%s ended', self.name)
