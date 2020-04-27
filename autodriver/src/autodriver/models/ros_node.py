import rospy


class ROSNode(object):
    """ A base class for all ROS Nodes."""

    def __init__(self, name):
        self.name = name

    def start(self):
        rospy.init_node(self.name)
