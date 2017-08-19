from easier_node.easy_node import EasyNode
import rospy
from sensor_msgs.msg import CompressedImage, Image
from duckietown_utils.instantiate_utils import instantiate

class NewLineDetectorNode2(EasyNode):
    def __init__(self):
        EasyNode.__init__(self, 'line_detector2', 'line_detector_node2')
        self.detector = None
        self.pub_edge = None
        self.pub_color_segment = None

    def on_parameters_changed(self, _first_time, updated):
        self.info(updated)
        if 'verbose' in updated:
            self.info('Verbose is now %r' % self.config.verbose)

            if self.config.verbose:
                self.pub_edge = rospy.Publisher("~edge", Image, queue_size=1)
                self.pub_color_segment = rospy.Publisher(
                    "~colorSegment", Image, queue_size=1)
            else:
                self.pub_edge = None
                self.pub_color_segment = None

        if 'detector' in updated:
            c = self.config.detector
            assert isinstance(c, list) and len(c) == 2, c

            self.info('detector config: %s' % str(c))

            self.detector = instantiate(c[0], c[1])
