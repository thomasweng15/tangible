#!/usr/bin/env python

import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import Pose

class ARTagAnnotator():
    def __init__(self):
        self.markers = []
        self.tag_sub = rospy.Subscriber("ar_pose_marker", AlvarMarkers, self.tag_cb)
        self.pub = rospy.Publisher("annotations", Pose, queue_size=10)

    def tag_cb(self, msg):
        self.markers = msg.markers

    def annotate_tag(self, id):
        marker = self._get_marker_by_id(id)
        if marker is None:
            return 
        self.pub.publish(marker.pose.pose)

    def _get_marker_by_id(self, m_id):
        matches = [m for m in self.markers if m.id == m_id]
        return matches[0] if matches else None

if __name__ == "__main__":
    rospy.init_node("ar_tag_annotator")

    Annotator = ARTagAnnotator()
    
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        Annotator.annotate_tag(0)
        rate.sleep()