#!/usr/bin/env python

"""
Wed Oct 26 17:11:41 EEST 2016, Nikos Koukis
Compute the transformation of the roof cameras wrt the workspace origin.

"""

import re
import os

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped

class MultiRobotBroadcaster:
    def __init__(self):
        self._broadcaster = tf2_ros.TransformBroadcaster()

        self.gt_ns = "ground_truth"
        self._get_ros_server_parameters()

        self.lcamera_frame_ID = "left_camera"
        self.rcamera_frame_ID = "right_camera"

        self.lcamera_ns = "/ar_multi_boards_top_left/"
        self.rcamera_ns = "/ar_multi_boards_top_right/"

        self.seq = 0

        self._init_subscribers_publishers()
        # self._initStaticTransformationProps()

    def _init_subscribers_publishers(self):
        rospy.Subscriber(self.lcamera_ns + "transform",
                         TransformStamped, self._handle_incoming_transform)
        rospy.Subscriber(self.rcamera_ns + "transform",
                         TransformStamped, self._handle_incoming_transform)

    def _get_ros_server_parameters(self):
        """
        Read the necessary for the current node parameters from the ROS
        parameter server

        """

        # Fetch the origin marker
        origin_marker_ID_param = "{}/origin_marker_ID".format(self.gt_ns)
        assert rospy.has_param(origin_marker_ID_param), "{} doesn't exist. Please set this first and rerun node.  Exiting...\n".format(origin_marker_ID_param)
        self.stat_marker_frame_ID = rospy.get_param(origin_marker_ID_param)
        self.stat_marker_frame_ID.lstrip("/")
        assert(self.stat_marker_frame_ID[0:2] == "mf")

    def _handle_incoming_transform(self, incoming_tf):
        """Send to the TF topic the incoming geometry_msgs.TransformStamped.

        Make sure that the origin marker_ID is the parent of the camera frames
        by reversing the child and parent frames in case the origin marker is
        involved in the transformation

        """

        if incoming_tf.child_frame_id == self.stat_marker_frame_ID:
            # Basic data are the same
            outgoing_tf = TransformStamped()
            outgoing_tf.header.seq = incoming_tf.header.seq
            outgoing_tf.header.stamp = incoming_tf.header.stamp

            # Stap the parent and child frames
            outgoing_tf.child_frame_id = incoming_tf.header.frame_id
            outgoing_tf.header.frame_id = incoming_tf.child_frame_id

            # Translation
            outgoing_tf.transform.translation = incoming_tf.transform.translation
            outgoing_tf.transform.rotation = incoming_tf.transform.rotation

        else:
            outgoing_tf = incoming_tf


        self._broadcaster.sendTransform(outgoing_tf)

    def run(self):
        rate = rospy.Rate(10.0)

        while not rospy.is_shutdown():
                rate.sleep()


def main():
    node_name = "tf2_broadcaster"
    print("Initializing node \"{}\"...".format(node_name))
    rospy.init_node(node_name)

    br = MultiRobotBroadcaster()
    br.run()


if __name__ == "__main__":
    main()

