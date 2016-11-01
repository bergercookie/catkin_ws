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

        self.world_frame_ID = "/world"
        self.lcamera_frame_ID = "/left_camera"
        self.rcamera_frame_ID = "/right_camera"

        self.lcamera_ns = "/ar_multi_boards_top_left/"
        self.rcamera_ns = "/ar_multi_boards_top_right/"

        self.seq = 0

        self._initSubscribersPublishers()
        # self._initStaticTransformationProps()

    def _initSubscribersPublishers(self):
        rospy.Subscriber(self.lcamera_ns + "transform",
                         TransformStamped, self._handleIncomingTF)
        rospy.Subscriber(self.rcamera_ns + "transform",
                         TransformStamped, self._handleIncomingTF)

    def _initStaticTransformationProps(self):
        """DO NOT USE
        Static transformation Properties from the cameras to the origin of
        the workspace.
        
        The following values have been computed as the mean of a
        series of measurements of the transformation of the camera to the
        a marker placed at the workspace origin (with the same orientation as
        the origin axes) as given by ar_sys
        """

        # left_camera => origin
        trans = TransformStamped()

        trans.header.stamp = rospy.Time.now()
        trans.header.frame_id = self.world_frame_ID
        trans.header.seq = self.seq

        trans.child_frame_id = self.lcamera_frame_ID
        self._parseResultsFile(
            os.path.join(
                os.path.dirname(__file__),
                "../workspace_transforms/results_transform_left_to_origin.txt"),
            trans.transform)
        print "Transformation left_camera => origin:\n", trans.transform
        self.lcamera_to_origin_tf = trans

        # origin => right_camera 
        trans = TransformStamped()

        trans.header.stamp = rospy.Time.now()
        trans.header.frame_id = self.world_frame_ID
        trans.header.seq = self.seq

        trans.child_frame_id = self.rcamera_frame_ID

        self._parseResultsFile(
            os.path.join(
                os.path.dirname(__file__),
                "../workspace_transforms/results_transform_right_to_origin.txt"),
            trans.transform)
        print "Transformation right_camera => origin:\n", trans.transform
        self.rcamera_to_origin_tf = trans


    def _handleIncomingTF(self, incoming_tf):
        """Send to the TF topic the incoming geometry_msgs.TransformStamped."""
        self._broadcaster.sendTransform(incoming_tf)

        # TODO
        # Take the opposite transform so that your tf tree can have a global
        # "/world" start

    # def _sendLCameraOriginTransform(self):
        # self._broadcaster.sendTransform(self.lcamera_to_origin_tf)

    # def _sendRCameraOriginTransform(self):
        # self._broadcaster.sendTransform(self.rcamera_to_origin_tf)

    def _parseResultsFile(self, results_fname, transform):
        """DO NOT USE
        Parse the file that holds the transformation of one of the cameras to
        the point marked as origin
        
        """
        assert(os.path.isfile(results_fname))
        with open(results_fname) as f:
            lines = f.readlines()
            print "contents are: ", lines
            assert(len(lines) == 2)

            header, vals = lines
            header = re.split(r"\t+", header.rstrip("\t\n"))
            vals = re.split(r"\t+", vals.rstrip("\t"))
            print "headers are: ", header
            print "vals are: ", vals

            for i in range(len(header)):
                exec("transform.{h} = float({v})".format(h=header[i],
                                                         v=vals[i]))

    def run(self):
        rate = rospy.Rate(10.0)

        while not rospy.is_shutdown():
            # compute and post the transformations of the cameras wrt the
            # origin
            # self._sendLCameraOriginTransform()
            # self._sendRCameraOriginTransform()

            # self.seq += 1

            rate.sleep()


def main():
    node_name = "tf2_broadcaster"
    print("Initializing node \"{}\"...".format(node_name))
    rospy.init_node(node_name)

    br = MultiRobotBroadcaster()
    br.run()


if __name__ == "__main__":
    main()

