#!/usr/bin/env python  

"""
Wed Oct 26 17:11:41 EEST 2016, Nikos Koukis
Compute the transformation of the roof cameras wrt the workspace origin.

"""

import rospy
import tf
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
        self._initStaticTransformationProps()

    def _initSubscribersPublishers(self):
        rospy.Subscriber(self.lcamera_ns + "transform", TransformStamped, self._handleIncomingTF)
        rospy.Subscriber(self.rcamera_ns + "transform", TransformStamped, self._handleIncomingTF)

    def _initStaticTransformationProps(self):
        """
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

        trans.transform.translation.x = 0.38970197
        trans.transform.translation.y = 0.35614557
        trans.transform.translation.z = 2.85212646

        trans.transform.rotation.x = -0.70526475
        trans.transform.rotation.y = 0.70832277
        trans.transform.rotation.z = -0.02660438
        trans.transform.rotation.w = 0.00932528

        self.lcamera_to_origin_tf = trans

        # origin => right_camera 
        trans = TransformStamped()

        trans.header.stamp = rospy.Time.now()
        trans.header.frame_id = self.world_frame_ID
        trans.header.seq = self.seq

        trans.child_frame_id = self.rcamera_frame_ID

        trans.transform.translation.x = -0.44225105
        trans.transform.translation.y = -0.66875798
        trans.transform.translation.z = 2.84594372

        trans.transform.rotation.x = 0.99846558
        trans.transform.rotation.y = -0.00614078
        trans.transform.rotation.z = 0.00938969
        trans.transform.rotation.w = 0.05405621

        self.rcamera_to_origin_tf = trans


    def _handleIncomingTF(self, incoming_tf):
        """Send to the TF topic the incoming geometry_msgs.TransformStamped."""
        self._broadcaster.sendTransform(incoming_tf)

    def _sendLCameraOriginTransform(self):
        self._broadcaster.sendTransform(self.lcamera_to_origin_tf)

    def _sendRCameraOriginTransform(self):
        self._broadcaster.sendTransform(self.rcamera_to_origin_tf)


    def run(self):
        rate = rospy.Rate(10.0)

        while not rospy.is_shutdown():
            # compute and post the transformations of the cameras wrt the
            # origin
            self._sendLCameraOriginTransform()
            self._sendRCameraOriginTransform()

            self.seq += 1

            rate.sleep()


def main():
    node_name = "origin_transformations_broadcaster"
    rospy.init_node(node_name)

    br = MultiRobotBroadcaster()
    br.run()


if __name__=="__main__":
    main()

