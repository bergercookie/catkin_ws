#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, TransformStamped, Pose
from nav_msgs.msg import Path
import tf2_geometry_msgs
import tf2_ros
import os
from tf.transformations import euler_from_quaternion
import numpy as np

"""
Publish the path of an arbitrary marker wrt the world frame and the static
marker frame
"""
class GroundTruthMonitor():
    def __init__(self):
        """Initialize class instance."""

        self.gt_ns = "ground_truth"
        self._getROSServerParameters()

        self.seq = 0

        self.lcamera_frame_ID = "left_camera"
        self.rcamera_frame_ID = "right_camera"

        self.mov_marker_frame_ID = "mf1"

        self.rate = rospy.Rate(10.0)

        # marker_ID => marker path wrt the workspace origin marker
        # (stat_marker_frame_ID)
        self.gt_paths = {}
        # marker_ID => message sequence in their corresponding paths
        self.gt_path_msg_seqs = {}
        # marker_ID => corresponding nav_msgs/Path publisher object
        self.gt_path_publishers = {}

        # frames that have already been registered - ignore the camera tf
        # frames as well as the origin frame
        self.registered_frames = []
        self.prev_registered_frames = []

        # tf2 related objects
        self.tf_buffer = tf2_ros.Buffer()
        self._listener = tf2_ros.TransformListener(self.tf_buffer)

        self._initSubscribersPublishers()

    def _initMarkerPath(self, marker_ID):
        """Initialize a marker id path for each one of newly found markers.
        
        Ignore only the stat_marker_frame_ID that is to represent the origin.
        """
        print("Initializing a new messages path, for marker_ID \"{}\"".format(
            marker_ID))
        ros_start_time = rospy.Time.now()

        self.gt_paths[marker_ID] = Path()
        self.gt_paths[marker_ID].header.stamp = ros_start_time
        self.gt_paths[marker_ID].header.frame_id = self.stat_marker_frame_ID
        self.gt_paths[marker_ID].header.seq = 1

        # What's the sequence number of the *next available* pose in the
        # odometry path
        self.gt_path_msg_seqs[marker_ID] = 0

        # initialize the corresponding publisher object
        self.gt_path_publishers[marker_ID] = rospy.Publisher(
            "{ns}/marker_{id}".format(ns=self.gt_ns, id=marker_ID),
            Path,
            queue_size=1)

        self._addToMarkerPath(marker_ID)

    def _addToMarkerPath(self, marker_ID):
        """
        Add a new pose to an existing marker's path, based on the incoming
        transformation to the origin
        """
        assert(marker_ID in self.gt_paths.keys())
        assert(marker_ID in self.gt_path_msg_seqs.keys())

        # initialize a PoseStamped at the origin of the child frame
        # looking at its X direction
        marker_pose = PoseStamped()
        marker_pose.header.seq = self.gt_path_msg_seqs[marker_ID]
        marker_pose.header.stamp = rospy.Time.now()
        marker_pose.header.frame_id = marker_ID
        marker_pose.pose = Pose()

        try:
            # static marker => moving marker
            marker_trans = self.tf_buffer.lookup_transform(
                self.stat_marker_frame_ID,
                marker_pose.header.frame_id,
                rospy.Time(0),
                rospy.Duration(1.0))
            marker_gt_pose = tf2_geometry_msgs.do_transform_pose(
                marker_pose,
                marker_trans)

            self.gt_paths[marker_ID].poses.append(marker_gt_pose)

        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            print("Exception occurred:\n{0}\n".format(e))
            return

        self.gt_path_publishers[marker_ID].publish(self.gt_paths[marker_ID])
        self.gt_path_msg_seqs[marker_ID] += 1


    def _initSubscribersPublishers(self):
        """Initialize subscriber, publisher instances."""
        pass

    def _saveMarkerPathToTextFile(self, marker_ID):
        """Save the path of a specific marker to a textfile."""
        # TODO - test this method
        assert(marker_ID in self.gt_paths.keys())
        assert(marker_ID in self.gt_path_msg_seqs.keys())

        fname = os.path.join(self.gt_trajectories_dirname,
                             "{}_path.txt".format(marker_ID))
        print("Saving the 2D path of marker_ID \"{}\" to {}...".format(
            marker_ID, fname))
        with open(fname, "w") as f:
            poses_to_save = self.gt_paths[marker_ID].poses
            poses_to_save = [poses_to_save[index] for index in range(
                0, len(poses_to_save), 10)]
            for a_pose_stamped in poses_to_save:
                quaternion = a_pose_stamped.pose.orientation
                q = np.array(
                    [quaternion.x, quaternion.y, quaternion.z, quaternion.w],
                    dtype=np.float64, copy=True)
                orientation_3d_euler = euler_from_quaternion(q)

                f.write("{time} {x} {y} {theta}\n".format(
                    time=a_pose_stamped.header.stamp,
                    x=a_pose_stamped.pose.position.x, 
                    y=a_pose_stamped.pose.position.y, 
                    theta=orientation_3d_euler[-1] # Yaw
                ))

    def _getROSServerParameters(self):
        """
        Read the necessary for the current node parameters from the ROS
        parameter server
        
        """

        # Fetch the origin marker
        origin_marker_ID_param = "{}/origin_marker_ID".format(self.gt_ns)
        assert rospy.has_param(origin_marker_ID_param), "{} doesn't exist. Please set this first and rerun node.  Exiting...\n".format(origin_marker_ID_param)
        self.stat_marker_frame_ID = rospy.get_param(origin_marker_ID_param)
        self.stat_marker_frame_ID.lstrip("\\")
        assert(self.stat_marker_frame_ID[0:2] == "mf")

    def queryRegisteredFrames(self):
        """Get a list of registered so far frames.

        note: all_frames_as_strings lists all the frames since the *current*
        node start. Frames that were registered previous to that are not
        listed. Frames are persistent even when the aruco codes are not in
        the cameras' scope
        """
        frames_str = self.tf_buffer.all_frames_as_string()
        # keep just the second word of each line, that is the frame we are
        # interested in
        if len(frames_str):
            frames = [line.split(" ")[1] for line in frames_str.split("\n")
                      if len(line)]
            # keep only those that start with mf, ignore the origin
            frames = [a_frame for a_frame in frames if a_frame[0:2] == "mf"
                      and a_frame != self.stat_marker_frame_ID]

            # update the inner counter for registered frames
            self.prev_registered_frames = self.registered_frames
            self.registered_frames = frames

            print "Registered frames so far: {}".format(", ".join(frames))
            return frames

    def refreshGroundTruthPaths(self):
        """Update the nav_msgs/Path of each registered marker."""
        for marker_ID in self.registered_frames:
            self._addToMarkerPath(marker_ID)

    def saveMarkerPaths(self):
        """Save all marker paths to external files in the disk."""

        # create output directory if not there
        self.gt_trajectories_dirname = "ground_truth_trajectories"
        if not os.path.isdir(self.gt_trajectories_dirname):
            os.mkdir(self.gt_trajectories_dirname)
        else:
            print("Ground Truth directory \"{}\" already exists, removing it...".format(self.gt_trajectories_dirname))

        for marker_ID in self.registered_frames:
            self._saveMarkerPathToTextFile(marker_ID)


    def run(self):
        while not rospy.is_shutdown():
            # initialize the odometry path of any newly registered frames
            self.queryRegisteredFrames()
            if self.registered_frames > self.prev_registered_frames:
                frames_to_add = set(self.registered_frames).difference(
                    self.prev_registered_frames)
                for a_frame in frames_to_add:
                    self._initMarkerPath(a_frame)
            
            self.refreshGroundTruthPaths()
            self.rate.sleep()

        print("Saving marker paths...\n" + "=" * 20)
        self.saveMarkerPaths()


def main():
    node_name = "ground_truth_monitor"
    rospy.init_node(node_name)
    print("Initializing {} node...".format(node_name))

    monitor = GroundTruthMonitor()
    monitor.run()


if __name__ == "__main__":
    main()

