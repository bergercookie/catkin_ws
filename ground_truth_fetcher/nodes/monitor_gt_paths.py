#!/usr/bin/env python

""" Track the ground-truth paths of the moving marker IDs."""

from __future__ import print_function
import rospy
from geometry_msgs.msg import PoseStamped, Pose, Quaternion
from nav_msgs.msg import Path
import tf2_geometry_msgs
import tf2_ros
from tf2_ros import TFMessage
import os
from tf.transformations import euler_from_quaternion
import numpy as np


class GroundTruthMonitor(object):
    """
    Publish the path of an arbitrary marker wrt the static marker frame that
    operates as the global origin

    In a multi-robot setup each robot is supposed to have 2 aruco markers
    onboard:
    - One for ground-truth validation that faces the roof cameras
    - One that faces up front and is used at inter-robot meetings so that each
    robot knows the relative transformation wrt its counterpart

    Algorithm should make use of only one of the above for fetching the ground
    truth.  To solve this the top markers should have an *odd* marker ID and
    the inter-robot meetings markers must have *even* marker IDs. This can be
    modified from the configuration file - see  MR_USE_ODD_ARUCO_MARKERS_FOR_GT
    environment variable
    """

    def __init__(self):
        """Initialize class instance."""
        self._get_ros_server_parameters()

        self.lcamera_frame_ID = "left_camera"
        self.rcamera_frame_ID = "right_camera"

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
        self.top_registered_frames = []
        self.prev_top_registered_frames = []

        # tf2 related objects
        self.tf_buffer = tf2_ros.Buffer()
        self._listener = tf2_ros.TransformListener(self.tf_buffer)

        self._init_subscribers_publishers()

    def _init_marker_path(self, marker_ID):
        """Initialize a marker id path for each one of newly found markers.

        Ignore only the stat_marker_frame_ID that is to represent the origin.
        """
        rospy.logwarn(
            "Initializing a new messages path, for marker_ID \"{}\"".format(
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

        self._add_to_marker_path(marker_ID,
                                 do_evaluate_pose=False,
                                 keep_as_2D=True)

    def _add_to_marker_path(self, marker_ID,
                            do_evaluate_pose=True,
                            keep_as_2D=True):
        """
        Add a new pose to an existing marker's path, based on the incoming
        transformation to the origin.

        If the incoming transformation suggests an illogical displacement based
        on the robot velocity, direction it should be ignored by the algorithm
        """
        assert marker_ID in self.gt_paths.keys()
        assert marker_ID in self.gt_path_msg_seqs.keys()

        res = self._lookup_marker_gt_pose(marker_ID,
                                          lookup_duration=1.0,
                                          keep_as_2D=keep_as_2D)
        if res:
            [_, marker_gt_pose] = res

            # Accept new pose only if is under the velocity upper threshold
            if not do_evaluate_pose or (
                    do_evaluate_pose and
                    self._evaluate_candidate_pose(marker_ID, marker_gt_pose)):

                self.gt_paths[marker_ID].poses.append(marker_gt_pose)
                self.gt_path_publishers[marker_ID].publish(self.gt_paths[marker_ID])
                self.gt_path_msg_seqs[marker_ID] += 1

    def _evaluate_candidate_pose(self, marker_ID, cand_pose_stamped):
        """
        Evaluate a potential marker pose.

        Evaluation is taking place based on the marker current position
        estimate, the max velocity that the robot agent may reach, as well as
        the timestamp of the candidate transformation with regards to the
        timestamp of the latest accepted marker pose.

        note: candidate pose should already be wrt the global frame

        returns: True if the suggested by the new pose velocity is smaller than
                 the velocity threshold

        """

        last_pose_stamped = self.gt_paths[marker_ID].poses[-1]

        last_pose_t = last_pose_stamped.header.stamp
        cand_pose_t = cand_pose_stamped.header.stamp
        rospy.logdebug("last_pose_t = {}".format(last_pose_t))
        rospy.logdebug("cand_pose_t = {}".format(cand_pose_t))

        last_pose_pos = last_pose_stamped.pose.position
        cand_pose_pos = cand_pose_stamped.pose.position

        time_diff_nsec = cand_pose_t.to_nsec() - last_pose_t.to_nsec()
        assert time_diff_nsec >= 0, \
            "Time difference between last inserted ({}) and candidate pose ({}) is negative!  Exiting...".format(
                last_pose_t, cand_pose_t)

        # Fetch the distance between the candidate and the latest inserted pose
        dist = GroundTruthMonitor.get_distance_between_points(
            cand_pose_pos,
            last_pose_pos)

        vel_proposed = dist / (time_diff_nsec * 10 ** -9)
        res = vel_proposed < self.vel_thresh
        rospy.logdebug("Proposed velocity: {}".format(vel_proposed))

        # if not res:
            # rospy.logwarn(
                # "distance surpasses velocity threshold! Returning false.")
        return res


    def _lookup_marker_gt_pose(self, marker_ID,
                               lookup_duration=1.0,
                               keep_as_2D=True):
        """
        Utility method that looks up the transformation of a marker_ID to
        the static marker that represents the origin.

        If a transformation is successfully found it is returned by the method,
        along with a pose with regards to the static marker ID, otherwise None
        is returned

        """

        # initialize a PoseStamped at the origin of the child frame
        # looking at its X direction
        marker_pose = PoseStamped()
        marker_pose.header.seq = self.gt_path_msg_seqs[marker_ID]
        marker_pose.header.stamp = rospy.Time.now()
        marker_pose.header.frame_id = marker_ID
        marker_pose.pose = Pose()

        pair = None
        try:
            # static marker => moving marker
            marker_trans = self.tf_buffer.lookup_transform(
                self.stat_marker_frame_ID,
                marker_pose.header.frame_id,
                rospy.Time(0),
                rospy.Duration(lookup_duration))
            # Remove the z-components of the transform
            if keep_as_2D:
                rospy.logdebug(("Removing z component from the transform"
                               "translation, rotation parts"))
                # Translation
                GroundTruthMonitor.remove_trans_comp(marker_trans.transform,
                                                axis="z")

                # Rotation - TODO
                # GroundTruthMonitor.remove_rot_comp(marker_trans.transform,
                                                   # axis="x")
                # GroundTruthMonitor.remove_rot_comp(marker_trans.transform,
                                                   # axis="y")


            marker_gt_pose = tf2_geometry_msgs.do_transform_pose(
                marker_pose,
                marker_trans)


            pair = [marker_trans, marker_gt_pose]
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as exc:
            rospy.loginfo("Exception occurred:\n{0}\n".format(exc))

        return pair

    @staticmethod
    def remove_trans_comp(transform, axis):
        """
        Remove translation component from a geometry_msgs/Transfrom object.
        """

        assert axis in ["x", "y", "z"]
        exec("transform.translation.{} = 0".format(axis))

    @staticmethod
    def remove_rot_comp(transform, axis):
        """
        Remove a rotational component from a geometry_msgs/Transform object.

        TODO: Doesn't seem to produce a valid quaternion for use in the do_transform_pose tf2 call
        """

        assert axis in ["x", "y", "z"]

        exec("transform.rotation.{c} = 0".format(c=axis))
        quat_normalized = GroundTruthMonitor.get_normalized_quat(transform.rotation)

        transform.rotation = quat_normalized

############################################################################################################
# TODO - Test these methods
# TODO - Put them in a seperate module

    @staticmethod
    def get_RPY_from_quat(quat):
        """
        Return the roll, pitch, yaw rotations of a geometry_msgs/Quaternion.
        """

        yaw = np.arctan2(2.0 * (quat.y * quat.z + quat.w * quat.x),
                         quat.w ** 2 - quat.x ** 2 - quat.y ** 2 + quat.z ** 2)

        pitch = np.arcsin(-2.0 * (quat.x * quat.z - quat.w * quat.y))

        roll = np.arctan2(2.0 * (quat.x * quat.y + quat.w * quat.z),
                          quat.w ** 2 + quat.x ** 2 - quat.y ** 2 - quat.z ** 2)

        return roll, pitch, yaw


    @staticmethod
    def get_quat_magn(quat):
        """Compute the magnitude of a geometry_msgs/Quaternion object.

        This is defined as the square root of the multiplication of the
        quaternion with its conjugate
        """

        return np.sqrt(quat.w ** 2 +
                       quat.x ** 2 +
                       quat.y ** 2 +
                       quat.z ** 2)


    @staticmethod
    def get_normalized_quat(quat):
        """
        Return the normalized version of the provided geometry_msgs/Quaternion
        object.
        """

        quat_out = Quaternion
        magn = GroundTruthMonitor.get_quat_magn(quat)

        # devide each one of the coordinates with the quaternion magnitude
        for comp in ["x", "y", "z", "w"]:
            exec("quat_out.{c} = quat.{c}".format(c=comp))

        return quat_out



    @staticmethod
    def multiply_quats(quat1, quat2):
        """Compute the outcome of quaternion mulitplication.

        For explanations on this formula, see
        http://web.archive.org/web/20060914224155/http://web.archive.org/web/20041029003853/http://www.j3d.org/matrix_faq/matrfaq_latest.html#Q50
        """

        quat_out = Quaternion
        quat_out.w = (quat1.w * quat2.w -
                      quat1.x * quat2.x -
                      quat1.y * quat2.y -
                      quat1.z * quat2.z)

        quat_out.x = (quat1.w * quat2.x +
                      quat1.x * quat2.w +
                      quat1.y * quat2.z +
                      quat1.z * quat2.y)

        quat_out.y = (quat1.w * quat2.y +
                      quat1.y * quat2.w +
                      quat1.z * quat2.x -
                      quat1.z * quat2.z)

        quat_out.z = (quat1.w * quat2.z +
                      quat1.z * quat2.w +
                      quat1.x * quat2.y -
                      quat1.y * quat2.x)

        return quat_out


    @staticmethod
    def get_quat_conj(quat):
        """Compute the conjugate of a geom_msgs/Quaternion object."""

        quat_out = Quaternion
        quat_out.w = quat.w
        quat_out.x = -quat.x
        quat_out.y = -quat.y
        quat_out.z = -quat.z

        return quat_out

############################################################################################################

    def _init_anchor_publisher(self, marker_ID):
        """
        Start publishing the transformation between the origin and the anchor
        node of a specific SLAM agent.

        The anchor is located at the first pose the corresponding marker_ID is
        detected
        """
        rospy.logwarn(
            "Initializing anchor frame, for marker_ID \"{}\"".format(
                marker_ID))

        [marker_trans, _] = self._lookup_marker_gt_pose(marker_ID,
                                                        lookup_duration=2.0,
                                                        keep_as_2D=True)
        assert marker_trans

        # change the target frame of the transformation
        marker_trans.child_frame_id = "{}_anchor".format(marker_ID)
        self.tf_static_pub.publish([marker_trans])

    @staticmethod
    def get_distance_between_points(point_1, point_2):
        """
        Get the distance (magnitude of the pose difference) between two
        geom_msgs/Point objects
        """

        return np.sqrt((point_1.x - point_2.x) ** 2 +
                       (point_1.y - point_2.y) ** 2 +
                       (point_1.z - point_2.z) ** 2)


    def _init_subscribers_publishers(self):
        """Initialize subscriber, publisher instances."""

        self.tf_static_pub = rospy.Publisher("/tf_static",
                                             TFMessage,
                                             queue_size=1,
                                             latch=True)

    def _save_marker_path_to_text_file(self, marker_ID):
        """Save the path of a specific marker to a textfile."""
        assert marker_ID in self.gt_paths.keys()
        assert marker_ID in self.gt_path_msg_seqs.keys()

        fname = os.path.join(self.gt_trajectories_dirname,
                             "{}_path.txt".format(marker_ID))
        print("Saving the 2D path of marker_ID \"{}\" to {}...".format(
            marker_ID, fname))
        with open(fname, "w") as a_file:
            poses_to_save = self.gt_paths[marker_ID].poses
            poses_to_save = [poses_to_save[index] for index in range(
                0, len(poses_to_save), 10)]
            for a_pose_stamped in poses_to_save:
                quaternion = a_pose_stamped.pose.orientation
                quat = np.array(
                    [quaternion.x, quaternion.y, quaternion.z, quaternion.w],
                    dtype=np.float64, copy=True)
                orientation_3d_euler = euler_from_quaternion(quat)

                a_file.write("{time} {x} {y} {theta}\n".format(
                    time=a_pose_stamped.header.stamp,
                    x=a_pose_stamped.pose.position.x,
                    y=a_pose_stamped.pose.position.y,
                    theta=orientation_3d_euler[-1] # Yaw
                ))

    def _get_ros_server_parameters(self):
        """
        Read the necessary for the current node parameters from the ROS
        parameter server

        """
        # Fetch the name of the ground truth topic namespace
        self.gt_ns = rospy.get_param("/ground_truth_ns")

        # Fetch the origin marker
        origin_marker_ID_param = "{}/origin_marker_ID".format(self.gt_ns)
        assert rospy.has_param(origin_marker_ID_param),\
            ("\"{}\" doesn't exist. "
             "Please set this first and rerun node. "
             "Exiting...\n".format(origin_marker_ID_param))
        self.stat_marker_frame_ID = rospy.get_param(origin_marker_ID_param)
        self.stat_marker_frame_ID.lstrip("/")
        assert self.stat_marker_frame_ID[0:2] == "mf"

        # fetch whether we are tracking odd or even markers
        track_odd_marker_IDs_param = "{}/track_odd_marker_IDs".format(
            self.gt_ns)
        assert rospy.has_param(track_odd_marker_IDs_param),\
            ("\"{}\" doesn't exist. "
             "Please set this first and rerun node. "
             "Exiting...\n".format(track_odd_marker_IDs_param))
        self.track_odd_marker_IDs = rospy.get_param(track_odd_marker_IDs_param)
        rospy.logwarn("Tracking {} markers...".format(
            "ODD" if self.track_odd_marker_IDs is True else "EVEN"))

        # max velocity that the markers are moving with - Handy for filtering
        # out "ground_truth" bad data
        # Velocity is in [m/s]
        self.vel_thresh = \
            rospy.get_param("{}/markers_max_speed".format(self.gt_ns),
                            default=0.2)


    def _query_registered_frames(self):
        """Get a list of registered so far frames.

        note: all_frames_as_strings lists all the frames since the *current*
        node is launched. Frames that were registered previous to this node
        execution are not listed. Frames are persistent even when the aruco
        codes are not in the cameras' scope
        """
        frames_str = self.tf_buffer.all_frames_as_string()
        # keep just the second word of each line, that is the frame we are
        # interested in
        if len(frames_str):
            frames = [line.split(" ")[1] for line in frames_str.split("\n")
                      if len(line)]
            # Keep only those that start with mf
            # Ignore the origin
            # Ignore the anchors
            frames = [a_frame for a_frame in frames
                      if a_frame[0:2] == "mf"
                      and a_frame != self.stat_marker_frame_ID
                      and not a_frame.endswith("anchor")]
            # Ignore the Odd (Or even) markers as each robot has both types of
            # these on it
            if self.track_odd_marker_IDs: # Keep the Odd marekrs
                frames = [a_frame for a_frame in frames if
                          int(a_frame[-1]) % 2 == 1]
            else: # Keep the Even markers
                frames = [a_frame for a_frame in frames if
                          int(a_frame[-1]) % 2 == 0]

            # update the inner counter for registered frames
            self.prev_top_registered_frames = self.top_registered_frames
            self.top_registered_frames = frames

            rospy.logdebug("Registered (top-facing) frames so far: {}".format(", ".join(frames)))
            return frames

    def _refresh_ground_truth_paths(self):
        """Update the nav_msgs/Path of each registered marker."""
        for marker_ID in self.top_registered_frames:
            self._add_to_marker_path(marker_ID)

    def _save_marker_paths(self):
        """Save all marker paths to external files in the disk."""

        # create output directory if not there
        self.gt_trajectories_dirname = "ground_truth_trajectories"
        if not os.path.isdir(self.gt_trajectories_dirname):
            os.mkdir(self.gt_trajectories_dirname)
        else:
            rospy.loginfo("Ground Truth directory \"{}\" already exists, removing it...".format(self.gt_trajectories_dirname))

        for marker_ID in self.top_registered_frames:
            self._save_marker_path_to_text_file(marker_ID)


    def run(self):
        """Main method in class."""
        while not rospy.is_shutdown():
            # initialize the odometry path of any newly registered frames
            self._query_registered_frames()
            if self.top_registered_frames > self.prev_top_registered_frames:
                frames_to_add = set(self.top_registered_frames).difference(
                    self.prev_top_registered_frames)
                for a_frame in frames_to_add:
                    self._init_marker_path(a_frame)
                    self._init_anchor_publisher(a_frame)

            self._refresh_ground_truth_paths()
            self.rate.sleep()

        rospy.loginfo("Saving marker paths...\n" + "=" * 20)
        self._save_marker_paths()


def main():
    """Main."""
    node_name = "ground_truth_monitor"
    rospy.init_node(node_name)
    rospy.loginfo("Initializing {} node...".format(node_name))

    monitor = GroundTruthMonitor()
    rospy.loginfo("{}: Initialized!".format(node_name))
    monitor.run()


if __name__ == "__main__":
    main()

