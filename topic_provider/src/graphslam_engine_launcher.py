#!/usr/bin/env python

""" 
Wrapper for launching the MRPT graphslam-engine application from a ROS node.

"""

import subprocess
from subprocess import call
import os
import rospy
from supplementary.custom_exceptions import MRPTExecutableNotFoundError


def main():

    # initialize the ros node
    node_name = "graphslam_engine_launcher"
    rospy.init_node(node_name, log_level=rospy.DEBUG)

    rospy.loginfo("{name}: Initializing...".format(name=node_name))

    bash_mrpt_var = "MRPT_DIR"
    ros_graphslam_engine_ns = "/graphslam_engine"

    command_opt_to_prefix = {
        "NRD": "--node-reg",
        "ERD": "--edge-reg",
        "GSO": "--optimizer",
        "ini_file": "--ini-file",
        "gt_file": "--ground-truth"
    }
    command_opt_to_value = {} # None if this is a switch argument

    # find the graphslam-engine executable
    if bash_mrpt_var in os.environ:
        mrpt_path = os.environ[bash_mrpt_var]
        app_path = "".join([mrpt_path,
                            os.sep, "bin", os.sep,
                            "graphslam-engine"])
    elif call(["which", "graphslam-engine"]): # search for it in the user path
        app_path = "graphslam-engine" # already in path
    else: # can't be found - inform user
        raise MRPTExecutableNotFoundError("graphslam-engine")
    rospy.logdebug("{name}: graphslam-engine path: {path}".format(
        name=node_name,
        path=app_path))

    # fetch the cmd options from the ROS parameter server
    rospy.logdebug("{name}: Fetching the command line arguments...".format(
        name=node_name))

    # deciders - optimizers
    for decider in ["NRD", "ERD", "GSO"]:
        choice = rospy.get_param(
            "{ns}/deciders_optimizers/{dec}".format(
                ns=ros_graphslam_engine_ns,
                dec=decider))
        command_opt_to_value[decider] = choice

    # filenames
    for fname_arg in ["ini_file", "gt_file"]:
        choice = rospy.get_param("{ns}/filenames/{fname}".format(
            ns=ros_graphslam_engine_ns,
            fname=fname_arg))
        command_opt_to_value[fname_arg] = choice

    # build the command and execute it
    command_parts = [app_path, "--online"]

    assert(len(command_opt_to_prefix) == len(command_opt_to_value))
    for key in command_opt_to_prefix.keys():
        prefix = command_opt_to_prefix[key]
        value = command_opt_to_value[key]

        if value is not "":
            command_parts += [prefix]
            command_parts += [value]

    rospy.logdebug("{name}: graphslam-engine command:\n{command}".format(
        name=node_name,
        command=command_parts))
    call(command_parts)


if __name__ == "__main__":
    main()
    


# path to executable

rawlog_path = " " # TODO - add here..


# command composition and execution
# call([app_path, 






