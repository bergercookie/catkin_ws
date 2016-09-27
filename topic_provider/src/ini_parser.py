#!/usr/bin/env python
# Thu Sep 22 11:48:34 EEST 2016, Nikos Koukis

"""
Purpose of this node is to parse an external .ini file that is created for the
MRPT graphslam-engine application and fetch some of the defined parameters in
it (e.g. server address, server port number).

"""

import rospy
import configparser
import os
from supplementary.custom_exceptions import FileNotFoundError



def main():
    """
    Grep variables defined in a .ini file and push the values to the ROS
    parameter server.

    """

    node_name = "ini_parser"
    rospy.init_node(node_name)

    comm_ini_section = "CMeasurementProviderParameters"
    top_param_ns = "/graphslam_engine/"

    # Initialization - Fetch where I am reading from to the ROS param. server
    ini_file_path = rospy.get_param("".join([
        top_param_ns,
        "filenames/ini_file"]))
    rospy.loginfo("{name}: ini_file_path = {path}".format(
        name=node_name,
        path=ini_file_path))

    # make sure file exists
    if not os.path.isfile(ini_file_path):
        raise FileNotFoundError(ini_file_path)

    # parse the file - publish to the parameter server
    parser = configparser.ConfigParser(comment_prefixes=("#", ";", "//"))
    parser.read(ini_file_path)

    rospy.logdebug("{name}: Ini file sections are:\n\t{sections}".format(
        name=node_name,
        sections=parser.sections()))

    # Parse the necessary parameters from the .ini file and post to ROS param
    # server

    # Server configuration parameters
    server_addr = parser.get(comm_ini_section,
                             "server_addr",
                             fallback="127.0.0.1")
    server_port_no = int(parser.get(comm_ini_section,
                                    "server_port_no",
                                    fallback=6800))
    rospy.set_param("".join([top_param_ns, "tcp/server_addr"]),
                    server_addr)
    rospy.set_param("".join([top_param_ns, "tcp/server_port_no"]),
                    server_port_no)

    rospy.loginfo("{name}: TCP parameters are set.".format(name=node_name))

    # Available message types for the data transmission
    msg_types_format_1 = int(parser.get(comm_ini_section,
                                        "MSG_TYPE_FORMAT_1",
                                        fallback=1))
    msg_types_format_2 = int(parser.get(comm_ini_section,
                                        "MSG_TYPE_FORMAT_2",
                                        fallback=2))
    msg_types_exit = int(parser.get(comm_ini_section,
                                    "MSG_TYPE_EXIT",
                                    fallback=99))
    rospy.set_param("".join([top_param_ns, "msg_types/format_1"]),
                    msg_types_format_1)
    rospy.set_param("".join([top_param_ns, "msg_types/format_2"]),
                    msg_types_format_2)
    rospy.set_param("".join([top_param_ns, "msg_types/exit"]),
                    msg_types_exit)


if __name__ == "__main__":
    main()
