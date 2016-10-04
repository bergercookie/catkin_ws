#!/usr/bin/env python
# Tue Oct 4 12:31:30 EEST 2016, Nikos Koukis

"""
Tue Oct 4 12:31:34 EEST 2016, Nikos Koukis

Current script sets up the adhoc network connection for the current host.

"""

from __future__ import print_function
import argparse
from subprocess import call
import os
from custom_exceptions import NoRootAccessError, ProgramNotFoundError, InterfaceNotFoundError
from time import gmtime, strftime

# Arguement Parsing
parser = argparse.ArgumentParser(
    description='Setup Ad-Hoc network configuration for mlti-robot SLAM')

# wireless interface
parser.add_argument(
    '-w', '--wlan_interface',
    default="wlan0",
    help='Specify the wireless interface')

# dry-run flag
parser.add_argument(
    '-d', '--dry-run',
    default=False,
    action="store_true",
    help="Monitor the actions that the script will take if executed")

# ip-address of current host
parser.add_argument(
    '-I', '--ip_address',
    help="Specify the IP address of the current host")

# Run interactively
parser.add_argument(
    "-i", "--interactive",
    action="store_true",
    help="Run script in interactive mode")
args = vars(parser.parse_args())


def main():
    interactive = args["interactive"]
    dry_run = args["dry_run"]

    # Specify whether we are running for real or on a dry-run
    if dry_run:
        print("[!] Script is running on a dry run! Specified commands will not be executed")


    if interactive:
        print("[!] In interactive mode:")
        wlan_interface = raw_input(
            "wlan interface to be used: [Default = wlan0] ")
        ip_address = raw_input(
            "Current host's IP address in ad-hoc: [Default = 192.168.100.11] ")
    else:
        wlan_interface = args["wlan_interface"]
        ip_address = args["ip_address"]

    check_reqs(ip_address=ip_address, wlan_interface=wlan_interface)

    # create interfaces file
    interfaces_dir = "/etc/network/interfaces.d"
    interfaces_main = "/etc/network/interfaces"
    interfaces_fname = "multi_robot_adhoc"
    if not os.path.isdir(interfaces_dir):
        print("Directory {} doesn't exist, creating it...".format(
            interfaces_dir))
        if not dry_run:
            os.mkdir(interfaces_dir, 715)

    # write the interfaces file
    with open("".join([interfaces_dir, os.sep, interfaces_fname]), "w") as f:
        comments = [
            "Automatically generated script - {date}".format(
                date=strftime("%a, %d %b %Y %H:%M:%S", gmtime()))
        ]
        directives = ["auto {wlan}".format(wlan=wlan_interface),
                      "iface {wlan} inet static".format(wlan=wlan_interface),
                      "   wireless-essid multi-robot-exp",
                      "   wireless-mode ad-hoc",
                      "   wireless-channel 5",
                      "   wireless-enc off",
                      "   address {addr}".format(addr=ip_address),
                      "   netmask 255.255.255.0",
                      "   broadcast 192.168.100.255",
                      "   gateway 192.168.100.1",
                      "   dns-nameservers 192.168.100.1",
                      ]

        comments = ["#" + comment + os.linesep for comment in comments]
        directives = [directive + os.linesep for directive in directives]

        print("Writing interfaces file...\nContents:")
        for line in comments + directives:
            print("\t" + line)

        if not dry_run:
            f.write("".join(comments))
            f.write("".join(os.linesep))
            f.write("".join(directives))

    print("Successfully written ad-hoc configuration to {}".format(
        "".join([interfaces_dir, os.sep, interfaces_fname])))

    # Source directive in /etc/network/interfaces
    print("Adding corresponding source line in {} file...".format(
        interfaces_main))
    line_exists = False
    source_command = "source-directory interfaces.d"
    with open(interfaces_main, "r") as f:
        lines = f.readlines()

        if source_command in lines:
            print("\nLine already exists, continuing normally...")
            line_exists = True
    if not line_exists:
        with open(interfaces_main, "a") as f:
            if not dry_run:
                f.write(source_command)
            print("OK.")

    # Restart the network interface for the changes to take effect
    print("Restarting {wlan} interface...".format(wlan=wlan_interface))
    if not dry_run:
        call(["ifconfig", wlan_interface, "down"])
        call(["ifconfig", wlan_interface, "up"])



def check_reqs(**kargs):
    """Check if the reuired tools exist in the system."""

    # root access?
    if os.getuid() != 0:
        raise NoRootAccessError()

    # ifconfig installed?
    print("Checking if ifconfig command is available...")
    if call(["which", "ifconfig"]) != 0:
        raise ProgramNotFoundError("ifconfig")
    print("OK")

    # wlan interface exists?
    print("Checking if given interface is valid...")
    if call(["iwconfig", kargs["wlan_interface"]]) != 0:
        print("")
        raise InterfaceNotFoundError(kargs["wlan_interface"])
    print("OK")



if __name__ == "__main__":
    main()


