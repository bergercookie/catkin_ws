Tue Oct 4 11:31:01 EEST 2016, Nikos Koukis

Directory contains scripts that are used for the initial networking setup of
the agents in the multi-robot SLAM experiment.

More specifically:
- /etc/networking/interface files for each one of the agents, for setting up
    an Ad-Hoc network
- TODO


# Installation instructions:

- Add line to /etc/network/interfaces: "source-directory interfaces.d"
- Copy this file to /etc/network/interfaces.d/multi_robot_setup
- Restart network - [sudo] ifconfig wlan6 down && [sudo] ifconfig wlan6 up

