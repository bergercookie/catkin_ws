# Software setup example

Current section includes a complete example on downloading, setting up and
configuring the graphSLAM algorithm.

## System prerequisites

Following is information on the machine, as
well as on important 3rd party software on the latter, the installation was
tested on:

- **Computer Model:** MacBook Pro 8,1
- **Processor Type:** Intel Core i5 @2,3 GHz
- **Operating System:** Ubuntu 14.04

    It's important that users stick with Ubuntu 14.04 instead
    of the more recent 16.04 variant. For the latter OSRF supports
    binary installation of only the \texttt{ROS Kinetic}, with which in
    turn most of the utilised ROS packages won't work.

- Versions of notable software

    + **gcc, g++ version:** 4.8.4

    + **Eigen version:**

        Minimum required Eigen version is 3.2.0. If your version is lower than
        that, either upgrade, or use the Eigen embedded version found in
        $path_to_mrpt_src/otherlibs/eigen3. For the latter use the following Eigen
        directive when compiling
        `cmake $path_to_mrpt_src -DEIGEN_USE_EMBEDDED_VERSION:BOOL=ON`. More on
        this can be found [here](https://github.com/MRPT/mrpt/issues/325)


## Configuration steps

1. Install MRPT from source as explained in the Appendix section of [this
document](http://147.102.51.10:3000/bergercookie/mr-slam-thesis-text/src/master/report.pdf).

2. Setup ROS Jade as instructed in the [official
   documentation](http://wiki.ros.org/jade/Installation/). Installing ROS
   Indigo can also be a viable choice but bare in mind that single- and
   multi-robot simulations were run and tested exclusively on Jade.

   During the time of writing, the `ros-jade-desktop-full` was used. If you aren't
   interested in running graphSLAM in the Gazebo simulator, you probably can go
   for the `ros-jade-desktop` variant.

3. Clone repository of used ROS
   packages **recursively** (yes, we do use Git submodules).
   The stable (and fully tested) version of the repository can be found
   [here](http://controlsystemslab.gr/code/bergercookie/mr_slam_ws) while the
   up-to-date and more active version
   [here](https://github.com/bergercookie/catkin_ws)

```bash

$ mkdir -p ~/catkin_ws/src # modify this in case of already existing catkin ws
$ git clone --recursive https://github.com/bergercookie/catkin_ws catkin_ws/src # modify in case you want the stable version

# in case previous command fails (e.g. due to an internet connection problem)
# just continue with the setup of submodules:
$ cd ~/catkin_ws/src
$ git submodule update --init

```

4. Initialize catkin workspace - Modify .bashrc accordingly

```bash
# Initialize this catkin workspace - Link to /opt/*/CMakeLists file
$ ~/catkin_ws/src$ catkin_init_workspace

# Add environment-setting lines in your .bashrc
$ echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
$ echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc

# reload bashrc
$ source ~/.bashrc

```

5. Install required apt packages:

    The following should be installed in the system for the `catkin_make`
    compilation to be successful:

    - libspnav-dev
    - libv4l-dev
    - libaria-dev

6. Test that the MRPT graphslam-engine application works - Execute using the
   provided sample datasets as described
   [here](http://www.mrpt.org/list-of-mrpt-apps/application-graphslamengine/).
   A sample command to execute would be the following:

```bash
$MRPT_DIR/bin/graphslam-engine \
    -i $mrpt/share/mrpt/config_files/graphslam-engine/odometry_2DRangeScans.ini \
    -r $mrpt/share/mrpt/datasets/graphslam-engine-demos/action_observations_map/simul.rawlog \
    -g $mrpt/share/mrpt/datasets/graphslam-engine-demos/action_observations_map/simul.rawlog.GT.txt \
    --2d
```

7. Test that the mrpt_graphslam_2d ROS wrapper works as expected, use the
   instructions provided [in the
   mrpt_graphslam_2d](http://github.com/bergercookie/mrpt_graphslam_2d)
   package.

----------

## Notes

In case you encounter any problem during the aforementioned process or you find
part of the instructions inaccurate, [open a Github
issue](https://github.com/bergercookie/catkin_ws/issues)

