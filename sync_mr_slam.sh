#!/usr/bin/env bash
#
# Fri May 19 16:14:43 EEST 2017, Nikos Koukis
# Sync contents of the mrpt, mr-slam-ws Git repositories with their
# corresonding remotes

# The following aliases should be a priori defined
# mrpt => path to the mrpt local source repository
# MRPT_DIR => path to the mrpt build directory
# catkin_ws => path to the ROS catkin root directory
disp_section_str() {
    local section_header="======================================="
    printf "$1...\n"
    printf "$section_header\n"
    sleep 0.5
}


# assert that the paths exist
if ! [[ -d $mrpt ]]; then
    printf "Environment variable \"mrpt\" is not set. Set this and rerun script.\nExiting..."
    exit 1
fi
if ! [[ -d $catkin_ws ]]; then
    printf "Environment variable \"catkin_ws\" is not set.\nInput the path towards the catkin workspace to sync: "
    read catkin_ws
    # http://stackoverflow.com/a/27485157/2843583
    catkin_ws="${catkin_ws/#\~/$HOME}" # expand possible "~"

    if ! [[ -d $catkin_ws ]]; then
        printf "\"$catkin_ws\" is not a valid path.\nExiting..."
        exit 1
    fi
fi

disp_section_str "Updating MRPT"
cd $mrpt
git pull --ff-only

disp_section_str "Running make on MRPT"
make -C $MRPT_DIR -j4 -l4

disp_section_str "Updating catkin_ws"
cd $catkin_ws/src
git pull
git submodule sync
git submodule update --recursive

branch=devel
repo=mrpt_navigation
disp_section_str "Checking out ${repo}/${branch}"
cd ${repo}
git checkout ${branch}
git pull --ff-only

cd ..

branch=graphslam-devel
repo=mrpt_slam
disp_section_str "Checking out ${repo}/${branch}"
cd ${repo}
git checkout ${branch}
git pull --ff-only

disp_section_str "Running catkin_make"
catkin_make -C $catkin_ws -j4 -l4


echo "All set. Exiting..."
exit 0
