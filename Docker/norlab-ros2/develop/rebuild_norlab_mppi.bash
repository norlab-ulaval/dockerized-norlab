#!/bin/bash -i

#set -e # exit script if any statement returns a non-true return value
#set -v

cd "${DS_DEV_WORKSPACE}"

# Install dependencies
sudo apt-get update
# rosdep install: looks at all the packages in the src directory and tries to find and install their dependencies on your platform
rosdep install --from-paths ./src --ignore-packages-from-source --rosdistro ${ROS_DISTRO} --default-yes

# colcon build step: rebuild everything in the catkin workspace DS_DEV_WORKSPACE
source /opt/ros/${ROS_DISTRO}/setup.bash
colcon build
source ${DS_DEV_WORKSPACE}/install/setup.bash


# (Priority) todo:refactor (ref task NLSAR-222 🛠→ setupEnv*.sh scripts for deployement case)
## Environment setup
#norlab_mppi_env_setup="${DS_DEV_WORKSPACE}/src/${DS_TARGET_PROJECT_SRC_REPO}/mppi_util/setupEnv${DS_HOST_TYPE}.sh"
#echo "source ${norlab_mppi_env_setup}" >> ~/.bashrc

# (NICE TO HAVE) todo:implement >> Terminal splash screen

echo
echo "  Make sure your workspace is properly overlayed by the setup script by checking the ROS_PACKAGE_PATH environment variable. "
echo "  It should include the directory you're in: ${DS_DEV_WORKSPACE}/src:/opt/ros/melodic/share"
echo
printenv | grep -e ROS -e MASTER -e HOSTNAME -e DS_
echo

cd /

# (Priority) todo:refactor (ref task NLSAR-222 🛠→ setupEnv*.sh scripts for deployement case)
#echo "
#  Done building norlab-ros2.
#  Finale step:
#    1. source your norlab-ros2 environment
#      # source ~/.bashrc
#    2. check if norlab-ros2 was properly sourced
#      # printenv | grep AR_
#  or open a new terminal in the container
#    $ bash /ds_attach.bash <container name>
#"


## # (ICEBOXED) todo:assessment >> exec bash does not behave like expected!
#exec bash -i
