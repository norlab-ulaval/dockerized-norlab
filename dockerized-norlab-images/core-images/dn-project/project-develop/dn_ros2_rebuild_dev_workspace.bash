#!/bin/bash -i

#set -e # exit script if any statement returns a non-true return value
#set -v

cd "${DN_DEV_WORKSPACE}"

# Install dependencies
sudo apt-get update
# rosdep install: looks at all the packages in the src directory and tries to find and install their dependencies on your platform
rosdep install --from-paths ./src --ignore-packages-from-source --rosdistro "${ROS_DISTRO}" --default-yes

## colcon build step: rebuild everything in the catkin workspace DN_DEV_WORKSPACE
source "/opt/ros/${ROS_DISTRO}/setup.bash"
colcon build --symlink-install
# --merge-install
source "${DN_DEV_WORKSPACE}/install/setup.bash"



# (NICE TO HAVE) todo:implement >> Terminal splash screen

# ToDo: refactor `echo "  It should include the directory you're in: ${DN_DEV_WORKSPACE}/src:/opt/ros/melodic/share"`
echo
echo "  Make sure your workspace is properly overlayed by the setup script by checking the ROS_PACKAGE_PATH environment variable. "
echo "  It should include the directory you're in: ${DN_DEV_WORKSPACE}/src:/opt/ros/melodic/share"
echo
printenv | grep -i -e ROS -e MASTER -e HOSTNAME -e DS_ -e DN_
echo

cd /

# (Priority) todo:refactor (ref task NLSAR-222 🛠→ setupEnv*.sh scripts for deployement case)
#echo "
#  Done building norlab-ros.
#  Finale step:
#    1. source your norlab-ros environment
#      # source ~/.bashrc
#    2. check if norlab-ros was properly sourced
#      # printenv | grep AR_
#  or open a new terminal in the container
#    $ bash /dn_attach.bash <container name>
#"


## # (ICEBOXED) todo:assessment >> exec bash does not behave like expected!
#exec bash -i
