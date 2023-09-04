#!/usr/bin/env bash

# ★ Note: Hugely inpired by
#  https://github.com/dusty-nv/jetson-containers/blob/bd40fdfc41b912b93de1c33ec91cd0534cf2fcc6/scripts/ros2_build.sh

# this script builds a ROS2 distribution from source
# ROS_DISTRO, ROS_ROOT, ROS_PKG environment variables should be set

echo "ROS2 builder => ROS_DISTRO=$ROS_DISTRO ROS_PKG=$ROS_PKG ROS_ROOT=$ROS_ROOT"

set -e
#set -x

curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list >/dev/null
apt-get update

# install development packages
apt-get install -y --no-install-recommends \
  libbullet-dev \
  libpython3-dev \
  python3-colcon-common-extensions \
  python3-flake8 \
  python3-pip \
  python3-numpy \
  python3-rosdep \
  python3-setuptools \
  python3-vcstool \
  python3-rosinstall-generator \
  libasio-dev \
  libtinyxml2-dev \
  libcunit1-dev

# install some pip packages needed for testing
pip3 install --upgrade --no-cache-dir \
  argcomplete \
  flake8-blind-except \
  flake8-builtins \
  flake8-class-newline \
  flake8-comprehensions \
  flake8-deprecated \
  flake8-docstrings \
  flake8-import-order \
  flake8-quotes \
  pytest-repeat \
  pytest-rerunfailures \
  pytest

# upgrade cmake - https://stackoverflow.com/a/56690743
# this is needed to build some of the ROS2 packages
# use pip to upgrade cmake instead because of kitware's rotating GPG keys:
# https://github.com/dusty-nv/jetson-containers/issues/216
python3 -m pip install --upgrade pip
pip3 install --no-cache-dir scikit-build
pip3 install --upgrade --no-cache-dir --verbose cmake
cmake --version
which cmake

## remove other versions of Python3
## workaround for 'Could NOT find Python3 (missing: Python3_NumPy_INCLUDE_DIRS Development'
#apt purge -y python3.9 libpython3.9* || echo "python3.9 not found, skipping removal"
#ls -ll /usr/bin/python*

# create the ROS_ROOT directory
mkdir -p ${ROS_ROOT}/src
cd ${ROS_ROOT}

# download ROS sources
# https://answers.ros.org/question/325245/minimal-ros2-installation/?answer=325249#post-id-325249
rosinstall_generator --deps --rosdistro ${ROS_DISTRO} ${ROS_PKG} \
  launch_xml \
  launch_yaml \
  launch_testing \
  launch_testing_ament_cmake \
  demo_nodes_cpp \
  demo_nodes_py \
  example_interfaces \
  camera_calibration_parsers \
  camera_info_manager \
  cv_bridge \
  v4l2_camera \
  vision_opencv \
  vision_msgs \
  image_geometry \
  image_pipeline \
  image_transport \
  compressed_image_transport \
  compressed_depth_image_transport \
  >ros2.${ROS_DISTRO}.${ROS_PKG}.rosinstall
cat ros2.${ROS_DISTRO}.${ROS_PKG}.rosinstall
vcs import src <ros2.${ROS_DISTRO}.${ROS_PKG}.rosinstall

# https://github.com/dusty-nv/jetson-containers/issues/181
rm -r ${ROS_ROOT}/src/ament_cmake
git -C ${ROS_ROOT}/src/ clone https://github.com/ament/ament_cmake -b ${ROS_DISTRO}

# skip installation of some conflicting packages
SKIP_KEYS="libopencv-dev libopencv-contrib-dev libopencv-imgproc-dev python-opencv python3-opencv"

echo "--skip-keys $SKIP_KEYS"

# install dependencies using rosdep
rosdep init
rosdep update
rosdep install -y \
  --ignore-src \
  --from-paths src \
  --rosdistro ${ROS_DISTRO} \
  --skip-keys "$SKIP_KEYS"

# build it all - for verbose, see https://answers.ros.org/question/363112/how-to-see-compiler-invocation-in-colcon-build
colcon build \
  --merge-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

# remove build files
rm -rf ${ROS_ROOT}/src
rm -rf ${ROS_ROOT}/logs
rm -rf ${ROS_ROOT}/build
rm ${ROS_ROOT}/*.rosinstall

# cleanup apt
rm -rf /var/lib/apt/lists/*
apt-get clean
