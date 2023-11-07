#!/usr/bin/env bats
#
# Usage in docker container
#   $ REPO_ROOT=$(pwd) && RUN_TESTS_IN_DIR='tests'
#   $ docker run -it --rm -v "$REPO_ROOT:/code" bats/bats:latest "$RUN_TESTS_IN_DIR"
#
#   Note: "/code" is the working directory in the bats official image
#
# bats-core ref:
#   - https://bats-core.readthedocs.io/en/stable/tutorial.html
#   - https://bats-core.readthedocs.io/en/stable/writing-tests.html
#   - https://opensource.com/article/19/2/testing-bash-bats
#       ↳ https://github.com/dmlond/how_to_bats/blob/master/test/build.bats
#
# Helper library: 
#   - https://github.com/bats-core/bats-assert
#   - https://github.com/bats-core/bats-support
#   - https://github.com/bats-core/bats-file
#

BATS_HELPER_PATH=/usr/lib/bats
if [[ -d ${BATS_HELPER_PATH} ]]; then
  load "${BATS_HELPER_PATH}/bats-support/load"
  load "${BATS_HELPER_PATH}/bats-assert/load"
  load "${BATS_HELPER_PATH}/bats-file/load"
  load "bats_testing_tools/bats_helper_functions"
  #load "${BATS_HELPER_PATH}/bats-detik/load" # << Kubernetes support
else
  echo -e "\n[\033[1;31mERROR\033[0m] $0 path to bats-core helper library unreachable at \"${BATS_HELPER_PATH}\"!"
  echo '(press any key to exit)'
  read -r -n 1
  exit 1
fi

# ====Setup========================================================================================

TESTED_FILE="dn_expose_container_env_variables.bash"
TESTED_FILE_PATH="dockerized-norlab-images/core-images/dn-project/project-develop"

setup_file() {
  BATS_DOCKER_WORKDIR=$(pwd) && export BATS_DOCKER_WORKDIR

  mkdir -p /dockerized-norlab/utilities/norlab-shell-script-tools

  cp  /code/dockerized-norlab/.env.dockerized-norlab /dockerized-norlab/
  cp -r /code/dockerized-norlab/utilities/norlab-shell-script-tools /dockerized-norlab/utilities/

  mkdir -p /ros2_ws_mock/install/
  touch /ros2_ws_mock/install/local_setup.bash

  set -o allexport
  source /dockerized-norlab/.env.dockerized-norlab || return 1
  set +o allexport

  assert_file_exist /dockerized-norlab/utilities/norlab-shell-script-tools/src/function_library/prompt_utilities.bash
  assert_file_exist /ros2_ws_mock/install/local_setup.bash

#  pwd >&3 && tree -L 1 -a -hug >&3
  printenv >&3
}

setup() {
  cd "$TESTED_FILE_PATH" || exit

  # PRE CONDITION: Variable need to be set prior for this script
  export DN_DEV_WORKSPACE=/ros2_ws_mock
  mkdir -p /dn_container_env_variable

  source ./$TESTED_FILE
}

# ====Teardown=====================================================================================

teardown() {
  bats_print_run_env_variable_on_error
}

teardown_file() {
    rm -r -f /dockerized-norlab/
    rm -r -f /ros2_ws_mock/
    assert_dir_not_exist /dockerized-norlab/
    assert_dir_not_exist /ros2_ws_mock/
    unset $DN_DEV_WORKSPACE
}

# ====Local helper fonction========================================================================

function export_DN_container_env() {
    # Muting on purpose so that it does not clash with bats framework:
    # - (critical) PATH
    # - (seams ok) PYTHONPATH
    # - HOSTNAME
    # - LD_PRELOAD

    # ROS related env
    export ROS_DISTRO=foxy
    export ROS_ROOT=/opt/ros/foxy
    export ROS_VERSION=2
    export ROS_PYTHON_VERSION=3
    export ROS_DOMAIN_ID=1
    export ROS_LOCALHOST_ONLY=0
    export PYTHONPATH=/opt/ros/foxy/lib/python3.8/site-packages:/opt/ros/foxy/install/lib/python3.8/site-packages
    export AMENT_PREFIX_PATH=/opt/ros/foxy:/opt/ros/foxy/install
    export CMAKE_PREFIX_PATH=/opt/ros/foxy/install
    export COLCON_PREFIX_PATH=/opt/ros/foxy/install
    export PKG_CONFIG_PATH=/opt/ros/foxy/install/lib/aarch64-linux-gnu/pkgconfig:/opt/ros/foxy/install/lib/pkgconfig
    export LD_LIBRARY_PATH=/opt/ros/foxy/lib/aarch64-linux-gnu:/opt/ros/foxy/lib:/opt/ros/foxy/install/opt/yaml_cpp_vendor/lib:/opt/ros/foxy/install/lib:/usr/local/cuda/lib64:/usr/local/cuda/lib64:
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    export OPENBLAS_CORETYPE=ARMV8

    # Cuda related env
    export CUDA_HOME=/usr/local/cuda
    export NVIDIA_VISIBLE_DEVICES=all
    export NVIDIA_DRIVER_CAPABILITIES=graphics
#    export LD_PRELOAD=/usr/lib/aarch64-linux-gnu/libgomp.so.1

    # Display forwarding related env
    export DISPLAY=host.docker.internal:0
    export LIBGL_ALWAYS_INDIRECT=1
    export QT_X11_NO_MITSHM=1

    # Dockerized-NorLab related env
    export DN_DEV_WORKSPACE=/ros2_ws_mock
    export DN_PROJECT_PATH=/ros2_ws_mock/src/f1tenth-redleader-controller
    export DN_CONTAINER_NAME=IamProject-test
    export DN_ACTIVATE_POWERLINE_PROMT=true
    export DN_GDB_SERVER_PORT=7777
    export DN_PROJECT_GIT_NAME=dockerized-norlab-project-mock
    export DN_SSH_SERVER_PORT=2222
    export DN_SSH_SERVER_USER=pycharm-debugger
    export DN_PROJECT_GIT_DOMAIN=norlab-ulaval

    export PYTHONUNBUFFERED=1
  }

function show_DN_container_env() {
      printenv | grep \
         -e ROS_DISTRO \
         -e ROS_ROOT \
         -e ROS_VERSION \
         -e ROS_PYTHON_VERSION \
         -e ROS_DOMAIN_ID \
         -e ROS_LOCALHOST_ONLY \
         -e AMENT_PREFIX_PATH \
         -e CMAKE_PREFIX_PATH \
         -e COLCON_PREFIX_PATH \
         -e PKG_CONFIG_PATH \
         -e PYTHONPATH \
         -e LD_LIBRARY_PATH \
         -e RMW_IMPLEMENTATION \
         -e OPENBLAS_CORETYPE \
         -e CUDA_HOME \
         -e NVIDIA_VISIBLE_DEVICES \
         -e NVIDIA_DRIVER_CAPABILITIES \
#         -e LD_PRELOAD \
         -e DISPLAY \
         -e LIBGL_ALWAYS_INDIRECT \
         -e QT_X11_NO_MITSHM \
         -e DN_DEV_WORKSPACE \
         -e DN_PROJECT_PATH \
         -e DN_CONTAINER_NAME \
         -e DN_ACTIVATE_POWERLINE_PROMT \
         -e DN_GDB_SERVER_PORT \
         -e DN_PROJECT_GIT_NAME \
         -e DN_SSH_SERVER_PORT \
         -e DN_SSH_SERVER_USER \
         -e DN_PROJECT_GIT_DOMAIN
}

# ====Test casses==================================================================================


@test "running $TESTED_FILE from ok cwd › expect pass" {
  cd "${BATS_DOCKER_WORKDIR}/${TESTED_FILE_PATH}"
  run source ./$TESTED_FILE
  assert_success
}

@test "running $TESTED_FILE › var unset › expect failure" {
  unset DN_DEV_WORKSPACE
  run source ./$TESTED_FILE
  assert_output --partial "Variable DN_DEV_WORKSPACE unset"
}

@test "running $TESTED_FILE › check file/dir created › expect pass" {
  run source ./$TESTED_FILE
  tree -L 2 -a

  assert_dir_exist "/dn_container_env_variable"

  DN_CONTAINER_EXPOSE_ENV_PATH="/dn_container_env_variable/.env.dn_expose_${DN_CONTAINER_NAME}"
  assert_file_exist $DN_CONTAINER_EXPOSE_ENV_PATH

}

@test "running $TESTED_FILE › check env var agregated › expect pass" {

  export_DN_container_env
  run source ./$TESTED_FILE
#  tree -L 1 -a

  DN_CONTAINER_EXPOSE_ENV_PATH="/dn_container_env_variable/.env.dn_expose_${DN_CONTAINER_NAME}"
  assert_file_exist $DN_CONTAINER_EXPOSE_ENV_PATH
  assert_file_not_empty $DN_CONTAINER_EXPOSE_ENV_PATH
#  more $DN_CONTAINER_EXPOSE_ENV_PATH  >&3


  assert_file_contains $DN_CONTAINER_EXPOSE_ENV_PATH "ROS_VERSION=2"
  assert_file_contains $DN_CONTAINER_EXPOSE_ENV_PATH "NVIDIA_VISIBLE_DEVICES=all"
  assert_file_contains $DN_CONTAINER_EXPOSE_ENV_PATH "LIBGL_ALWAYS_INDIRECT=1"
  assert_file_contains $DN_CONTAINER_EXPOSE_ENV_PATH "PKG_CONFIG_PATH=/opt/ros/foxy/install/lib/aarch64-linux-gnu/pkgconfig:/opt/ros/foxy/install/lib/pkgconfig"
  assert_file_contains $DN_CONTAINER_EXPOSE_ENV_PATH "ROS_PYTHON_VERSION=3"
  assert_file_contains $DN_CONTAINER_EXPOSE_ENV_PATH "NVIDIA_DRIVER_CAPABILITIES=graphics"
  assert_file_contains $DN_CONTAINER_EXPOSE_ENV_PATH "ROS_DOMAIN_ID=1"
  assert_file_contains $DN_CONTAINER_EXPOSE_ENV_PATH "DN_DEV_WORKSPACE=/ros2_ws_mock"
  assert_file_contains $DN_CONTAINER_EXPOSE_ENV_PATH "DN_PROJECT_PATH=/ros2_ws_mock/src/f1tenth-redleader-controller"
  assert_file_contains $DN_CONTAINER_EXPOSE_ENV_PATH "DN_CONTAINER_NAME=IamProject-test"
  assert_file_contains $DN_CONTAINER_EXPOSE_ENV_PATH "AMENT_PREFIX_PATH=/opt/ros/foxy:/opt/ros/foxy/install"
  assert_file_contains $DN_CONTAINER_EXPOSE_ENV_PATH "CMAKE_PREFIX_PATH=/opt/ros/foxy/install"
  assert_file_contains $DN_CONTAINER_EXPOSE_ENV_PATH "COLCON_PREFIX_PATH=/opt/ros/foxy/install"
  assert_file_contains $DN_CONTAINER_EXPOSE_ENV_PATH "DN_ACTIVATE_POWERLINE_PROMT=true"
  assert_file_contains $DN_CONTAINER_EXPOSE_ENV_PATH "OPENBLAS_CORETYPE=ARMV8"
  assert_file_contains $DN_CONTAINER_EXPOSE_ENV_PATH "DN_GDB_SERVER_PORT=7777"
  assert_file_contains $DN_CONTAINER_EXPOSE_ENV_PATH "DN_PROJECT_GIT_NAME=dockerized-norlab-project-mock"
  assert_file_contains $DN_CONTAINER_EXPOSE_ENV_PATH "DISPLAY=host.docker.internal:0"
  assert_file_contains $DN_CONTAINER_EXPOSE_ENV_PATH "DN_SSH_SERVER_PORT=2222"
  assert_file_contains $DN_CONTAINER_EXPOSE_ENV_PATH "ROS_LOCALHOST_ONLY=0"
  assert_file_contains $DN_CONTAINER_EXPOSE_ENV_PATH "DN_SSH_SERVER_USER=pycharm-debugger"
  assert_file_contains $DN_CONTAINER_EXPOSE_ENV_PATH "ROS_ROOT=/opt/ros/foxy"
  assert_file_contains $DN_CONTAINER_EXPOSE_ENV_PATH "DN_PROJECT_GIT_DOMAIN=norlab-ulaval"
  assert_file_contains $DN_CONTAINER_EXPOSE_ENV_PATH "ROS_DISTRO=foxy"
  assert_file_contains $DN_CONTAINER_EXPOSE_ENV_PATH "RMW_IMPLEMENTATION=rmw_fastrtps_cpp"
  assert_file_contains $DN_CONTAINER_EXPOSE_ENV_PATH "QT_X11_NO_MITSHM=1"

  assert_file_contains $DN_CONTAINER_EXPOSE_ENV_PATH "LD_LIBRARY_PATH=/opt/ros/foxy/lib/aarch64-linux-gnu:/opt/ros/foxy/lib:/opt/ros/foxy/install/opt/yaml_cpp_vendor/lib:/opt/ros/foxy/install/lib:/usr/local/cuda/lib64:/usr/local/cuda/lib64:"
}

@test "running $TESTED_FILE › check PYTHONPATH › expect pass" {

  echo "PYTHONPATH=${PYTHONPATH}"
#  echo "LD_LIBRARY_PATH=${LD_LIBRARY_PATH}"

  export_DN_container_env
  run source ./$TESTED_FILE
#  tree -L 1 -a

  DN_CONTAINER_EXPOSE_ENV_PATH="/dn_container_env_variable/.env.dn_expose_${DN_CONTAINER_NAME}"
  assert_file_exist $DN_CONTAINER_EXPOSE_ENV_PATH
  assert_file_not_empty $DN_CONTAINER_EXPOSE_ENV_PATH
  more $DN_CONTAINER_EXPOSE_ENV_PATH  >&3

  assert_file_contains $DN_CONTAINER_EXPOSE_ENV_PATH "PYTHONPATH=\${PYTHONPATH}:/opt/ros/foxy/lib/python3.8/site-packages:/opt/ros/foxy/install/lib/python3.8/site-packages"

}

#@test 'fail()' {
#  fail 'this test always fails'
#}

