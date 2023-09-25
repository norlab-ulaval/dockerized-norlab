
# Execute ros sourcing system wide instead of in .bashrc which is not executed
# in non-interactive shell such as in pycharm-debugger
ROS_ENV_SETUP="/opt/ros/${ROS_DISTRO}/install/setup.bash"
echo "sourcing ${ROS_ENV_SETUP}"
source "${ROS_ENV_SETUP}"
