#!/bin/bash

set -e
source /import_dockerized_norlab_container_tools.bash
n2st::print_msg "Execute ${MSG_DIMMED_FORMAT}entrypoint.norlab-stack.bash${MSG_END_FORMAT}"


#n2st::print_msg "sourcing /opt/ros/${ROS_DISTRO}/setup.bash from entrypoint.norlab-stack.bash"
#source "/opt/ros/${ROS_DISTRO}/setup.bash"
#n2st::print_msg "sourcing ${DN_DEV_WORKSPACE}/install/setup.bash from entrypoint.norlab-stack.bash"
#source "${DN_DEV_WORKSPACE}/install/setup.bash"

n2st::print_msg "Launching mapper ros2 nodes in the background"
nohup ros2 launch mapper_config_template mapper.launch &

n2st::print_msg "Execute the following lines in the terminal:

      ${MSG_DIMMED_FORMAT}rviz2 -d ~/demo/config.rviz${MSG_END_FORMAT}
      ${MSG_DIMMED_FORMAT}ros2 bag play \${DN_DEV_WORKSPACE}/demo/demo${MSG_END_FORMAT}

  See ${MSG_DIMMED_FORMAT}https://github.com/norlab-ulaval/Norlab_wiki/wiki/Tutorials:-3D-Mapping-(ros2)${MSG_END_FORMAT} for details.

  PS Dont forget to feed the robot on your way out.
"

exec "$@"
