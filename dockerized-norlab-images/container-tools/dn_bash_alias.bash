#!/bin/bash

alias tree='pwd && tree -L 1'
alias tree2='tree -L 2'

# Dockerized-NorLab aliases (from dependencies img)
alias dn_info='source /dockerized-norlab/dockerized-norlab-images/container-tools/dn_info.bash'

# Dockerized-NorLab aliases (from develop img)
alias dn_ros2_rebuild_dev_workspace='source /dockerized-norlab/project/project-develop/dn_ros2_rebuild_dev_workspace.bash'
alias dn_fetch_ros_env_variables='source /dockerized-norlab/project/project-develop/dn_fetch_ros_env_variables.bash'
alias dn_expose_container_env_variables='source /dockerized-norlab/project/project-develop/dn_expose_container_env_variables.bash'
