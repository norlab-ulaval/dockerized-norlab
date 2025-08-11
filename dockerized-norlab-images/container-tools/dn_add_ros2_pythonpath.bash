#!/bin/bash
# =================================================================================================
# Set ROS2 related python interpreter sys.path
#
# Usage:
#   $ source dn_add_ros2_pythonpath.bash [pth|env|syscustom]
#
# Options:
#   pth                 Create .pth file for permanent addition (default)
#   env                 Set PYTHONPATH environment variable
#   syscustom           Create sitecustomize.py for automatic detection
#
# Global:
#   read ROS_DISTRO
#   read/write PYTHONPATH
#   read DN_SHOW_DEBUG_INFO
#
# =================================================================================================
#DN_SHOW_DEBUG_INFO=false # (CRITICAL) ToDo: on task end >> mute this line ←

function dn::add_ros2_python_paths() {
  local method=${1:-"pth"} # Either "env", "pth" or "syscustom"
  local python_version
  local ros_python_path1
  local ros_python_path2
  local site_packages_dir
  local pythonpath_pre_ros2_source
  local pythonpath_post_ros2_source

  # ....Pre-condition..............................................................................
  test -n "$( declare -f n2st::print_msg )" || { echo -e "\033[1;31m[DN error]\033[0m The N2ST lib is not loaded!" 1>&2 && return 1; }
  test -n "$( declare -f dn::source_ros2 )" || { n2st::print_msg_error "The DN lib is not loaded!"; return 1; }

  if [[ -z "${ROS_DISTRO}" ]]; then
    n2st::print_msg_warning "ROS_DISTRO is not set! Skipping dn::add_ros2_python_paths." 1>&2
    return 0
  fi

  # ....Setup......................................................................................
  python_version="python$(n2st::which_python3_version)"
  ros_python_path1="/opt/ros/${ROS_DISTRO}/lib/${python_version}/site-packages"
  ros_python_path2="/opt/ros/${ROS_DISTRO}/local/lib/${python_version}/dist-packages"
  site_packages_dir=$(python3 -c "import site; print(site.getsitepackages()[0])" 2>/dev/null)

  # ....Begin......................................................................................
  if [[ -d "${site_packages_dir}" ]] && [[ ${method} == "pth" ]]; then
    # Method 1: Create .pth file for permanent addition.
    # Note: PyCharm ssh remote interpreter configuration rely on .pth to populate the interpreter
    #       python path, meaning we don't have to manualy add them via the gui.

    # Capture Python path before sourcing ROS2
    pythonpath_pre_ros2_source=$(python3 -c "import sys; print(sys.path)" 2>/dev/null)

    # Capture Python path after sourcing ROS2
    pythonpath_post_ros2_source=$(dn::source_ros2 >/dev/null && python3 -c "import sys; print(sys.path)")

    # Compute the difference between the two Python paths
    local pythonpath_difference
    pythonpath_difference=$(python3 -c "
import sys
import os

pre_paths = ${pythonpath_pre_ros2_source}
post_paths = ${pythonpath_post_ros2_source}

# Find paths that are in post_paths but not in pre_paths
difference = []
for path in post_paths:
  if path not in pre_paths:
      difference.append(path)

difference.append('${ros_python_path1}')
difference.append('${ros_python_path2}')

# Print each path on a separate line
for each_path in difference:
  if os.path.exists(each_path):
      print(each_path)

" 2>/dev/null)

    if [[ ${DN_SHOW_DEBUG_INFO} == true ]]; then
      echo -e "${MSG_DIMMED_FORMAT}
pythonpath_pre_ros2_source:\n${pythonpath_pre_ros2_source}\n
pythonpath_post_ros2_source:\n${pythonpath_post_ros2_source}\n
pythonpath_difference:\n${pythonpath_difference}\n
${MSG_END_FORMAT}"
    fi

    # Write the difference to the .pth file
    if [[ -n "${pythonpath_difference}" ]]; then
      echo "${pythonpath_difference}" > "${site_packages_dir}/dna_ros_${ROS_DISTRO}.pth"
      n2st::print_msg "Created .pth file: ${site_packages_dir}/dna_ros_${ROS_DISTRO}.pth with ROS2-added Python paths"
    else
      # Fallback to the original hardcoded paths if difference computation fails
      (
        echo "${ros_python_path1}"
        echo "${ros_python_path2}"
      ) > "${site_packages_dir}/dna_ros_${ROS_DISTRO}.pth"
      n2st::print_msg_warning "No difference detected, using fallback paths: ${ros_python_path1} and ${ros_python_path2}"
    fi

  elif [[ ${method} == "env" ]]; then
    # Method 2: Environment variable

    if [[ -d ${ros_python_path1} ]]; then
      export PYTHONPATH="${ros_python_path1}:${PYTHONPATH}"
      n2st::print_msg "Added ${ros_python_path1} to PYTHONPATH"
    else
      n2st::print_msg_warning "${ros_python_path1} is unreachable skip adding to PYTHONPATH"
    fi

    if [[ -d ${ros_python_path2} ]]; then
      export PYTHONPATH="${ros_python_path2}:${PYTHONPATH}"
      n2st::print_msg "Added ${ros_python_path2} to PYTHONPATH"
    else
      n2st::print_msg_warning "${ros_python_path2} is unreachable skip adding to PYTHONPATH"
    fi

  elif [[ -d "$(python3 -m site --user-site)" ]] && [[ ${method} == "syscustom" ]]; then
    # Method 3: Create sitecustomize.py for automatic detection

    cat >> "${site_packages_dir}/sitecustomize.py" << EOF

import sys
import os

# DNA ROS path auto-detection
ros_distro = os.environ.get('ROS_DISTRO')
if ros_distro:
  python_version = f"python{sys.version_info.major}.{sys.version_info.minor}"
  ros_path = [
      f"/opt/ros/{ros_distro}/lib/{python_version}/site-packages",
      f"/opt/ros/{ros_distro}/local/lib/{python_version}/dist-packages"
      ]

  for each_path in ros_path:
    if os.path.exists(each_path) and each_path not in sys.path:
        sys.path.insert(0, each_path)
EOF
  fi

  return 0
}

# ::::Main:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
if [[ "${BASH_SOURCE[0]}" = "$0" ]]; then
  # This script is being run, ie: __name__="__main__"
  echo -e "\033[1;31m[DN error]\033[0m This script must be sourced!
        i.e.: $ source $(basename "$0")" 1>&2
  exit 1
else
  # This script is being sourced, ie: __name__="__source__"

  # Default method: "pth"
  dn::add_ros2_python_paths "$@" || { echo -e "\033[1;31m[DN error]\033[0m dn::add_ros2_python_paths exited with error!" 1>&2; exit 1; }
fi

