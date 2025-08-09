#!/bin/bash

function dna::setup_ros2_python_paths() {
  local method=${1:-"pth"} # Either "env", "pth" or "syscustom"
  local python_version
  local ros_python_path1
  local ros_python_path2
  local site_packages_dir
  local pythonpath_pre_ros2_source
  local pythonpath_post_ros2_source

  # Get Python version dynamically
  python_version=$(python3 -c "import sys; print(f'python{sys.version_info.major}.{sys.version_info.minor}')")

  if [[ -n "${ROS_DISTRO}" ]]; then
    ros_python_path1="/opt/ros/${ROS_DISTRO}/lib/${python_version}/site-packages"
    ros_python_path2="/opt/ros/${ROS_DISTRO}/local/lib/${python_version}/dist-packages"

    # Method 1: Environment variable
    if [[ -d "${ros_python_path1}" ]] && [[ ${method} == "env" ]]; then
      export PYTHONPATH="${ros_python_path1}:${ros_python_path2}:${PYTHONPATH}"
      echo -e "\033[1;33m[DN]\033[0m Added to PYTHONPATH: ${ros_python_path1} and ${ros_python_path2}"
    fi

    # Method 2: Create .pth file for permanent addition.
    # Note: PyCharm ssh remote interpreter configuration rely on .pth to populate the interpreter
    #       python path, meaning we don't have to manualy add them via the gui.
    site_packages_dir=$(python3 -c "import site; print(site.getsitepackages()[0])" 2>/dev/null)
    if [[ -d "${site_packages_dir}" ]] && [[ ${method} == "pth" ]]; then
      # Capture Python path before sourcing ROS2
      pythonpath_pre_ros2_source=$(python3 -c "import sys; print(sys.path)" 2>/dev/null)
      
      # Check if dn::source_ros2 function is available
      test -n "$( declare -f dn::source_ros2 )" || { echo -e "\033[1;31m[DN error]\033[0m The DN lib is not loaded!" 1>&2 && exit 1; }
      
      # Capture Python path after sourcing ROS2
      pythonpath_post_ros2_source=$(dn::source_ros2 >/dev/null && python3 -c "import sys; print(sys.path)")
      
      # Compute the difference between the two Python paths
      local pythonpath_difference
      pythonpath_difference=$(python3 -c "
import sys
pre_paths = ${pythonpath_pre_ros2_source}
post_paths = ${pythonpath_post_ros2_source}

# Find paths that are in post_paths but not in pre_paths
difference = []
for path in post_paths:
    if path not in pre_paths:
        difference.append(path)

# Print each path on a separate line
for path in difference:
    print(path)
" 2>/dev/null)

      # Write the difference to the .pth file
      if [[ -n "${pythonpath_difference}" ]]; then
        echo "${pythonpath_difference}" > "${site_packages_dir}/dna_ros_${ROS_DISTRO}.pth"
        echo -e "\033[1;33m[DN]\033[0m Created .pth file: ${site_packages_dir}/dna_ros_${ROS_DISTRO}.pth with ROS2-added Python paths"
      else
        # Fallback to the original hardcoded paths if difference computation fails
        (
          echo "${ros_python_path1}"
          echo "${ros_python_path2}"
        ) > "${site_packages_dir}/dna_ros_${ROS_DISTRO}.pth"
        echo -e "\033[1;33m[DN warning]\033[0m No difference detected, using fallback paths: ${ros_python_path1} and ${ros_python_path2}"
      fi
    fi

    # Method 3: Create sitecustomize.py for automatic detection
    if [[ -d "${site_packages_dir}" ]] && [[ ${method} == "syscustom" ]]; then
      cat > "${site_packages_dir}/dna_sitecustomize.py" << EOF
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
  fi

  return 0
}

dna::setup_ros2_python_paths "$@" || exit 1
