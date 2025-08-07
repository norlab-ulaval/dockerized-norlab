#!/bin/bash

function dna::setup_python_paths() {
  local python_version
  local ros_python_path
  local site_packages_dir
  local method=${1=-2}

  # Get Python version dynamically
  python_version=$(python3 -c "import sys; print(f'python{sys.version_info.major}.{sys.version_info.minor}')")

  if [[ -n "${ROS_DISTRO}" ]]; then
    ros_python_path="/opt/ros/${ROS_DISTRO}/lib/${python_version}/site-packages"

    # Method 1: Environment variable
    if [[ -d "${ros_python_path}" ]] && [[ ${method} -eq 1 ]]; then
      export PYTHONPATH="${ros_python_path}:${PYTHONPATH}"
      echo -e "\033[1;32m[DNA]\033[0m Added to PYTHONPATH: ${ros_python_path}"
    fi

    # Method 2: Create .pth file for permanent addition
    site_packages_dir=$(python3 -c "import site; print(site.getsitepackages()[0])" 2>/dev/null)
    if [[ -d "${site_packages_dir}" && -w "${site_packages_dir}" ]] && [[ ${method} -eq 2 ]]; then
      echo "${ros_python_path}" > "${site_packages_dir}/dna_ros_${ROS_DISTRO}.pth"
      echo -e "\033[1;32m[DNA]\033[0m Created .pth file: ${site_packages_dir}/dna_ros_${ROS_DISTRO}.pth"
    fi

    # Method 3: Create sitecustomize.py for automatic detection
    if [[ -d "${site_packages_dir}" && -w "${site_packages_dir}" ]] && [[ ${method} -eq 3 ]]; then
      cat > "${site_packages_dir}/dna_sitecustomize.py" << EOF
import sys
import os

# DNA ROS path auto-detection
ros_distro = os.environ.get('ROS_DISTRO')
if ros_distro:
    python_version = f"python{sys.version_info.major}.{sys.version_info.minor}"
    ros_path = f"/opt/ros/{ros_distro}/lib/{python_version}/site-packages"

    if os.path.exists(ros_path) and ros_path not in sys.path:
        sys.path.insert(0, ros_path)
EOF
    fi
  fi

  return 0
}

dna::setup_python_paths "$@" || exit 1
