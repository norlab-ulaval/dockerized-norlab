#!/usr/bin/env bats
# =================================================================================================
# Unit tests for dn_add_ros2_pythonpath.bash
# Following N2ST guidelines: test actual function behavior, not mock implementations
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
# =================================================================================================

BATS_HELPER_PATH=/usr/lib/bats
if [[ -d ${BATS_HELPER_PATH} ]]; then
  load "${BATS_HELPER_PATH}/bats-support/load"
  load "${BATS_HELPER_PATH}/bats-assert/load"
  load "${BATS_HELPER_PATH}/bats-file/load"
  load "${SRC_CODE_PATH}/${N2ST_BATS_TESTING_TOOLS_RELATIVE_PATH}/bats_helper_functions"
else
  echo -e "\n[\033[1;31mERROR\033[0m] $0 path to bats-core helper library unreachable at \"${BATS_HELPER_PATH}\"!" 1>&2
  echo '(press any key to exit)'
  read -r -n 1
  exit 1
fi

# ====Setup========================================================================================

TESTED_FILE="dn_add_ros2_pythonpath.bash"
TESTED_FILE_PATH="/code/dockerized-norlab/dockerized-norlab-images/container-tools"

# executed once before starting the first test (valide for all test in that file)
setup_file() {
  BATS_DOCKER_WORKDIR=$(pwd) && export BATS_DOCKER_WORKDIR

  source "$BATS_DOCKER_WORKDIR/import_dockerized_norlab_tools.bash"
}


# executed before each test
setup() {
  cd "$TESTED_FILE_PATH" || exit 1
  
  # Create temporary directory for testing actual file operations
  TEST_SITE_PACKAGES_DIR=$(temp_make)
  TEST_PYTHON_BIN_DISTRO="$(temp_make)"
  export TEST_SITE_PACKAGES_DIR
  export TEST_PYTHON_BIN_DISTRO

  # Set up test environment variables
  export ROS_DISTRO="humble"
  export DN_DEV_WORKSPACE="/ros2_ws"

  PYTHON3_VERSION=$(n2st::which_python3_version)
  export PYTHON3_VERSION

  # Mock ros2 distro python
  mkdir -p "/opt/ros/${ROS_DISTRO}/lib/python${PYTHON3_VERSION}/site-packages"
  mkdir -p "/opt/ros/${ROS_DISTRO}/local/lib/python${PYTHON3_VERSION}/dist-packages"

  
  # Mock dn::source_ros2 to be available by default (can be overridden in individual tests)
  function dn::source_ros2() {
    return 0
  }
  export -f dn::source_ros2
}

# ====Teardown=====================================================================================

# executed after each test
teardown() {
  bats_print_run_env_variable_on_error
  
  # Clean up temporary directory
  if [[ -n "$TEST_SITE_PACKAGES_DIR" ]] && [[ -d "$TEST_SITE_PACKAGES_DIR" ]]; then
    temp_del "${TEST_SITE_PACKAGES_DIR}"
    temp_del "${TEST_PYTHON_BIN_DISTRO}"
  fi

  rm -rf "/opt/ros/${ROS_DISTRO}/lib/python${PYTHON3_VERSION}/site-packages"
  rm -rf "/opt/ros/${ROS_DISTRO}/local/lib/python${PYTHON3_VERSION}/dist-packages"

  # Unset mock functions
  unset -f dn::source_ros2 2>/dev/null || true
  unset -f python3 2>/dev/null || true
  unset -f bash 2>/dev/null || true
  unset PYTHONPATH
}

## executed once after finishing the last test (valide for all test in that file)
#teardown_file() {
#}

# ====Test cases==================================================================================

@test "assess execute with \"bash $TESTED_FILE\" › expect fail" {
#  skip "TMP DEV" # ToDo: on task end >> delete this line ←
  run bash "$TESTED_FILE"
  assert_failure
}

@test "${TESTED_FILE} › source script › expect pass" {
#  skip "TMP DEV" # ToDo: on task end >> delete this line ←
  run source "$TESTED_FILE"
  assert_success
}

@test "dn::add_ros2_python_paths › function exists after sourcing › expect pass" {
#  skip "TMP DEV" # ToDo: on task end >> delete this line ←
  source "$TESTED_FILE"

  # Test that function is available
  run type dn::add_ros2_python_paths
  assert_success
}

@test "dn::add_ros2_python_paths › method=env › PYTHONPATH modification › expect pass" {
#  skip "TMP DEV" # ToDo: on task end >> delete this line ←

  # Test env method - should modify PYTHONPATH if ROS directories exist
  run source "$TESTED_FILE" "env"
  assert_success

  # Save current PYTHONPATH
  local original_pythonpath="$PYTHONPATH"
#  echo "PYTHONPATH: $PYTHONPATH"  >&3 # (Priority) ToDo: on task end >> mute this line ↓

  assert_empty "$PYTHONPATH"
  source "$TESTED_FILE" "env"

#  echo "PYTHONPATH: $PYTHONPATH"  >&3 # (Priority) ToDo: on task end >> mute this line ↓
  assert_not_empty "$PYTHONPATH"
  assert_not_equal "$original_pythonpath" "$PYTHONPATH"

  # The function should execute without error even if ROS paths don't exist
  # We're testing actual behavior, not mocking
}

@test "dn::add_ros2_python_paths › method=pth › .pth file creation › expect pass" {
#  skip "TMP DEV" # ToDo: on task end >> delete this line ←

  # Mock python distro
  mkdir -p "${TEST_PYTHON_BIN_DISTRO}/python9.99/dist-packages"

  # Mock python3 for controlled testing
  function python3() {
    case "$*" in
      *"site.getsitepackages"*)
        echo "$TEST_SITE_PACKAGES_DIR"
        ;;
      *)
        command python3 "$@"
        ;;
    esac
  }
  export -f python3


  for distro in "humble" "galactic" "foxy"; do
    export ROS_DISTRO="$distro"

    # Mock dn::source_ros2 to be available
    function dn::source_ros2() {
      # Return mock post-ROS2 Python path with additional paths
      export PYTHONPATH="${TEST_PYTHON_BIN_DISTRO}/python9.99/dist-packages:/ros2_ws/install/vesc_msgs/local/lib/python3${PYTHON3_VERSION}/dist-packages:/ros2_ws/build/f1tenth_gym_ros:/ros2_ws/install/f1tenth_gym_ros/lib/python3${PYTHON3_VERSION}/site-packages:/ros2_ws/build/examples_rclpy_minimal_subscriber:/ros2_ws/install/examples_rclpy_minimal_subscriber/lib/python${PYTHON3_VERSION}/site-packages:/ros2_ws/build/examples_rclpy_minimal_publisher:/ros2_ws/install/examples_rclpy_minimal_publisher/lib/python${PYTHON3_VERSION}/site-packages:/opt/ros/${ROS_DISTRO}/lib/python${PYTHON3_VERSION}/site-packages:/opt/ros/${ROS_DISTRO}/local/lib/python3${PYTHON3_VERSION}/dist-packages:/ros2_ws/src/RedLeader-research-codebase/src:/ros2_ws/src/RedLeader-research-codebase/tests:/ros2_ws/src/RedLeader-research-codebase/external_data:/home/redleader:/usr/lib/python310.zip:/usr/lib/python${PYTHON3_VERSION}:/usr/lib/python3${PYTHON3_VERSION}/lib-dynload:/usr/local/lib/python${PYTHON3_VERSION}/dist-packages:/opt/f1tenth_gym/gym:/opt/mbrl-lib:/usr/lib/python3/dist-packages:/usr/lib/python${PYTHON3_VERSION}/dist-packages"
      return 0
    }
    export -f dn::source_ros2

    # Mock ros2 distro python
    mkdir -p "/opt/ros/${ROS_DISTRO}/lib/python${PYTHON3_VERSION}/site-packages"
    mkdir -p "/opt/ros/${ROS_DISTRO}/local/lib/python${PYTHON3_VERSION}/dist-packages"


    # Test pth method - should create .pth file
    run source "$TESTED_FILE" "pth"
    assert_success

    # Verify .pth file was created (testing actual file system effects)
    assert_file_exist "$TEST_SITE_PACKAGES_DIR/dna_ros_${ROS_DISTRO}.pth"

    # Verify file content format (one path per line, no quotes or brackets)
    run cat "$TEST_SITE_PACKAGES_DIR/dna_ros_${ROS_DISTRO}.pth"
    assert_success

    # Content should not contain brackets, quotes, or commas (proper .pth format)
    refute_output --partial "["
    refute_output --partial "]"
    refute_output --partial "'"
    refute_output --partial '"'
    refute_output --partial ","

    # Should contain actual ROS paths
    assert_output --partial "${TEST_PYTHON_BIN_DISTRO}/python9.99/dist-packages"
    assert_output --partial "/opt/ros/${ROS_DISTRO}/lib/python${PYTHON3_VERSION}/site-packages"
    assert_output --partial "/opt/ros/${ROS_DISTRO}/local/lib/python${PYTHON3_VERSION}/dist-packages"

    unset PYTHONPATH
  done

}

@test "dn::add_ros2_python_paths › method=pth › missing DN lib › expect fail" {
#  skip "TMP DEV" # ToDo: on task end >> delete this line ←

  # Mock only python3 site.getsitepackages 
  function python3() {
    if [[ "$*" == *"site.getsitepackages"* ]]; then
      echo "$TEST_SITE_PACKAGES_DIR"
    else
      command python3 "$@"
    fi
  }
  export -f python3
  
  # Ensure dn::source_ros2 is NOT available (test actual dependency check)
  # Unset the mock function from setup before sourcing
  unset -f dn::source_ros2 2>/dev/null || true
  
  # Test the actual script behavior when DN lib is missing
  # The script automatically executes dn::add_ros2_python_paths "$@" at the end
  # When sourced without arguments, it defaults to "pth" method and should fail
  run bash -c "source '$TESTED_FILE'"
  assert_failure
  assert_output --partial "The DN lib is not loaded!"
}

@test "dn::add_ros2_python_paths › method=syscustom › sitecustomize.py creation › expect pass" {
#  skip "TMP DEV" # ToDo: on task end >> delete this line ←

  # Mock only python3 site.getsitepackages
  function python3() {
    if [[ "$1" == "-m" ]] && [[ "$2" == "site" ]] && [[ "$3" == "--user-site" ]]; then
      echo "$TEST_SITE_PACKAGES_DIR"
    elif [[ "$*" == *"site.getsitepackages"* ]]; then
      echo "$TEST_SITE_PACKAGES_DIR"
    else
      command python3 "$@"
    fi
  }
  export -f python3

  # Test syscustom method - should create sitecustomize.py
  run source "$TESTED_FILE" "syscustom"
  assert_success
  
  # Verify sitecustomize.py was created (testing actual file system effects)
  assert_file_exist "$TEST_SITE_PACKAGES_DIR/sitecustomize.py"
  
  # Verify file content contains expected Python code
  run cat "$TEST_SITE_PACKAGES_DIR/sitecustomize.py"
  assert_success
  assert_output --partial "DNA ROS path auto-detection"
  assert_output --partial "ros_distro = os.environ.get('ROS_DISTRO')"
  assert_output --partial "sys.path.insert"
}

@test "dn::add_ros2_python_paths › no ROS_DISTRO › expect pass but no action" {
#  skip "TMP DEV" # ToDo: on task end >> delete this line ←

  unset ROS_DISTRO
  source "$TESTED_FILE"

  # Should succeed but take no action when ROS_DISTRO is not set
  run dn::add_ros2_python_paths "env"
  assert_success
  assert_output --partial "ROS_DISTRO is not set! Skipping dn::add_ros2_python_paths."

  # No .pth file should be created when ROS_DISTRO is missing
  run dn::add_ros2_python_paths "pth"
  assert_success
  assert_file_not_exist "$TEST_SITE_PACKAGES_DIR/dna_ros_.pth"
  assert_output --partial "ROS_DISTRO is not set! Skipping dn::add_ros2_python_paths."
}
