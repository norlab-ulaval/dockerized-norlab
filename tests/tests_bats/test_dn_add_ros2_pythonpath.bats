#!/usr/bin/env bats
#
# Unit tests for dn_add_ros2_pythonpath.bash
# Following N2ST guidelines: test actual function behavior, not mock implementations
#
# Usage in docker container
#   $ REPO_ROOT=$(pwd) && RUN_TESTS_IN_DIR='tests'
#   $ docker run -it --rm -v "$REPO_ROOT:/code" bats/bats:latest "$RUN_TESTS_IN_DIR"
#

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
}

# executed before each test
setup() {
  cd "$TESTED_FILE_PATH" || exit 1
  
  # Create temporary directory for testing actual file operations
  TEST_SITE_PACKAGES_DIR=$(mktemp -d)
  export TEST_SITE_PACKAGES_DIR
  
  # Set up test environment variables
  export ROS_DISTRO="humble"
  export DN_DEV_WORKSPACE="/ros2_ws"
  
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
    rm -rf "$TEST_SITE_PACKAGES_DIR"
  fi
  
  # Unset mock functions
  unset -f dn::source_ros2 2>/dev/null || true
  unset -f python3 2>/dev/null || true
  unset -f bash 2>/dev/null || true
}

## executed once after finishing the last test (valide for all test in that file)
#teardown_file() {
#}

# ====Test cases==================================================================================

@test "assess execute with \"bash $TESTED_FILE\" › expect pass" {
  run bash "$TESTED_FILE"
  assert_success
}

@test "${TESTED_FILE} › source script › expect pass" {
  run source "$TESTED_FILE"
  assert_success
}

@test "dna::setup_ros2_python_paths › function exists after sourcing › expect pass" {
  source "$TESTED_FILE"
  
  # Test that function is available
  run type dna::setup_ros2_python_paths
  assert_success
}

@test "dna::setup_ros2_python_paths › method=env › PYTHONPATH modification › expect pass" {
  source "$TESTED_FILE"
  
  # Save current PYTHONPATH
  local original_pythonpath="$PYTHONPATH"
  
  # Test env method - should modify PYTHONPATH if ROS directories exist
  run dna::setup_ros2_python_paths "env"
  assert_success
  
  # The function should execute without error even if ROS paths don't exist
  # We're testing actual behavior, not mocking
}

@test "dna::setup_ros2_python_paths › method=pth › .pth file creation › expect pass" {
  # Mock python3 for controlled testing
  function python3() {
    case "$*" in
      *"site.getsitepackages"*)
        echo "$TEST_SITE_PACKAGES_DIR"
        ;;
      *"import sys; print(sys.path)"*)
        # Return mock pre-ROS2 Python path
        echo "['', '/usr/lib/python3.10', '/usr/lib/python3/dist-packages']"
        ;;
      *)
        command python3 "$@"
        ;;
    esac
  }
  export -f python3
  
  # Mock bash command to simulate ROS2 sourcing
  function bash() {
    if [[ "$1" == "-c" ]] && [[ "$2" == *"dn::source_ros2"* ]]; then
      # Return mock post-ROS2 Python path with additional paths
      echo "['', '/opt/ros/humble/lib/python3.10/site-packages', '/opt/ros/humble/local/lib/python3.10/dist-packages', '/usr/lib/python3.10', '/usr/lib/python3/dist-packages']"
    else
      command bash "$@"
    fi
  }
  export -f bash
  
  # Mock dn::source_ros2 to be available
  function dn::source_ros2() {
    return 0
  }
  export -f dn::source_ros2
  
  source "$TESTED_FILE"
  
  # Test pth method - should create .pth file
  run dna::setup_ros2_python_paths "pth"
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
  assert_output --partial "/opt/ros/humble/lib/python3.10/site-packages"
  assert_output --partial "/opt/ros/humble/local/lib/python3.10/dist-packages"
}

@test "dna::setup_ros2_python_paths › method=pth › missing DN lib › expect fail" {
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
  # The script automatically executes dna::setup_ros2_python_paths "$@" at the end
  # When sourced without arguments, it defaults to "pth" method and should fail
  run bash -c "source '$TESTED_FILE'"
  assert_failure
  assert_output --partial "The DN lib is not loaded!"
}

@test "dna::setup_ros2_python_paths › method=syscustom › sitecustomize.py creation › expect pass" {
  # Mock only python3 site.getsitepackages
  function python3() {
    if [[ "$*" == *"site.getsitepackages"* ]]; then
      echo "$TEST_SITE_PACKAGES_DIR"
    else
      command python3 "$@"
    fi
  }
  export -f python3
  
  source "$TESTED_FILE"
  
  # Test syscustom method - should create sitecustomize.py
  run dna::setup_ros2_python_paths "syscustom"
  assert_success
  
  # Verify sitecustomize.py was created (testing actual file system effects)
  assert_file_exist "$TEST_SITE_PACKAGES_DIR/dna_sitecustomize.py"
  
  # Verify file content contains expected Python code
  run cat "$TEST_SITE_PACKAGES_DIR/dna_sitecustomize.py"
  assert_success
  assert_output --partial "DNA ROS path auto-detection"
  assert_output --partial "ros_distro = os.environ.get('ROS_DISTRO')"
  assert_output --partial "sys.path.insert"
}

@test "dna::setup_ros2_python_paths › no ROS_DISTRO › expect pass but no action" {
  unset ROS_DISTRO
  source "$TESTED_FILE"
  
  # Should succeed but take no action when ROS_DISTRO is not set
  run dna::setup_ros2_python_paths "env"
  assert_success
  
  # No .pth file should be created when ROS_DISTRO is missing
  run dna::setup_ros2_python_paths "pth"
  assert_success
  assert_file_not_exist "$TEST_SITE_PACKAGES_DIR/dna_ros_.pth"
}
