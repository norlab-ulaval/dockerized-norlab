#!/bin/bash
#
# Integration test for dn_add_ros2_pythonpath.bash
# Following N2ST guidelines: test real integration scenarios without duplicating implementation
#
# Usage:
#   $ bash test_dn_add_ros2_pythonpath_integration.bash
#

set -e

# Setup test environment
TEST_DIR=$(dirname "$(readlink -f "$0")")
REPO_ROOT=$(cd "${TEST_DIR}/../.." && pwd)
CONTAINER_TOOLS_DIR="${REPO_ROOT}/dockerized-norlab-images/container-tools"
TESTED_SCRIPT="${CONTAINER_TOOLS_DIR}/dn_add_ros2_pythonpath.bash"

echo
echo "=== Integration Test for dn_add_ros2_pythonpath.bash ==="
echo "Test directory: ${TEST_DIR}"
echo "Repository root: ${REPO_ROOT}"
echo "Tested script: ${TESTED_SCRIPT}"
echo

# Verify script exists
if [[ ! -f "${TESTED_SCRIPT}" ]]; then
    echo "ERROR: Script not found at ${TESTED_SCRIPT}"
    exit 1
fi

echo "✓ Script file exists"

# Setup mock function for dn::source_ros2 to avoid dependency issues
function dn::source_ros2() {
    return 0
}
export -f dn::source_ros2

# Test 1: Basic script execution with different ROS distros
echo
echo "Test 1: Script execution with different ROS distributions"
for distro in "humble" "galactic" "foxy"; do
    export ROS_DISTRO="$distro"
    if timeout 10 bash "${TESTED_SCRIPT}"; then
        echo "✓ Script executes without errors for ROS $distro"
    else
        echo "✗ Script execution failed for ROS $distro"
        exit 1
    fi
done

# Test 2: Function availability and basic functionality
echo
echo "Test 2: Function availability and method execution"
export ROS_DISTRO="humble"
source "${TESTED_SCRIPT}"

if declare -f dna::setup_ros2_python_paths >/dev/null; then
    echo "✓ Function dna::setup_ros2_python_paths is available after sourcing"
else
    echo "✗ Function not available after sourcing"
    exit 1
fi

# Test all three methods
for method in "env" "pth" "syscustom"; do
    if dna::setup_ros2_python_paths "$method"; then
        echo "✓ Method '$method' executes successfully"
    else
        echo "✗ Method '$method' failed"
        exit 1
    fi
done

# Test 3: Real file system integration with temporary site-packages
echo
echo "Test 3: Real file system integration test"
TEST_SITE_PACKAGES="/tmp/integration_test_site_packages"
mkdir -p "${TEST_SITE_PACKAGES}"

# Mock python3 only for site.getsitepackages to use our test directory
function python3() {
    case "$*" in
        *"site.getsitepackages"*)
            echo "$TEST_SITE_PACKAGES"
            ;;
        *)
            command python3 "$@"
            ;;
    esac
}
export -f python3

# Mock dn::source_ros2 to be available
function dn::source_ros2() {
    return 0
}
export -f dn::source_ros2

# Test pth method with real file creation
if dna::setup_ros2_python_paths "pth"; then
    echo "✓ pth method executes successfully in integration test"
    
    # Check if .pth file was actually created
    pth_file="${TEST_SITE_PACKAGES}/dna_ros_${ROS_DISTRO}.pth"
    if [[ -f "$pth_file" ]]; then
        echo "✓ .pth file created successfully: $pth_file"

        echo && cat $pth_file && echo # ToDo: on task end >> mute this line ←

        # Verify file content format (should be paths, one per line, no brackets/quotes)
        if grep -q "^\/" "$pth_file" && ! grep -q "[\[\]',]" "$pth_file"; then
            echo "✓ .pth file format is correct (paths without brackets/quotes)"
        else
            echo "✗ .pth file format is incorrect"
            echo "File content:"
            cat "$pth_file"
            exit 1
        fi

    else
        echo "✓ No .pth file created (expected if no path difference detected)"
    fi
else
    echo "✗ pth method failed in integration test"
    exit 1
fi

# Test syscustom method with real file creation
if dna::setup_ros2_python_paths "syscustom"; then
    echo "✓ syscustom method executes successfully"
    
    # Check if sitecustomize.py was created
    sitecustom_file="${TEST_SITE_PACKAGES}/dna_sitecustomize.py"
    if [[ -f "$sitecustom_file" ]]; then
        echo "✓ sitecustomize.py file created successfully"

        echo && cat $sitecustom_file && echo # ToDo: on task end >> mute this line ←

        # Verify it contains valid Python code
        if python3 -m py_compile "$sitecustom_file" 2>/dev/null; then
            echo "✓ sitecustomize.py contains valid Python syntax"
        else
            echo "✗ sitecustomize.py contains invalid Python syntax"
            exit 1
        fi
    else
        echo "✗ sitecustomize.py not created"
        exit 1
    fi
else
    echo "✗ syscustom method failed"
    exit 1
fi


# Cleanup
rm -rf "${TEST_SITE_PACKAGES}"
unset -f python3 dn::source_ros2

echo
echo "=== All integration tests passed! ==="
