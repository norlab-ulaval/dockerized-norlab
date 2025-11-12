# GDBServer Remote Debugging Implementation Plan

## Overview

This plan outlines the implementation of gdbserver remote debugging support for Dockerized-NorLab containers, enabling remote debugging of C/C++ applications from IDEs like CLion running on the host machine.

## Current State Analysis

### ✅ Already Implemented Infrastructure:
1. Package installation: `gdbserver` package is installed in `dockerized-norlab-images/core-images/dependencies/Dockerfile.core`
2. **Port Configuration**: `DN_GDB_SERVER_PORT` environment variable set to 7777 in project-develop Dockerfile
3. **Port Exposure**: Container exposes port 7777 in Dockerfile
4. **Port Mapping**: Docker-compose maps host port 7777 to container port 7777
5. **Security Settings**: Container has required security options for debugging:
   - `seccomp=unconfined`
   - `apparmor=unconfined` 
   - `SYS_PTRACE` capability
   - Host PID namespace access
6. **Basic Documentation**: Port 7777 mentioned in README.md manual usage example

### ❌ Missing Components:
1. **GDBServer Configuration**: No gdbserver-specific setup in `dn_install_debugging_tools.bash`
2. **Utility Scripts**: No scripts for starting/managing gdbserver processes
3. **Process Management**: No handling of gdbserver lifecycle
4. **CLion Integration Documentation**: No specific instructions for CLion remote debugging setup
5. **Usage Documentation**: No comprehensive guide for gdbserver usage workflows

## Implementation Plan

### Phase 1: Core Script Enhancement
**Target File**: `dockerized-norlab-images/container-tools/dn_install_debugging_tools.bash`

#### 1.1 Environment Variable Validation
**Implementation Example**:
```bash
# Add to existing pre-condition check block (around line 54)
# BEFORE:
{
  test -n "${DN_SSH_SERVER_PORT:?'Env variable needs to be set and non-empty.'}" && \
  test -n "${DN_SSH_SERVER_USER:?'Env variable needs to be set and non-empty.'}" && \
  test -n "${DN_SSH_SERVER_USER_PASSWORD:?'Env variable needs to be set and non-empty.'}" && \
  test -n "${DN_PROJECT_GID:?'Env variable needs to be set and non-empty.'}" && \
  test -n "${DN_PROJECT_USER:?'Env variable needs to be set and non-empty.'}" && \
  test -n "${DEBIAN_FRONTEND:?'Env variable need to be set and non-empty.'}" && \
  [[ "${DEBIAN_FRONTEND}" == "noninteractive" ]];
} || n2st::print_msg_error_and_exit "Failed dn::setup_debugging_tools pre-condition check!"

# AFTER:
{
  test -n "${DN_SSH_SERVER_PORT:?'Env variable needs to be set and non-empty.'}" && \
  test -n "${DN_SSH_SERVER_USER:?'Env variable needs to be set and non-empty.'}" && \
  test -n "${DN_SSH_SERVER_USER_PASSWORD:?'Env variable needs to be set and non-empty.'}" && \
  test -n "${DN_PROJECT_GID:?'Env variable needs to be set and non-empty.'}" && \
  test -n "${DN_PROJECT_USER:?'Env variable needs to be set and non-empty.'}" && \
  test -n "${DN_GDB_SERVER_PORT:?'Env variable needs to be set and non-empty.'}" && \
  test -n "${DEBIAN_FRONTEND:?'Env variable need to be set and non-empty.'}" && \
  [[ "${DEBIAN_FRONTEND}" == "noninteractive" ]];
} || n2st::print_msg_error_and_exit "Failed dn::setup_debugging_tools pre-condition check!"
```

#### 1.2 GDBServer Configuration Section
**Implementation Example** (Add after SSH server setup, around line 118):
```bash
# ===Service: gdbserver============================================================================
n2st::print_msg "Setting up gdbserver remote debugging support..."

# ....Create gdbserver utility scripts............................................................
n2st::print_msg "Creating gdbserver utility scripts..."

# [Utility scripts content will be shown in Phase 2]

# Make scripts executable
chmod +x /usr/local/bin/dn-gdbserver-start
chmod +x /usr/local/bin/dn-gdbserver-control

n2st::print_msg "Gdbserver utility scripts installed:"
n2st::print_msg "  - dn-gdbserver-start: Start gdbserver for debugging"
n2st::print_msg "  - dn-gdbserver-control: Control gdbserver processes"
n2st::print_msg "Port configured: ${DN_GDB_SERVER_PORT}"
```

**Documentation Header Update**:
```bash
# BEFORE:
#   3. Installs/update debugging the necessary packages,

# AFTER:
#   3. Sets up gdbserver for remote debugging with utility scripts.
#   4. Installs/update debugging the necessary packages,
```

**Global Variables Documentation Update**:
```bash
# Add to the "Global Variables" section in the script header:
# - DN_GDB_SERVER_PORT (Read): The port for the gdbserver remote debugging.
```

### Phase 2: Utility Scripts Creation
**Location**: `/usr/local/bin/` (installed during debugging tools setup)

#### 2.1 GDBServer Startup Script (`dn-gdbserver-start`)
**Complete Implementation**:

File: `/usr/local/bin/dn-gdbserver-start`
```bash
#!/bin/bash
# =================================================================================================
# Start gdbserver for remote debugging
# Usage: 
#   dn-gdbserver-start attach <pid>           # Attach to running process
#   dn-gdbserver-start launch <executable>    # Launch executable for debugging
#   dn-gdbserver-start multi                  # Start multi-process mode
# =================================================================================================

set -e

# Source environment
source /import_dockerized_norlab_container_tools.bash 2>/dev/null || true

DN_GDB_SERVER_PORT=${DN_GDB_SERVER_PORT:-7777}

case "${1:-}" in
    attach)
        if [[ -z "${2:-}" ]]; then
            echo "Error: Process ID required for attach mode"
            echo "Usage: dn-gdbserver-start attach <pid>"
            exit 1
        fi
        echo "Starting gdbserver on port ${DN_GDB_SERVER_PORT}, attaching to PID $2"
        exec gdbserver :${DN_GDB_SERVER_PORT} --attach $2
        ;;
    launch)
        if [[ -z "${2:-}" ]]; then
            echo "Error: Executable required for launch mode"
            echo "Usage: dn-gdbserver-start launch <executable> [args...]"
            exit 1
        fi
        shift # remove 'launch' argument
        echo "Starting gdbserver on port ${DN_GDB_SERVER_PORT}, launching: $*"
        exec gdbserver :${DN_GDB_SERVER_PORT} "$@"
        ;;
    multi)
        echo "Starting gdbserver in multi-process mode on port ${DN_GDB_SERVER_PORT}"
        exec gdbserver --multi :${DN_GDB_SERVER_PORT}
        ;;
    *)
        echo "GDBServer Remote Debugging Utility"
        echo "Usage:"
        echo "  dn-gdbserver-start attach <pid>           # Attach to running process"
        echo "  dn-gdbserver-start launch <executable>    # Launch executable for debugging"  
        echo "  dn-gdbserver-start multi                  # Start multi-process mode"
        echo ""
        echo "Port: ${DN_GDB_SERVER_PORT}"
        echo ""
        echo "CLion Integration:"
        echo "  1. Create Remote GDB Server debug configuration"
        echo "  2. Set 'localhost' as target host"
        echo "  3. Set '${DN_GDB_SERVER_PORT}' as port"
        echo "  4. Ensure debug symbols are built (-g flag)"
        exit 1
        ;;
esac
```

**Usage Examples**:
```bash
# Attach to a running ROS 2 node
dn-gdbserver-start attach 1234

# Launch a C++ executable for debugging
dn-gdbserver-start launch ./my_program --arg1 value1

# Start multi-process mode
dn-gdbserver-start multi
```

#### 2.2 GDBServer Control Script (`dn-gdbserver-control`)
**Complete Implementation**:

File: `/usr/local/bin/dn-gdbserver-control`
```bash
#!/bin/bash
# =================================================================================================
# Control gdbserver processes
# Usage: 
#   dn-gdbserver-control status    # Show running gdbserver processes
#   dn-gdbserver-control kill      # Kill all gdbserver processes
# =================================================================================================

set -e

DN_GDB_SERVER_PORT=${DN_GDB_SERVER_PORT:-7777}

case "${1:-}" in
    status)
        echo "Checking gdbserver processes on port ${DN_GDB_SERVER_PORT}..."
        if pgrep -f "gdbserver.*:${DN_GDB_SERVER_PORT}" > /dev/null; then
            echo "Active gdbserver processes:"
            pgrep -af "gdbserver.*:${DN_GDB_SERVER_PORT}"
            echo ""
            echo "Network connections:"
            netstat -tlnp 2>/dev/null | grep ":${DN_GDB_SERVER_PORT} " || echo "No listening connections on port ${DN_GDB_SERVER_PORT}"
        else
            echo "No gdbserver processes found on port ${DN_GDB_SERVER_PORT}"
        fi
        ;;
    kill)
        echo "Killing gdbserver processes on port ${DN_GDB_SERVER_PORT}..."
        if pgrep -f "gdbserver.*:${DN_GDB_SERVER_PORT}" > /dev/null; then
            pkill -f "gdbserver.*:${DN_GDB_SERVER_PORT}"
            echo "Gdbserver processes killed"
        else
            echo "No gdbserver processes found to kill"
        fi
        ;;
    *)
        echo "GDBServer Control Utility"
        echo "Usage:"
        echo "  dn-gdbserver-control status    # Show running gdbserver processes"
        echo "  dn-gdbserver-control kill      # Kill all gdbserver processes"
        echo ""
        echo "Port: ${DN_GDB_SERVER_PORT}"
        exit 1
        ;;
esac
```

**Usage Examples with Expected Output**:
```bash
# Check status when no gdbserver is running
$ dn-gdbserver-control status
Checking gdbserver processes on port 7777...
No gdbserver processes found on port 7777

# Check status when gdbserver is running
$ dn-gdbserver-control status
Checking gdbserver processes on port 7777...
Active gdbserver processes:
12345 pts/1    S+     0:00 gdbserver :7777 --attach 98765

Network connections:
tcp        0      0 0.0.0.0:7777            0.0.0.0:*               LISTEN      12345/gdbserver

# Kill running gdbserver processes
$ dn-gdbserver-control kill
Killing gdbserver processes on port 7777...
Gdbserver processes killed
```

### Phase 3: CLion Integration Support

#### 3.1 Remote Toolchain Configuration
**Step-by-Step Implementation Guide**:

**Step 1: Configure SSH Connection**
1. Open CLion → Settings (Ctrl+Alt+S on Linux/Windows, Cmd+, on macOS)
2. Navigate to: Build, Execution, Deployment → Deployment
3. Click "+" to add new deployment configuration
4. Select "SFTP" and name it "DN-Container"
5. Configure connection:
   ```
   Type: SFTP
   Host: localhost
   Port: 2222
   Username: non-interactive-ros2
   Password: *****
   Root path: /
   ```

**Step 2: Configure Remote Toolchain**
1. Navigate to: Build, Execution, Deployment → Toolchains
2. Click "+" and select "Remote Host"
3. Configure settings:
   ```
   Name: DN-Remote-Toolchain
   Deployment configuration: DN-Container (from Step 1)
   CMake: /usr/bin/cmake
   Make: /usr/bin/make
   C Compiler: /usr/bin/gcc
   C++ Compiler: /usr/bin/g++
   Debugger: /usr/bin/gdb
   ```
4. Click "Test Connection" to verify all tools are detected

**Step 3: Set Default Toolchain**
1. In same Toolchains settings, move "DN-Remote-Toolchain" to top of list
2. This makes it the default for new projects

**CMake Configuration Example**:
```cmake
# Ensure debug symbols are included
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")
set(CMAKE_C_FLAGS_DEBUG "-g -O0")

# Optional: Enable additional debugging info
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ggdb")
```

#### 3.2 Debug Configuration Setup
**Step-by-Step Implementation Guide**:

**Step 1: Create GDB Remote Debug Configuration**
1. Navigate to: Run → Edit Configurations
2. Click "+" → GDB Remote Debug
3. Configure settings:
   ```
   Name: DN-Remote-Debug
   Target remote args: localhost:7777
   Symbol file: path/to/your/executable (with debug symbols)
   Sysroot: /
   Path mappings: (configure if source paths differ)
   ```

**Step 2: Advanced Configuration Options**
```
GDB Startup Commands (optional):
set solib-search-path /usr/lib/x86_64-linux-gnu:/lib/x86_64-linux-gnu
set substitute-path /old/path /new/path

Environment Variables:
ROS_DOMAIN_ID=1
RMW_IMPLEMENTATION=rmw_cyclonedx_cpp
```

**Step 3: Debugging Workflow Examples**

**Example 1: Debug a ROS 2 Node**
```bash
# In container terminal 1: Start your ROS 2 node
ros2 run my_package my_node

# In container terminal 2: Find PID and attach gdbserver
pgrep -f my_node
# Output: 12345
dn-gdbserver-start attach 12345

# In CLion: Start "DN-Remote-Debug" configuration
# Set breakpoints in your code and debug normally
```

**Example 2: Debug from Launch**
```bash
# In container: Launch with gdbserver
dn-gdbserver-start launch /path/to/my_executable --arg1 value1

# In CLion: Start "DN-Remote-Debug" configuration
# Program will start paused, allowing you to set breakpoints
```

**Troubleshooting Configuration**:

**Connection Issues**:
```bash
# Verify SSH connection from host
ssh -p 2222 non-interactive-ros2@localhost

# Test gdbserver connectivity
telnet localhost 7777
```

**Symbol File Issues**:
- Ensure executable was built with `-g` flag
- Verify symbol file path in debug configuration matches container path
- Check that executable is accessible from container

**Path Mapping Example**:
```
Local Path: /home/user/project/src
Remote Path: /project/src
```

### Phase 4: Documentation Creation

#### 4.1 Comprehensive User Guide
**Target File**: `GDBSERVER_USAGE_GUIDE.md`
**Complete Structure Example**:
````markdown
# GDBServer Remote Debugging Guide for Dockerized-NorLab

## Overview
Dockerized-NorLab now includes comprehensive gdbserver support for remote debugging of C/C++ applications running inside containers. This enables full debugging capabilities from IDEs like CLion running on your host machine.

## Prerequisites
- Dockerized-NorLab container with debugging tools enabled (project-develop image)
- CLion IDE (or other GDB-compatible debugger) on host machine
- Application compiled with debug symbols (`-g` flag)

## Quick Start

### Available Utility Scripts
#### `dn-gdbserver-start`
Start gdbserver for debugging:
```bash
# Attach to a running process
dn-gdbserver-start attach <pid>

# Launch an executable for debugging
dn-gdbserver-start launch <executable> [args...]

# Start multi-process mode (supports multiple debugging sessions)
dn-gdbserver-start multi
```

#### `dn-gdbserver-control`
Manage gdbserver processes:
```bash
# Check status of running gdbserver processes
dn-gdbserver-control status

# Kill all gdbserver processes
dn-gdbserver-control kill
```

## CLion Integration
[Include the detailed steps from Phase 3.1 and 3.2]

## Usage Examples
[Include the debugging workflow examples from Phase 3.2]

## Troubleshooting
[Include common issues and solutions from Phase 3.2]

## Security Considerations
- GDBServer only binds to container localhost by default
- SSH access required for CLion remote toolchain
- Debug builds contain symbol information
- Only enable debugging in development environments
````

#### 4.2 Integration with DNA Documentation
**Main README.md Updates**:

**Feature List Addition (around line 66)**:
```markdown
# BEFORE:
- Multi packages (non-exhaustive list): 
  - Deep-learning: `pytorch`, `torch2trt`, `tensordict`
  - Deep-rl: `torchrl`
  - Numerical computing: `numpy`, `numba`, `pycuda`, `tensorrt`
  - MLOps: `ray`, `omegaconf`, `hydra-core`

# AFTER:
- Multi packages (non-exhaustive list): 
  - Deep-learning: `pytorch`, `torch2trt`, `tensordict`
  - Deep-rl: `torchrl`
  - Numerical computing: `numpy`, `numba`, `pycuda`, `tensorrt`
  - MLOps: `ray`, `omegaconf`, `hydra-core`
  - Remote debugging: `gdbserver`, `openssh-server`, CLion integration
```

**Manual Usage Update (around line 106)**:
```markdown
# BEFORE:
docker run -it --rm \
    --env ROS_DOMAIN_ID=1 --env DISPLAY="${DISPLAY}" --env QT_X11_NO_MITSHM=1 \
    --net host -p 2222:2222 -p 6006:6006 -p 7777:7777 \
    --privileged \
    -v /etc/localtime:/etc/localtime:ro -v /tmp/.X11-unix:/tmp/.X11-unix:rw -v /tmp/.docker.xauth:/tmp/.docker.xauth:rw \
    norlabulaval/dockerized-norlab-dependencies-full:DN-bleeding-foxy-base-l4t-pytorch-r35.4.1

# AFTER:
docker run -it --rm \
    --env ROS_DOMAIN_ID=1 --env DISPLAY="${DISPLAY}" --env QT_X11_NO_MITSHM=1 \
    --net host -p 2222:2222 -p 6006:6006 -p 7777:7777 \
    --privileged \
    -v /etc/localtime:/etc/localtime:ro -v /tmp/.X11-unix:/tmp/.X11-unix:rw -v /tmp/.docker.xauth:/tmp/.docker.xauth:rw \
    norlabulaval/dockerized-norlab-dependencies-full:DN-bleeding-foxy-base-l4t-pytorch-r35.4.1

# Port Mapping Explanation:
# - 2222:2222  # SSH server for remote development
# - 6006:6006  # TensorBoard
# - 7777:7777  # GDBServer for remote debugging
```

**New Debugging Section Addition**:
````markdown
# Add after "Basic usage" section (around line 111)

---

# Remote Development & Debugging

## SSH Access
The containers include SSH server support for remote development:
- **Port**: 2222 (mapped from container's 22)
- **User**: `non-interactive-ros2` (with ROS2 environment pre-loaded)
- **Password**: `*****` (configurable via `DN_SSH_SERVER_USER_PASSWORD`)

```bash
# Connect to running container
ssh -p 2222 non-interactive-ros2@localhost
```

## Remote Debugging with GDBServer
Full C/C++ debugging support for CLion and other GDB-compatible IDEs:
- **Port**: 7777 (configurable via `DN_GDB_SERVER_PORT`)
- **Utility Scripts**: `dn-gdbserver-start`, `dn-gdbserver-control`
- **Integration**: Complete CLion remote debugging setup

For detailed setup instructions, see [GDBServer Usage Guide](GDBSERVER_USAGE_GUIDE.md).

### Quick Debugging Example
```bash
# In container: Attach to running process
dn-gdbserver-start attach <pid>

# In CLion: Connect with Remote GDB Debug configuration
# Target: localhost:7777
```
````

**Container Information Script Update**:
Update `dn_info.bash` to include gdbserver information in the debugging section (around line 121).

#### 4.3 Testing and Validation Examples
**Validation Script Structure**:
```bash
#!/bin/bash
# test_gdbserver_implementation.bash

echo "Testing GDBServer Implementation..."

# Test 1: Verify utility scripts exist and are executable
test -x /usr/local/bin/dn-gdbserver-start || { echo "FAIL: dn-gdbserver-start not found"; exit 1; }
test -x /usr/local/bin/dn-gdbserver-control || { echo "FAIL: dn-gdbserver-control not found"; exit 1; }

# Test 2: Verify scripts show help when run without arguments
/usr/local/bin/dn-gdbserver-start | grep -q "GDBServer Remote Debugging Utility" || { echo "FAIL: dn-gdbserver-start help"; exit 1; }
/usr/local/bin/dn-gdbserver-control | grep -q "GDBServer Control Utility" || { echo "FAIL: dn-gdbserver-control help"; exit 1; }

# Test 3: Verify environment variable is configured
test -n "${DN_GDB_SERVER_PORT}" || { echo "FAIL: DN_GDB_SERVER_PORT not set"; exit 1; }

# Test 4: Verify gdbserver is installed
which gdbserver >/dev/null || { echo "FAIL: gdbserver not installed"; exit 1; }

# Test 5: Verify port is not in use (clean state)
! netstat -tlnp 2>/dev/null | grep -q ":${DN_GDB_SERVER_PORT} " || { echo "FAIL: Port ${DN_GDB_SERVER_PORT} already in use"; exit 1; }

echo "SUCCESS: All GDBServer implementation tests passed"
```

## Technical Implementation Details

### GDBServer Usage Patterns:
1. **Attach Mode**: `gdbserver :7777 --attach <pid>` - attach to running process
2. **Launch Mode**: `gdbserver :7777 <executable> [args]` - launch and debug program
3. **Multi-process Mode**: `gdbserver --multi :7777` - support multiple debugging sessions

### CLion Integration Requirements:
1. **Remote Toolchain**: Configure remote GCC/GDB toolchain in CLion
2. **Deployment**: Set up automatic deployment of binaries to container
3. **Debug Configuration**: Create remote GDB debug configuration
4. **Symbol Paths**: Configure symbol and source file paths

### Security Considerations:
- GDBServer should only bind to localhost/container network
- Leverage existing container security model
- Document security implications for development environments

## Implementation Sequence

### Step 1: Script Enhancement ⭐ High Priority
- Modify `dn_install_debugging_tools.bash` to include gdbserver setup
- Add environment variable validation
- Create utility scripts during installation

### Step 2: Utility Scripts ⭐ High Priority
- Implement `dn-gdbserver-start` with all modes
- Implement `dn-gdbserver-control` for process management
- Ensure scripts are executable and properly installed

### Step 3: Documentation 🔶 Medium Priority
- Create comprehensive usage guide
- Document CLion integration steps
- Add troubleshooting section

### Step 4: DNA Integration 🔶 Medium Priority
- Update main documentation
- Add to feature lists
- Include in development workflows

### Step 5: Testing and Validation 🔷 Low Priority
- Test with sample C++ applications
- Validate CLion integration
- Test different debugging scenarios

## Success Criteria

### ✅ Functional Requirements:
- [ ] GDBServer utility scripts are installed and functional
- [ ] All three debugging modes work correctly (attach, launch, multi)
- [ ] CLion can successfully connect and debug containerized applications
- [ ] Process management works reliably
- [ ] Environment variables are properly validated

### ✅ Documentation Requirements:
- [ ] Comprehensive user guide is available
- [ ] CLion integration is fully documented
- [ ] Troubleshooting guide covers common issues
- [ ] Documentation is integrated with main DNA docs

### ✅ Quality Requirements:
- [ ] Scripts follow existing Dockerized-NorLab patterns
- [ ] Error handling is robust and user-friendly
- [ ] Security model is maintained
- [ ] Installation integrates cleanly with existing debugging tools setup

## Risk Assessment

### 🟨 Medium Risks:
- **CLion Version Compatibility**: Different CLion versions may have different remote debugging setups
- **Network Configuration**: Container networking issues could affect debugging connectivity
- **Process Permissions**: Debugging permissions might be affected by container security policies

### 🟩 Low Risks:
- **GDBServer Availability**: Package already confirmed installed
- **Port Conflicts**: Port 7777 already reserved and configured
- **Security Model**: Existing debugging security already established

## Dependencies

### Internal Dependencies:
- Existing debugging tools installation script structure
- Container security configuration
- SSH server setup for CLion toolchain access

### External Dependencies:
- CLion IDE on host machine
- Proper C/C++ toolchain in container
- Applications built with debug symbols (`-g` flag)

## Estimated Effort

- **Script Enhancement**: 2-3 hours
- **Utility Scripts Development**: 3-4 hours  
- **Documentation Creation**: 4-5 hours
- **Testing and Validation**: 2-3 hours
- **Total Estimated Effort**: 11-15 hours

## Next Steps

1. Begin with Phase 1 (Core Script Enhancement)
2. Implement and test utility scripts
3. Create comprehensive documentation
4. Integrate with main DNA documentation
5. Perform end-to-end testing with CLion

This plan provides a comprehensive roadmap for implementing gdbserver remote debugging support while leveraging existing infrastructure and maintaining Dockerized-NorLab's quality and security standards.
