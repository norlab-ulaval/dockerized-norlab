
function mock_docker_command_exit_ok() {
    function docker() {
      local tmp=$@
      export DOCKER_EXIT_CODE=0
      return 0
    }
}

function mock_docker_command_exit_error() {
    function docker() {
      local tmp=$@
      echo "Error" 1>&2
      export DOCKER_EXIT_CODE=1
      return 1
    }
}

function mock_docker_command_config_services() {
    function docker() {
      local tmp=$@
      echo "mock-service-one mock-service-two"
      export DOCKER_EXIT_CODE=0
      return 0
    }
}

function mock_docker_command_config_services_base_image_squash() {
    function docker() {
      local tmp=$@
      echo "base-image-squash-main base-image-squash-tester base-image-squash"
      export DOCKER_EXIT_CODE=0
      return 0
    }
}

function mock_docker_command_config_services_base_image_squash_exit_error() {
    function docker() {
      local tmp=$@
      echo "base-image-squash-main base-image-squash-tester base-image-squash"
      echo "Error" 1>&2
      export DOCKER_EXIT_CODE=1
      return 1
    }
}

function mock_docker_command_fail_once() {
    _fail_count=0
    function docker() {
      echo "MOCK DOCKER CALLED WITH: $*" >&3
      if [[ "$*" == *"config --services"* ]]; then
        echo "mock-service-one mock-service-two"
        return 0
      fi
      # Success for pre-callback commands
      if [[ $1 == "pull" ]] || [[ $1 == "run" ]] || [[ $1 == "inspect" ]]; then
        if [[ $1 == "inspect" ]]; then
            echo "PATH=/usr/local/bin"
        fi
        return 0
      fi

      if [[ ${_fail_count} -lt 1 ]]; then
        (( _fail_count++ ))
        echo "504 Gateway Time-out" 1>&2
        return 1
      fi
      echo "Success"
      return 0
    }
}
