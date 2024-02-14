
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
