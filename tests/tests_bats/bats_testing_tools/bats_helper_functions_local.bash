
function mock_docker_command_exit_ok() {
    function docker() {
      local tmp=$@
      return 0
    }
}

function mock_docker_command_exit_error() {
    function docker() {
      local tmp=$@
      echo "Error" 1>&2
      return 1
    }
}

function mock_docker_command_config_services() {
    function docker() {
      local tmp=$@
      echo "mock-service-one mock-service-two"
      return 0
    }
}
