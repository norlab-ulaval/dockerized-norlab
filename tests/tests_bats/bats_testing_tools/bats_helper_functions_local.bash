
# ToDo: On 'norlab-shell-script-tools' subsytem update to latest › delete 'fake_IS_TEAMCITY_RUN'
function fake_IS_TEAMCITY_RUN() {
    if [[ ! ${TEAMCITY_VERSION} ]]; then
        TEAMCITY_VERSION=fake
    fi
}

function mock_docker_command_exit_ok() {
    function docker() {
      return 0
    }
}

function mock_docker_command_exit_error() {
    function docker() {
      echo "Error" 1>&2
      return 1
    }
}
