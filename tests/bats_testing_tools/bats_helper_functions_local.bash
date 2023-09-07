
# ToDo: On 'norlab-shell-script-tools' subsytem update to latest › delete 'fake_IS_TEAMCITY_RUN'
function fake_IS_TEAMCITY_RUN() {
    if [[ ! ${TEAMCITY_VERSION} ]]; then
        TEAMCITY_VERSION=fake
    fi
}
