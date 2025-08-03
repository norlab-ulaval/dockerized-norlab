#!/bin/bash

function buidl_over_single_build_matrix() {
  local docker_flag=()

  # ....Docker compose build flags.................................................................
  docker_flag+=( build ) # command
#  docker_flag+=( --push )                    # Push service images
#  docker_flag+=( --pull )                    # Always attempt to pull a newer version of the image
#  docker_flag+=( --no-cache )                # Warning: Ok-ish for single compose file with no push/pull dependencies
#  docker_flag+=( --builder local-builder-multiarch-virtual )
#  docker_flag+=( --dry-run )                 # Execute command in dry run mode
#  docker_flag+=( --build-arg DEBIAN_FRONTEND=noninteractive )
#  docker_flag+=( --check )                   # Check build configuration
#  docker_flag+=( --print )                   # Print equivalent bake file
#  docker_flag+=( --with-dependencie )

  # ....l4t-images base images.....................................................................
#  docker_flag+=( l4t-base-image-main )
#  docker_flag+=( l4t-base-image-arm64 )
#  docker_flag+=( l4t-base-image-amd64 )
#  docker_flag+=( l4t-base-image-mimic-l4t-image )
#  docker_flag+=( l4t-base-image-arm64-final )
#  docker_flag+=( l4t-base-image-amd64-final )
#  docker_flag+=( l4t-base-image-arm64-tester )
#  docker_flag+=( l4t-base-image-amd64-tester )

  # ....ROS2-install base images...................................................................
  docker_flag+=( base-image-ros2-clean-main )
  docker_flag+=( base-image-ros2-clean-tester )
  docker_flag+=( base-image-ros2-clean )

  # ....dependencies images........................................................................
#  docker_flag+=( dependencies-core-main )
#  docker_flag+=( dependencies-core-tester )
#  docker_flag+=( dependencies-core )
#  docker_flag+=( dependencies-prompt )
#  docker_flag+=( dependencies-python-science-stack )
#  docker_flag+=( dependencies-ros2-custom-main )
#  docker_flag+=( dependencies-ros2-custom-tester )
#  docker_flag+=( dependencies-ros2-custom )

  # ....dn-control images..........................................................................
  #docker_flag+=( dn-control )

  # ....dn-control-deep images.....................................................................
#  docker_flag+=( dn-control-deep-learning )

#  docker_flag+=( dn-control-deep-rl-main )
#  docker_flag+=( dn-control-deep-rl-tester )
#  docker_flag+=( dn-control-deep-rl ) # final

#  docker_flag+=( dn-control-deep-rl-openai-gym )
#  docker_flag+=( dn-control-deep-rl-openai-gym-tester )

#  docker_flag+=( dn-control-deep-rl-f1tenth-gym-main )
#  docker_flag+=( dn-control-deep-rl-f1tenth-gym-tester )
#  docker_flag+=( dn-control-deep-rl-f1tenth-gym ) # final

  # ....dn-project images............................................................................
#  docker_flag+=( project-core-main )
#  docker_flag+=( project-core )
#  docker_flag+=( project-develop-main )
#  docker_flag+=( project-develop-test-ssh-user )
#  docker_flag+=( project-develop-test-project-user )
#  docker_flag+=( project-develop )
#  docker_flag+=( project-deploy-main )
#  docker_flag+=( project-deploy-tester )
#  docker_flag+=( project-deploy )

  # ....Script flag..................................................................................
  local script_flag=()
#  script_flag+=( --fail-fast )

  # ....Docker env variables.......................................................................
#  export BUILDX_BUILDER=local-builder-multiarch-virtual
#  export BUILDX_BUILDER=remote-norlab-cicd-builder-multiarch-native
  #export DOCKER_CONTEXT=desktop-linux
#  export BUILDKIT_PROGRESS=plain

  # ....Begin......................................................................................
  export NBS_OVERRIDE_ADD_DOCKER_CMD_AND_FLAG="${docker_flag[@]}"

  bash dockerized-norlab-scripts/build_script/dn_build_over_single_build_matrix.bash \
      "build_matrix_config/dev/.env.build_matrix.main.dev" "${script_flag[@]}"

  return 0
}

# ....Execute......................................................................................
echo && clear
buidl_over_single_build_matrix
