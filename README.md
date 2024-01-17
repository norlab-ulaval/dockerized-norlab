
<div align="center">

[//]: # ( ==== Logo ================================================== )
<br>
<br>
<a href="https://norlab.ulaval.ca">
    <picture>
      <source media="(prefers-color-scheme: dark)" srcset="/visual/norlab_logo_acronym_light.png">
      <source media="(prefers-color-scheme: light)" srcset="/visual/norlab_logo_acronym_dark.png">
      <img alt="Shows an the dark NorLab logo in light mode and light NorLab logo in dark mode." src="/visual/norlab_logo_acronym_dark.png" width="175">
    </picture>
</a>
<br>
<br>

[//]: # ( ==== Title ================================================= )
# _Dockerized-NorLab_


[//]: # ( ==== Hyperlink ============================================= )
<sup>
<a href="http://132.203.26.125:8111">NorLab TeamCity GUI</a>
(VPN/intranet access) &nbsp; • &nbsp;
<a href="https://hub.docker.com/repositories/norlabulaval">norlabulaval</a>
(Docker Hub) &nbsp;
<a href="https://github.com/dusty-nv/jetson-containers">Jetson-Containers</a> 
&nbsp; • &nbsp;
</sup>
<br>
<br>

[//]: # ( ==== Description =========================================== )

**Containerized development workflow for NorLab perception or control projects
<br>
leveraging [_docker_](https://www.docker.com) and [_nvidia-docker_](https://github.com/NVIDIA/nvidia-docker)
technology.**
<br>
<br>

[//]: # ( ==== Badges ================================================ )

[![semantic-release: conventional commits](https://img.shields.io/badge/semantic--release-conventional_commits-453032?logo=semantic-release)](https://github.com/semantic-release/semantic-release)
<img alt="GitHub release (with filter)" src="https://img.shields.io/github/v/release/norlab-ulaval/dockerized-norlab">
<a href="http://132.203.26.125:8111"><img src="https://img.shields.io/static/v1?label=JetBrains TeamCity&message=CI/CD&color=green?style=plastic&logo=teamcity" /></a>

[//]: # (Dockerhub image badge)
[//]: # (<a href="https://hub.docker.com/repository/docker/norlabulaval/dn-dependencies/"> <img alt="Docker Image Version &#40;latest semver&#41;" src="https://img.shields.io/docker/v/norlabulaval/dn-dependencies?logo=docker&label=dn-dependencies"> </a>)
[//]: # (<a href="https://hub.docker.com/repository/docker/norlabulaval/dn-control-deep-rl/"> <img alt="Docker Image Version &#40;latest semver&#41;" src="https://img.shields.io/docker/v/norlabulaval/dn-control-deep-rl?logo=docker&label=dn-control-deep-rl"> </a>)



<br>

[//]: # ( ==== Maintainer ============================================ )
<sub>
Maintainer <a href="https://redleader962.github.io">Luc Coupal</a>
</sub>

<br>
<hr style="color:lightgray;background-color:lightgray">
</div>


[//]: # ( ==== Body ================================================== )

### Features
- Multi ros distros: `foxy`, `humble` ...
- Multi ros packages: `core`, `base`, `desktop`
- Multi packages: `libpoitmatcher`, `pytorch`, `torchvision`, `numba`, `stable-baseline`, `f1tenth-gym` ...
- Multi OS versions:
  - Jetpack: `r35.2.1`, `r32.7.1` ...
  - Ubuntu: `focal` (in progress: `bionic` and `jammy`)
- Multi architectures: `l4t/arm64` (in progress: `linux/arm64`, `linux/x86`)
 
### Why
1. custom dependency management; 
2. development environment consistency; 
3. codebase stabilization trough continuous integration pipeline;
4. easy deployment to robots compute box;
5. results reproducibility.
 

### Usage example
Assuming a _docker builder_ with multi-architecture _docker-container_ driver named `local-builder-multiarch-virtual`
execute the following in repository root
```shell
export BUILDX_BUILDER=local-builder-multiarch-virtual \
  && export NBS_OVERRIDE_BUILD_MATRIX_MAIN=".env.build_matrix.main" \
  && export NBS_OVERRIDE_ADD_DOCKER_CMD_AND_FLAG="build" \
  && source dockerized-norlab-scripts/build_script/dn_build_all.bash --force-push
```

Note: To create a multi-architecture _docker builder_ with architecture virtualization
```shell
docker buildx create \
    --name local-builder-multiarch-virtual \
    --driver docker-container \
    --node local \
    --platform linux/amd64,linux/arm64 \
    --bootstrap \
    --use
```
