
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
- CUDA support via _nvidia-docker_
- Multi architectures: `l4t/arm64`, `linux/amd64`
- Multi OS versions:
  - Jetpack: `r35.2.1`, `r35.4.1`, `r36.2.8` ...
  - Ubuntu: `focal`, `jammy`
- Multi ros2 distros: `foxy`, `humble` ...
- Multi ros2 packages: `core`, `base`, `desktop`
- Multi packages (non-exhaustive list): 
  - Numerical computing: `numpy`, `numba`, `pycuda`, `tensorrt`
  - Deep-learning: `pytorch`, `torch2trt`, `tensordict`
  - Vision: `opencv`, `torchvision`
  - NorLab perception stack: `libpointmatcher[-ros]`, `norlab-icp-mapper[-ros]` 
  - NorLab control stack: (in progress) 
  - MLOps: `ray`, `omegaconf`, `hydra-core`, `wandb`
  - Deep-rl: `stable-baseline3`
  - Simulation: `gym`, `f1tenth-gym`
 
### Why
1. custom dependency management; 
2. development environment consistency; 
3. codebase stabilization trough continuous integration pipeline;
4. easy deployment to robots compute box;
5. results reproducibility.
 
### Clone repository
```shell
 git clone --recurse-submodule https://github.com/norlab-ulaval/dockerized-norlab.git
```

### Usage example
Assuming a _docker builder_ with multi-architecture _docker-container_ driver named `local-builder-multiarch-virtual`
execute the following in repository root
```shell
export BUILDX_BUILDER=local-builder-multiarch-virtual \
  && export NBS_OVERRIDE_BUILD_MATRIX_MAIN=".env.build_matrix.main" \
  && export NBS_OVERRIDE_ADD_DOCKER_CMD_AND_FLAG="build --push" \
  && source dockerized-norlab-scripts/build_script/dn_build_all.bash
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

```shell
 
⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⠟⢿⣿⣿⣿⣿⠃⠀⠀⠀⠀⠀⣿⣿⣿⣿⣿⠿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⠉⠀⠀⠀⠀⠈⢉⣡⣤⠤⠶⠶⠶⠶⢤⣤⣈⡉⠋⠀⠀⠀⠀⠙⢿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⠀⠀⣠⠶⠋⠉⠀⠀⠀⠀⠀⣿⣿⠀⠀⠀⠀⠀⠉⠛⢶⣄⠀⠀⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⠛⠿⢿⡟⢁⡶⠉⠀⠀⠀⠀⠀⠀⠘⣿⣷⣿⣿⣾⡿⠃⠀⠀⠀⠀⠀⠀⠉⢶⡈⢿⣿⣿⡿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⠁⠀⠀⢀⡾⠁⠀⠀⠀⠀⠀⠀⠀⠶⣿⣦⡀⣿⣿⣀⣴⣿⣦⠴⠒⠒⠒⠒⠦⢤⣌⣶⠀⠀⠀⠈⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣄⠀⠀⢠⠋⣶⣤⡀⢻⣿⠀⣿⡇⠀⣤⣄⠉⠻⣿⣿⠛⢁⠋⠀⠀⣶⣶⠀⣿⡆⢀⣤⠀⠀⣰⠁⠀⠀⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡿⢠⠃⠀⠀⠉⣻⣿⣿⣤⣿⣿⠀⣿⠛⠿⣿⣿⣿⣿⣿⢸⣿⠀⣿⣟⣤⣿⣿⣟⠉⠀⢠⠻⡀⣴⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡿⠀⡟⠀⠀⠈⠟⠋⠁⣤⣿⣿⣿⣶⣿⠂⠀⠀⣿⣿⠀⣿⢸⣿⣶⣿⣿⣿⣄⠈⠙⠁⢀⠋⠀⣿⠈⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⠀⠀⠀⢰⠁⠀⠀⠀⠀⠹⠟⠉⣀⣴⣾⡿⠛⠿⣿⣦⣿⣿⣿⣿⠟⠛⢿⣷⣤⡀⠉⠋⠀⣠⠋⠀⠀⢸⠀⠀⠀⠉⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
······················································•· Dockerized-NorLab ··•••···················································
⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⠀⠀⠀⢸⠀⠀⠀⠀⠀⢀⡀⠈⠻⣿⣶⣄⠀⣀⣶⣿⣿⣿⣿⣦⣀⠀⣹⣦⣭⣥⣤⢖⠋⠀⠀⠀⠀⢸⠀⠀⠀⢸⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣷⣶⣤⠀⡇⠀⠀⠀⡀⠉⠛⣿⣷⣦⣴⣿⣿⠛⠁⠀⣿⣿⠀⠉⢻⣿⣿⣤⣴⣾⡿⠛⠉⣀⠀⠀⠀⢸⠀⢀⣠⣼⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡆⢻⠀⠀⠈⠛⢿⣿⣾⣿⣿⣿⠀⣿⠀⣀⣴⣿⣿⣦⡀⢸⣿⠀⣿⡿⣿⣶⣿⠿⠛⠀⠀⠀⡟⣸⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⠟⠀⠀⢷⠀⣶⣿⠟⣻⣿⠀⣿⡏⠀⣿⡿⠛⢁⣿⣿⠉⠛⣿⣿⠀⣿⣿⠀⣿⡟⠿⣿⣶⠀⡿⠀⠙⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣄⠀⠀⠀⠻⡀⠀⠀⠛⠛⠀⠉⠁⠀⣀⣶⣿⠟⣿⣿⠿⣿⣦⡀⠀⠉⠉⠀⠛⠃⠀⠀⢠⠟⠀⠀⠀⣰⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣦⣀⣀⣤⡈⢷⡀⠀⠀⠀⠀⠀⠀⠀⠁⣠⣾⣿⣿⣶⣄⠉⠀⠀⠀⠀⠀⠀⠀⢀⡾⢁⡀⠀⠀⣾⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣶⠈⠓⣤⠀⠀⠀⠀⠀⠈⠋⠀⣿⣿⠀⠛⠀⠀⠀⠀⠀⢀⣤⠛⢠⣾⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡏⠀⠀⠀⠀⠉⠓⢦⣤⣀⡀⠀⠉⠉⠀⢀⣀⣤⡴⠚⠉⠀⠀⠀⠀⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣦⣀⠀⣴⣿⣿⣶⣆⠀⠀⠀⠀⠀⠀⣶⣶⣿⣶⡀⠀⣠⣶⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣄⣀⣀⣀⣀⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
 
                                                     https://norlab.ulaval.ca                                                     
                                  https://github.com/norlab-ulaval/norlab-shell-script-tools.git                                  
 
 
[NBS done] dn_build_all.bash execution summary

...................................................................................................................................




...................................................................................................................................

Service crawled:

   - l4t-base-image-main
   - l4t-base-image-amd64
   - l4t-base-image-mimic-l4t-image
   - l4t-base-image-amd64-final
   - l4t-base-image-amd64-tester
   - l4t-base-image-arm64
   - l4t-base-image-arm64-final
   - l4t-base-image-arm64-tester

with tag:

   Pass › DN-hot-l4t-pytorch-r35.2.1


...................................................................................................................................

Service crawled:

   - base-image-ros2-clean-main
   - base-image-ros2-clean-tester
   - base-image-ros2-clean

with tag:

   Pass › DN-hot-foxy-core-l4t-pytorch-r35.2.1
   Pass › DN-hot-foxy-base-l4t-pytorch-r35.2.1
   Pass › DN-hot-foxy-desktop-l4t-pytorch-r35.2.1


...................................................................................................................................

Service crawled:

   - dependencies-core-main
   - dependencies-core
   - dependencies-prompt
   - dependencies-python-science-stack
   - dependencies-ros2-custom-main
   - dependencies-ros2-custom-tester
   - dependencies-ros2-custom

with tag:

   Pass › DN-hot-foxy-core-l4t-pytorch-r35.2.1
   Pass › DN-hot-foxy-base-l4t-pytorch-r35.2.1
   Pass › DN-hot-foxy-desktop-l4t-pytorch-r35.2.1


...................................................................................................................................

Service crawled:

   - dn-control

with tag:

   Pass › DN-hot-foxy-core-l4t-pytorch-r35.2.1
   Pass › DN-hot-foxy-base-l4t-pytorch-r35.2.1
   Pass › DN-hot-foxy-desktop-l4t-pytorch-r35.2.1


...................................................................................................................................

Service crawled:

   - dn-control-deep-learning
   - dn-control-deep-rl
   - dn-control-deep-rl-openai-gym
   - dn-control-deep-rl-f1tenth

with tag:

   Pass › DN-hot-foxy-core-l4t-pytorch-r35.2.1
   Pass › DN-hot-foxy-base-l4t-pytorch-r35.2.1
   Pass › DN-hot-foxy-desktop-l4t-pytorch-r35.2.1


...................................................................................................................................

Service crawled:

   - dn-perception-main
   - dn-perception
   - dn-perception-libpointmatcher-main
   - dn-perception-libpointmatcher-tester
   - dn-perception-libpointmatcher
   - dn-perception-icp-mapper-main
   - dn-perception-icp-mapper-tester
   - dn-perception-icp-mapper
   - dn-perception-norlab-stack-main
   - dn-perception-norlab-stack-tester
   - dn-perception-norlab-stack

with tag:

   Pass › DN-hot-foxy-core-l4t-pytorch-r35.2.1
   Pass › DN-hot-foxy-base-l4t-pytorch-r35.2.1
   Pass › DN-hot-foxy-desktop-l4t-pytorch-r35.2.1


...................................................................................................................................

Service crawled:

   - project-core-main
   - project-core
   - project-develop-main
   - project-develop-tester
   - project-deploy-main
   - project-deploy-tester
   - project-deploy
   - project-develop

with tag:

   Pass › DN-hot-foxy-core-l4t-pytorch-r35.2.1
   Pass › DN-hot-foxy-base-l4t-pytorch-r35.2.1
   Pass › DN-hot-foxy-desktop-l4t-pytorch-r35.2.1


...................................................................................................................................


Completed dn_build_all.bash
▉▉▉▉▉▉▉▉▉▉▉▉▉▉▉▉▉▉▉▉▉▉▉▉▉▉▉▉▉▉▉▉▉▉▉▉▉▉▉▉▉▉▉▉▉▉▉▉▉▉▉▉▉▉▉▉▉▉▉▉▉▉▉▉▉▉▉▉▉▉▉▉▉▉▉▉▉▉▉▉▉▉▉▉▉▉▉▉▉▉▉▉▉▉▉▉▉▉▉▉▉▉▉▉▉▉▉▉▉▉▉▉▉▉▉▉▉▉▉▉▉▉▉▉▉▉▉▉▉▉▉



```
