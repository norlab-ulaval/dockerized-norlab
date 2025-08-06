
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

**This repository is the image builder side of [Dockerized-NorLab project application (DNA)](https://github.com/norlab-ulaval/dockerized-norlab-project). 
<br>
Dockerized-NorLab is a containerized development workflow for NorLab robotic software engineering project leveraging [_Docker_](https://www.docker.com) and [_NVIDIA Container Toolkit_](https://github.com/NVIDIA/nvidia-container-toolkit).**
<br>
<br>
<br>

[//]: # ( ==== Badges ================================================ )

[![semantic-release: conventional commits](https://img.shields.io/badge/semantic--release-conventional_commits-453032?logo=semantic-release)](https://github.com/semantic-release/semantic-release)
<img alt="GitHub release (with filter)" src="https://img.shields.io/github/v/release/norlab-ulaval/dockerized-norlab">
<a href="http://132.203.26.125:8111"><img alt="Static Badge" src="https://img.shields.io/badge/JetBrains%20TeamCity-CI-green?style=plastic&logo=teamcity"></a>

<br>

[//]: # ( ==== Maintainer ============================================ )
<sub>
Maintainer <a href="https://redleader962.github.io">Luc Coupal</a>
</sub>

<br>
<hr style="color:lightgray;background-color:lightgray">
</div>


[//]: # ( ==== Body ================================================== )

## Features
- CUDA support via _NVIDIA Container Toolkit_
- Multi architectures: `l4t/arm64`, `linux/amd64`
- Multi OS versions:
  - Jetpack: `r36.4.0`, `r35.4.1`
  - Ubuntu: `jammy`, `focal`
- Multi ros2 distros: `humble`, `galactic`
- Multi ros2 packages: `core`, `base`, `desktop`
- Multi packages (non-exhaustive list): 
  - Deep-learning: `pytorch`, `torch2trt`, `tensordict`
  - Deep-rl: `torchrl`
  - Numerical computing: `numpy`, `numba`, `pycuda`, `tensorrt`
  - MLOps: `ray`, `omegaconf`, `hydra-core`

## Why
1. custom dependency management; 
2. development environment consistency; 
3. codebase stabilization trough continuous integration pipeline;
4. easy deployment to robots compute box;
5. results reproducibility.
 
Recommanded reading by [Tobit Flatscher](https://github.com/2b-t)
- [Why should I use Docker when developing robotics software?](https://github.com/2b-t/docker-for-robotics/blob/main/doc/Motivation.md#2-why-should-i-use-docker-when-developing-robotics-software)
- [Why is Docker important in particular for academic and research institutions?](https://github.com/2b-t/docker-for-robotics/blob/main/doc/Motivation.md#3-why-is-docker-important-in-particular-for-academic-and-research-institutions)


---

# Basic usage

## The easy way
Use **[Dockerized-NorLab project application (DNA)](https://github.com/norlab-ulaval/dockerized-norlab-project)**, its made to work with DN images.

## Manual usage instructions
Note: _**Dockerized-Norlab**_ images are intended to be used as base images for **[Dockerized-NorLab project application (DNA)](https://github.com/norlab-ulaval/dockerized-norlab-project)**.

1. Pick an images that fit your project, the most basic one is [dockerized-norlab-dependencies-full](https://hub.docker.com/repository/docker/norlabulaval/dockerized-norlab-dependencies-full)
2. Pick the tag that fit your needs, e.g. `DN-bleeding-foxy-base-l4t-pytorch-r35.4.1`  
   Notes: `bleeding` is for the DN `dev` branch, `foxy` is the ros version, `base` is the ROS package type, `l4t-pytorch` is the Jetson container pkg, `r35.4.1` is the _Jetpack_ version 
3. Pull the image from _Dockerhub_
    ```shell
    docker pull norlabulaval/dockerized-norlab-dependencies-full:DN-bleeding-foxy-base-l4t-pytorch-r35.4.1
    ```
4. Pull the image from _Dockerhub_
    
   ```shell
    docker run -it --rm \
        --env ROS_DOMAIN_ID=1 --env DISPLAY="${DISPLAY}" --env QT_X11_NO_MITSHM=1 \
        --net host -p 2222:2222 -p 6006:6006 -p 7777:7777 \
        --privileged \
        -v /etc/localtime:/etc/localtime:ro -v /tmp/.X11-unix:/tmp/.X11-unix:rw -v /tmp/.docker.xauth:/tmp/.docker.xauth:rw \
        norlabulaval/dockerized-norlab-dependencies-full:DN-bleeding-foxy-base-l4t-pytorch-r35.4.1
    ``` 
    Note: 
    - Add the `--runtime nvidia` flag if you have nvidia gpu and _NVIDIA Container Toolkit_ is installed.
    - For _MacOs_ user, replace the line `--net host -p 2222:2222 -p 6006:6006 -p 7777:7777` with `--net bridge`. 


---

# Available images

Notes:
- All images are multi-architecture build (amr64 and amd64). 
- Most have CUDA support via _NVIDIA Container Toolkit_ base image, both for L4T (aka Jetson OS) and ubuntu.
- Bleeding tags are build from the latest `dev` branch version (e.g. `DN-bleeding-foxy-base-l4t-pytorch-r35.4.1`) and are rebuild each week. 
- All images also have a tag pinned for each release version, starting from release v0.5.2, e.g. `DN-v0.5.2-foxy-ros-core-l4t-r35.2.1`.


### Dockerized-Norlab Control

#### [dockerized-norlab-control-deep-rl](https://hub.docker.com/repository/docker/norlabulaval/dockerized-norlab-control-deep-rl)   
  ![Docker Image Version (tag)](https://img.shields.io/docker/v/norlabulaval/dockerized-norlab-control-deep-rl/DN-bleeding-foxy-desktop-l4t-pytorch-r36.4.0?logo=docker&color=blue&link=https%3A%2F%2Fhub.docker.com%2Frepository%2Fdocker%2Fnorlabulaval%2Fdockerized-norlab-control-deep-rl)
  ![Docker Image Version (tag)](https://img.shields.io/docker/v/norlabulaval/dockerized-norlab-control-deep-rl/DN-bleeding-foxy-base-l4t-pytorch-r36.4.0?logo=docker&color=blue&link=https%3A%2F%2Fhub.docker.com%2Frepository%2Fdocker%2Fnorlabulaval%2Fdockerized-norlab-control-deep-rl)
  ![Docker Image Version (tag)](https://img.shields.io/docker/v/norlabulaval/dockerized-norlab-control-deep-rl/DN-bleeding-foxy-core-l4t-pytorch-r36.4.0?logo=docker&color=blue&link=https%3A%2F%2Fhub.docker.com%2Frepository%2Fdocker%2Fnorlabulaval%2Fdockerized-norlab-control-deep-rl)

  ![Docker Image Version (tag)](https://img.shields.io/docker/v/norlabulaval/dockerized-norlab-control-deep-rl/DN-bleeding-foxy-desktop-l4t-pytorch-r35.4.1?logo=docker&color=blue&link=https%3A%2F%2Fhub.docker.com%2Frepository%2Fdocker%2Fnorlabulaval%2Fdockerized-norlab-control-deep-rl)
  ![Docker Image Version (tag)](https://img.shields.io/docker/v/norlabulaval/dockerized-norlab-control-deep-rl/DN-bleeding-foxy-base-l4t-pytorch-r35.4.1?logo=docker&color=blue&link=https%3A%2F%2Fhub.docker.com%2Frepository%2Fdocker%2Fnorlabulaval%2Fdockerized-norlab-control-deep-rl)
  ![Docker Image Version (tag)](https://img.shields.io/docker/v/norlabulaval/dockerized-norlab-control-deep-rl/DN-bleeding-foxy-core-l4t-pytorch-r35.4.1?logo=docker&color=blue&link=https%3A%2F%2Fhub.docker.com%2Frepository%2Fdocker%2Fnorlabulaval%2Fdockerized-norlab-control-deep-rl)


#### [dockerized-norlab-control-deep-rl-f1tenth](https://hub.docker.com/repository/docker/norlabulaval/dockerized-norlab-control-deep-rl-f1tenth) (alias [dockerized-norlab-control-deep-rl-f1tenth-gym](https://hub.docker.com/repository/docker/norlabulaval/dockerized-norlab-control-deep-rl-f1tenth-gym) )
  ![Docker Image Version (tag)](https://img.shields.io/docker/v/norlabulaval/dockerized-norlab-control-deep-rl-f1tenth/DN-bleeding-foxy-desktop-l4t-pytorch-r36.4.0?logo=docker&color=blue&link=https%3A%2F%2Fhub.docker.com%2Frepository%2Fdocker%2Fnorlabulaval%2Fdockerized-norlab-control-deep-rl-f1tenth)
  ![Docker Image Version (tag)](https://img.shields.io/docker/v/norlabulaval/dockerized-norlab-control-deep-rl-f1tenth/DN-bleeding-foxy-base-l4t-pytorch-r36.4.0?logo=docker&color=blue&link=https%3A%2F%2Fhub.docker.com%2Frepository%2Fdocker%2Fnorlabulaval%2Fdockerized-norlab-control-deep-rl-f1tenth)
  ![Docker Image Version (tag)](https://img.shields.io/docker/v/norlabulaval/dockerized-norlab-control-deep-rl-f1tenth/DN-bleeding-foxy-core-l4t-pytorch-r36.4.0?logo=docker&color=blue&link=https%3A%2F%2Fhub.docker.com%2Frepository%2Fdocker%2Fnorlabulaval%2Fdockerized-norlab-control-deep-rl-f1tenth)

  ![Docker Image Version (tag)](https://img.shields.io/docker/v/norlabulaval/dockerized-norlab-control-deep-rl-f1tenth/DN-bleeding-foxy-desktop-l4t-pytorch-r35.4.1?logo=docker&color=blue&link=https%3A%2F%2Fhub.docker.com%2Frepository%2Fdocker%2Fnorlabulaval%2Fdockerized-norlab-control-deep-rl-f1tenth)
  ![Docker Image Version (tag)](https://img.shields.io/docker/v/norlabulaval/dockerized-norlab-control-deep-rl-f1tenth/DN-bleeding-foxy-base-l4t-pytorch-r35.4.1?logo=docker&color=blue&link=https%3A%2F%2Fhub.docker.com%2Frepository%2Fdocker%2Fnorlabulaval%2Fdockerized-norlab-control-deep-rl-f1tenth)
  ![Docker Image Version (tag)](https://img.shields.io/docker/v/norlabulaval/dockerized-norlab-control-deep-rl-f1tenth/DN-bleeding-foxy-core-l4t-pytorch-r35.4.1?logo=docker&color=blue&link=https%3A%2F%2Fhub.docker.com%2Frepository%2Fdocker%2Fnorlabulaval%2Fdockerized-norlab-control-deep-rl-f1tenth)


#### [dockerized-norlab-control-deep-rl-openai-gym](https://hub.docker.com/repository/docker/norlabulaval/dockerized-norlab-control-deep-rl-openai-gym)  (legacy Gym)
  ![Docker Image Version (tag)](https://img.shields.io/docker/v/norlabulaval/dockerized-norlab-control-deep-rl-openai-gym/DN-bleeding-foxy-desktop-l4t-pytorch-r36.4.0?logo=docker&color=blue&link=https%3A%2F%2Fhub.docker.com%2Frepository%2Fdocker%2Fnorlabulaval%2Fdockerized-norlab-control-deep-rl-openai-gym)
  ![Docker Image Version (tag)](https://img.shields.io/docker/v/norlabulaval/dockerized-norlab-control-deep-rl-openai-gym/DN-bleeding-foxy-base-l4t-pytorch-r36.4.0?logo=docker&color=blue&link=https%3A%2F%2Fhub.docker.com%2Frepository%2Fdocker%2Fnorlabulaval%2Fdockerized-norlab-control-deep-rl-openai-gym)
  ![Docker Image Version (tag)](https://img.shields.io/docker/v/norlabulaval/dockerized-norlab-control-deep-rl-openai-gym/DN-bleeding-foxy-core-l4t-pytorch-r36.4.0?logo=docker&color=blue&link=https%3A%2F%2Fhub.docker.com%2Frepository%2Fdocker%2Fnorlabulaval%2Fdockerized-norlab-control-deep-rl-openai-gym)

  ![Docker Image Version (tag)](https://img.shields.io/docker/v/norlabulaval/dockerized-norlab-control-deep-rl-openai-gym/DN-bleeding-foxy-desktop-l4t-pytorch-r35.4.1?logo=docker&color=blue&link=https%3A%2F%2Fhub.docker.com%2Frepository%2Fdocker%2Fnorlabulaval%2Fdockerized-norlab-control-deep-rl-openai-gym)
  ![Docker Image Version (tag)](https://img.shields.io/docker/v/norlabulaval/dockerized-norlab-control-deep-rl-openai-gym/DN-bleeding-foxy-base-l4t-pytorch-r35.4.1?logo=docker&color=blue&link=https%3A%2F%2Fhub.docker.com%2Frepository%2Fdocker%2Fnorlabulaval%2Fdockerized-norlab-control-deep-rl-openai-gym)
  ![Docker Image Version (tag)](https://img.shields.io/docker/v/norlabulaval/dockerized-norlab-control-deep-rl-openai-gym/DN-bleeding-foxy-core-l4t-pytorch-r35.4.1?logo=docker&color=blue&link=https%3A%2F%2Fhub.docker.com%2Frepository%2Fdocker%2Fnorlabulaval%2Fdockerized-norlab-control-deep-rl-openai-gym)

#### [dockerized-norlab-control-deep-learning](https://hub.docker.com/repository/docker/norlabulaval/dockerized-norlab-control-deep-learning)   
  ![Docker Image Version (tag)](https://img.shields.io/docker/v/norlabulaval/dockerized-norlab-control-deep-learning/DN-bleeding-foxy-desktop-l4t-pytorch-r36.4.0?logo=docker&color=blue&link=https%3A%2F%2Fhub.docker.com%2Frepository%2Fdocker%2Fnorlabulaval%2Fdockerized-norlab-control-deep-learning)
  ![Docker Image Version (tag)](https://img.shields.io/docker/v/norlabulaval/dockerized-norlab-control-deep-learning/DN-bleeding-foxy-base-l4t-pytorch-r36.4.0?logo=docker&color=blue&link=https%3A%2F%2Fhub.docker.com%2Frepository%2Fdocker%2Fnorlabulaval%2Fdockerized-norlab-control-deep-learning)
  ![Docker Image Version (tag)](https://img.shields.io/docker/v/norlabulaval/dockerized-norlab-control-deep-learning/DN-bleeding-foxy-core-l4t-pytorch-r36.4.0?logo=docker&color=blue&link=https%3A%2F%2Fhub.docker.com%2Frepository%2Fdocker%2Fnorlabulaval%2Fdockerized-norlab-control-deep-learning)

  ![Docker Image Version (tag)](https://img.shields.io/docker/v/norlabulaval/dockerized-norlab-control-deep-learning/DN-bleeding-foxy-desktop-l4t-pytorch-r35.4.1?logo=docker&color=blue&link=https%3A%2F%2Fhub.docker.com%2Frepository%2Fdocker%2Fnorlabulaval%2Fdockerized-norlab-control-deep-learning)
  ![Docker Image Version (tag)](https://img.shields.io/docker/v/norlabulaval/dockerized-norlab-control-deep-learning/DN-bleeding-foxy-base-l4t-pytorch-r35.4.1?logo=docker&color=blue&link=https%3A%2F%2Fhub.docker.com%2Frepository%2Fdocker%2Fnorlabulaval%2Fdockerized-norlab-control-deep-learning)
  ![Docker Image Version (tag)](https://img.shields.io/docker/v/norlabulaval/dockerized-norlab-control-deep-learning/DN-bleeding-foxy-core-l4t-pytorch-r35.4.1?logo=docker&color=blue&link=https%3A%2F%2Fhub.docker.com%2Frepository%2Fdocker%2Fnorlabulaval%2Fdockerized-norlab-control-deep-learning)


### Dockerized-Norlab Dependencies
#### [dockerized-norlab-dependencies-full](https://hub.docker.com/repository/docker/norlabulaval/dockerized-norlab-dependencies-full) (alias [dockerized-norlab-dependencies-ros2-custom](https://hub.docker.com/repository/docker/norlabulaval/dockerized-norlab-dependencies-ros2-custom) )    
  ![Docker Image Version (tag)](https://img.shields.io/docker/v/norlabulaval/dockerized-norlab-dependencies-full/DN-bleeding-foxy-desktop-l4t-pytorch-r36.4.0?logo=docker&color=blue&link=https%3A%2F%2Fhub.docker.com%2Frepository%2Fdocker%2Fnorlabulaval%2Fdockerized-norlab-dependencies-full)
  ![Docker Image Version (tag)](https://img.shields.io/docker/v/norlabulaval/dockerized-norlab-dependencies-full/DN-bleeding-foxy-base-l4t-pytorch-r36.4.0?logo=docker&color=blue&link=https%3A%2F%2Fhub.docker.com%2Frepository%2Fdocker%2Fnorlabulaval%2Fdockerized-norlab-dependencies-full)
  ![Docker Image Version (tag)](https://img.shields.io/docker/v/norlabulaval/dockerized-norlab-dependencies-full/DN-bleeding-foxy-core-l4t-pytorch-r36.4.0?logo=docker&color=blue&link=https%3A%2F%2Fhub.docker.com%2Frepository%2Fdocker%2Fnorlabulaval%2Fdockerized-norlab-dependencies-full)

  ![Docker Image Version (tag)](https://img.shields.io/docker/v/norlabulaval/dockerized-norlab-dependencies-full/DN-bleeding-foxy-desktop-l4t-pytorch-r35.4.1?logo=docker&color=blue&link=https%3A%2F%2Fhub.docker.com%2Frepository%2Fdocker%2Fnorlabulaval%2Fdockerized-norlab-dependencies-full)
  ![Docker Image Version (tag)](https://img.shields.io/docker/v/norlabulaval/dockerized-norlab-dependencies-full/DN-bleeding-foxy-base-l4t-pytorch-r35.4.1?logo=docker&color=blue&link=https%3A%2F%2Fhub.docker.com%2Frepository%2Fdocker%2Fnorlabulaval%2Fdockerized-norlab-dependencies-full)
  ![Docker Image Version (tag)](https://img.shields.io/docker/v/norlabulaval/dockerized-norlab-dependencies-full/DN-bleeding-foxy-core-l4t-pytorch-r35.4.1?logo=docker&color=blue&link=https%3A%2F%2Fhub.docker.com%2Frepository%2Fdocker%2Fnorlabulaval%2Fdockerized-norlab-dependencies-full)

#### [dockerized-norlab-dependencies-python-science-stack](https://hub.docker.com/repository/docker/norlabulaval/dockerized-norlab-dependencies-python-science-stack)   
  ![Docker Image Version (tag)](https://img.shields.io/docker/v/norlabulaval/dockerized-norlab-dependencies-python-science-stack/DN-bleeding-foxy-desktop-l4t-pytorch-r36.4.0?logo=docker&color=blue&link=https%3A%2F%2Fhub.docker.com%2Frepository%2Fdocker%2Fnorlabulaval%2Fdockerized-norlab-dependencies-python-science-stack)
  ![Docker Image Version (tag)](https://img.shields.io/docker/v/norlabulaval/dockerized-norlab-dependencies-python-science-stack/DN-bleeding-foxy-base-l4t-pytorch-r36.4.0?logo=docker&color=blue&link=https%3A%2F%2Fhub.docker.com%2Frepository%2Fdocker%2Fnorlabulaval%2Fdockerized-norlab-dependencies-python-science-stack)
  ![Docker Image Version (tag)](https://img.shields.io/docker/v/norlabulaval/dockerized-norlab-dependencies-python-science-stack/DN-bleeding-foxy-core-l4t-pytorch-r36.4.0?logo=docker&color=blue&link=https%3A%2F%2Fhub.docker.com%2Frepository%2Fdocker%2Fnorlabulaval%2Fdockerized-norlab-dependencies-python-science-stack)
  
  ![Docker Image Version (tag)](https://img.shields.io/docker/v/norlabulaval/dockerized-norlab-dependencies-python-science-stack/DN-bleeding-foxy-desktop-l4t-pytorch-r35.4.1?logo=docker&color=blue&link=https%3A%2F%2Fhub.docker.com%2Frepository%2Fdocker%2Fnorlabulaval%2Fdockerized-norlab-dependencies-python-science-stack)
  ![Docker Image Version (tag)](https://img.shields.io/docker/v/norlabulaval/dockerized-norlab-dependencies-python-science-stack/DN-bleeding-foxy-base-l4t-pytorch-r35.4.1?logo=docker&color=blue&link=https%3A%2F%2Fhub.docker.com%2Frepository%2Fdocker%2Fnorlabulaval%2Fdockerized-norlab-dependencies-python-science-stack)
  ![Docker Image Version (tag)](https://img.shields.io/docker/v/norlabulaval/dockerized-norlab-dependencies-python-science-stack/DN-bleeding-foxy-core-l4t-pytorch-r35.4.1?logo=docker&color=blue&link=https%3A%2F%2Fhub.docker.com%2Frepository%2Fdocker%2Fnorlabulaval%2Fdockerized-norlab-dependencies-python-science-stack)

#### [dockerized-norlab-dependencies-prompt](https://hub.docker.com/repository/docker/norlabulaval/dockerized-norlab-dependencies-prompt)   
  ![Docker Image Version (tag)](https://img.shields.io/docker/v/norlabulaval/dockerized-norlab-dependencies-prompt/DN-bleeding-foxy-desktop-l4t-pytorch-r36.4.0?logo=docker&color=blue&link=https%3A%2F%2Fhub.docker.com%2Frepository%2Fdocker%2Fnorlabulaval%2Fdockerized-norlab-dependencies-prompt)
  ![Docker Image Version (tag)](https://img.shields.io/docker/v/norlabulaval/dockerized-norlab-dependencies-prompt/DN-bleeding-foxy-base-l4t-pytorch-r36.4.0?logo=docker&color=blue&link=https%3A%2F%2Fhub.docker.com%2Frepository%2Fdocker%2Fnorlabulaval%2Fdockerized-norlab-dependencies-prompt)
  ![Docker Image Version (tag)](https://img.shields.io/docker/v/norlabulaval/dockerized-norlab-dependencies-prompt/DN-bleeding-foxy-core-l4t-pytorch-r36.4.0?logo=docker&color=blue&link=https%3A%2F%2Fhub.docker.com%2Frepository%2Fdocker%2Fnorlabulaval%2Fdockerized-norlab-dependencies-prompt)
  
  ![Docker Image Version (tag)](https://img.shields.io/docker/v/norlabulaval/dockerized-norlab-dependencies-prompt/DN-bleeding-foxy-desktop-l4t-pytorch-r35.4.1?logo=docker&color=blue&link=https%3A%2F%2Fhub.docker.com%2Frepository%2Fdocker%2Fnorlabulaval%2Fdockerized-norlab-dependencies-prompt)
  ![Docker Image Version (tag)](https://img.shields.io/docker/v/norlabulaval/dockerized-norlab-dependencies-prompt/DN-bleeding-foxy-base-l4t-pytorch-r35.4.1?logo=docker&color=blue&link=https%3A%2F%2Fhub.docker.com%2Frepository%2Fdocker%2Fnorlabulaval%2Fdockerized-norlab-dependencies-prompt)
  ![Docker Image Version (tag)](https://img.shields.io/docker/v/norlabulaval/dockerized-norlab-dependencies-prompt/DN-bleeding-foxy-core-l4t-pytorch-r35.4.1?logo=docker&color=blue&link=https%3A%2F%2Fhub.docker.com%2Frepository%2Fdocker%2Fnorlabulaval%2Fdockerized-norlab-dependencies-prompt)

#### [dockerized-norlab-dependencies-core](https://hub.docker.com/repository/docker/norlabulaval/dockerized-norlab-dependencies-core)   
  ![Docker Image Version (tag)](https://img.shields.io/docker/v/norlabulaval/dockerized-norlab-dependencies-core/DN-bleeding-foxy-desktop-l4t-pytorch-r36.4.0?logo=docker&color=blue&link=https%3A%2F%2Fhub.docker.com%2Frepository%2Fdocker%2Fnorlabulaval%2Fdockerized-norlab-dependencies-core)
  ![Docker Image Version (tag)](https://img.shields.io/docker/v/norlabulaval/dockerized-norlab-dependencies-core/DN-bleeding-foxy-base-l4t-pytorch-r36.4.0?logo=docker&color=blue&link=https%3A%2F%2Fhub.docker.com%2Frepository%2Fdocker%2Fnorlabulaval%2Fdockerized-norlab-dependencies-core)
  ![Docker Image Version (tag)](https://img.shields.io/docker/v/norlabulaval/dockerized-norlab-dependencies-core/DN-bleeding-foxy-core-l4t-pytorch-r36.4.0?logo=docker&color=blue&link=https%3A%2F%2Fhub.docker.com%2Frepository%2Fdocker%2Fnorlabulaval%2Fdockerized-norlab-dependencies-core)
  
  ![Docker Image Version (tag)](https://img.shields.io/docker/v/norlabulaval/dockerized-norlab-dependencies-core/DN-bleeding-foxy-desktop-l4t-pytorch-r35.4.1?logo=docker&color=blue&link=https%3A%2F%2Fhub.docker.com%2Frepository%2Fdocker%2Fnorlabulaval%2Fdockerized-norlab-dependencies-core)
  ![Docker Image Version (tag)](https://img.shields.io/docker/v/norlabulaval/dockerized-norlab-dependencies-core/DN-bleeding-foxy-base-l4t-pytorch-r35.4.1?logo=docker&color=blue&link=https%3A%2F%2Fhub.docker.com%2Frepository%2Fdocker%2Fnorlabulaval%2Fdockerized-norlab-dependencies-core)
  ![Docker Image Version (tag)](https://img.shields.io/docker/v/norlabulaval/dockerized-norlab-dependencies-core/DN-bleeding-foxy-core-l4t-pytorch-r35.4.1?logo=docker&color=blue&link=https%3A%2F%2Fhub.docker.com%2Frepository%2Fdocker%2Fnorlabulaval%2Fdockerized-norlab-dependencies-core)


### Base images

#### [dockerized-norlab-base-image-ros2-clean](https://hub.docker.com/repository/docker/norlabulaval/dockerized-norlab-base-image-ros2-clean)    
  ![Docker Image Version (tag)](https://img.shields.io/docker/v/norlabulaval/dockerized-norlab-base-image-ros2-clean/DN-bleeding-foxy-desktop-l4t-pytorch-r36.4.0?logo=docker&color=blue&link=https%3A%2F%2Fhub.docker.com%2Frepository%2Fdocker%2Fnorlabulaval%2Fdockerized-norlab-base-image-ros2-clean)
  ![Docker Image Version (tag)](https://img.shields.io/docker/v/norlabulaval/dockerized-norlab-base-image-ros2-clean/DN-bleeding-foxy-base-l4t-pytorch-r36.4.0?logo=docker&color=blue&link=https%3A%2F%2Fhub.docker.com%2Frepository%2Fdocker%2Fnorlabulaval%2Fdockerized-norlab-base-image-ros2-clean)
  ![Docker Image Version (tag)](https://img.shields.io/docker/v/norlabulaval/dockerized-norlab-base-image-ros2-clean/DN-bleeding-foxy-core-l4t-pytorch-r36.4.0?logo=docker&color=blue&link=https%3A%2F%2Fhub.docker.com%2Frepository%2Fdocker%2Fnorlabulaval%2Fdockerized-norlab-base-image-ros2-clean)
  
 ![Docker Image Version (tag)](https://img.shields.io/docker/v/norlabulaval/dockerized-norlab-base-image-ros2-clean/DN-bleeding-foxy-desktop-l4t-pytorch-r35.4.1?logo=docker&color=blue&link=https%3A%2F%2Fhub.docker.com%2Frepository%2Fdocker%2Fnorlabulaval%2Fdockerized-norlab-base-image-ros2-clean)
  ![Docker Image Version (tag)](https://img.shields.io/docker/v/norlabulaval/dockerized-norlab-base-image-ros2-clean/DN-bleeding-foxy-base-l4t-pytorch-r35.4.1?logo=docker&color=blue&link=https%3A%2F%2Fhub.docker.com%2Frepository%2Fdocker%2Fnorlabulaval%2Fdockerized-norlab-base-image-ros2-clean)
  ![Docker Image Version (tag)](https://img.shields.io/docker/v/norlabulaval/dockerized-norlab-base-image-ros2-clean/DN-bleeding-foxy-core-l4t-pytorch-r35.4.1?logo=docker&color=blue&link=https%3A%2F%2Fhub.docker.com%2Frepository%2Fdocker%2Fnorlabulaval%2Fdockerized-norlab-base-image-ros2-clean)

#### [dockerized-vaul-base-image-ros2-clean](https://hub.docker.com/repository/docker/norlabulaval/dockerized-vaul-base-image-ros2-clean)    
  ![Docker Image Version (tag)](https://img.shields.io/docker/v/norlabulaval/dockerized-vaul-base-image-ros2-clean/DN-bleeding-foxy-desktop-l4t-pytorch-r36.4.0?logo=docker&color=blue&link=https%3A%2F%2Fhub.docker.com%2Frepository%2Fdocker%2Fnorlabulaval%2Fdockerized-vaul-base-image-ros2-clean)
  ![Docker Image Version (tag)](https://img.shields.io/docker/v/norlabulaval/dockerized-vaul-base-image-ros2-clean/DN-bleeding-foxy-base-l4t-pytorch-r36.4.0?logo=docker&color=blue&link=https%3A%2F%2Fhub.docker.com%2Frepository%2Fdocker%2Fnorlabulaval%2Fdockerized-vaul-base-image-ros2-clean)
  ![Docker Image Version (tag)](https://img.shields.io/docker/v/norlabulaval/dockerized-vaul-base-image-ros2-clean/DN-bleeding-foxy-core-l4t-pytorch-r36.4.0?logo=docker&color=blue&link=https%3A%2F%2Fhub.docker.com%2Frepository%2Fdocker%2Fnorlabulaval%2Fdockerized-vaul-base-image-ros2-clean)

  ![Docker Image Version (tag)](https://img.shields.io/docker/v/norlabulaval/dockerized-vaul-base-image-ros2-clean/DN-bleeding-foxy-desktop-l4t-pytorch-r35.4.1?logo=docker&color=blue&link=https%3A%2F%2Fhub.docker.com%2Frepository%2Fdocker%2Fnorlabulaval%2Fdockerized-vaul-base-image-ros2-clean)
  ![Docker Image Version (tag)](https://img.shields.io/docker/v/norlabulaval/dockerized-vaul-base-image-ros2-clean/DN-bleeding-foxy-base-l4t-pytorch-r35.4.1?logo=docker&color=blue&link=https%3A%2F%2Fhub.docker.com%2Frepository%2Fdocker%2Fnorlabulaval%2Fdockerized-vaul-base-image-ros2-clean)
  ![Docker Image Version (tag)](https://img.shields.io/docker/v/norlabulaval/dockerized-vaul-base-image-ros2-clean/DN-bleeding-foxy-core-l4t-pytorch-r35.4.1?logo=docker&color=blue&link=https%3A%2F%2Fhub.docker.com%2Frepository%2Fdocker%2Fnorlabulaval%2Fdockerized-vaul-base-image-ros2-clean)

#### [dockerized-norlab-base-image](https://hub.docker.com/repository/docker/norlabulaval/dockerized-norlab-base-image)   
  ![Docker Image Version (tag)](https://img.shields.io/docker/v/norlabulaval/dockerized-norlab-base-image/DN-bleeding-l4t-pytorch-r36.4.0?logo=docker&color=blue&link=https%3A%2F%2Fhub.docker.com%2Frepository%2Fdocker%2Fnorlabulaval%2Fdockerized-norlab-base-image)
  ![Docker Image Version (tag)](https://img.shields.io/docker/v/norlabulaval/dockerized-norlab-base-image/DN-bleeding-l4t-pytorch-r35.4.1?logo=docker&color=blue&link=https%3A%2F%2Fhub.docker.com%2Frepository%2Fdocker%2Fnorlabulaval%2Fdockerized-norlab-base-image)
  

[//]: # (### Dockerized-VAUL Dependencies)

[//]: # (#### [dockerized-vaul-dependencies-full]&#40;https://hub.docker.com/repository/docker/norlabulaval/dockerized-vaul-dependencies-full&#41; &#40;alias [dockerized-vaul-dependencies-ros2-custom]&#40;https://hub.docker.com/repository/docker/norlabulaval/dockerized-vaul-dependencies-ros2-custom&#41; &#41;    )

[//]: # (  ![Docker Image Version &#40;tag&#41;]&#40;https://img.shields.io/docker/v/norlabulaval/dockerized-vaul-dependencies-full/DN-bleeding-foxy-desktop-l4t-pytorch-r35.4.1?logo=docker&color=blue&link=https%3A%2F%2Fhub.docker.com%2Frepository%2Fdocker%2Fnorlabulaval%2Fdockerized-vaul-dependencies-full&#41;)

[//]: # (  ![Docker Image Version &#40;tag&#41;]&#40;https://img.shields.io/docker/v/norlabulaval/dockerized-vaul-dependencies-full/DN-bleeding-foxy-base-l4t-pytorch-r35.4.1?logo=docker&color=blue&link=https%3A%2F%2Fhub.docker.com%2Frepository%2Fdocker%2Fnorlabulaval%2Fdockerized-vaul-dependencies-full&#41;)

[//]: # (  ![Docker Image Version &#40;tag&#41;]&#40;https://img.shields.io/docker/v/norlabulaval/dockerized-vaul-dependencies-full/DN-bleeding-foxy-core-l4t-pytorch-r35.4.1?logo=docker&color=blue&link=https%3A%2F%2Fhub.docker.com%2Frepository%2Fdocker%2Fnorlabulaval%2Fdockerized-vaul-dependencies-full&#41;)

[//]: # ()
[//]: # (#### [dockerized-vaul-dependencies-python-science-stack]&#40;https://hub.docker.com/repository/docker/norlabulaval/dockerized-vaul-dependencies-python-science-stack&#41;   )

[//]: # (  ![Docker Image Version &#40;tag&#41;]&#40;https://img.shields.io/docker/v/norlabulaval/dockerized-vaul-dependencies-python-science-stack/DN-bleeding-foxy-desktop-l4t-pytorch-r35.4.1?logo=docker&color=blue&link=https%3A%2F%2Fhub.docker.com%2Frepository%2Fdocker%2Fnorlabulaval%2Fdockerized-vaul-dependencies-python-science-stack&#41;)

[//]: # (  ![Docker Image Version &#40;tag&#41;]&#40;https://img.shields.io/docker/v/norlabulaval/dockerized-vaul-dependencies-python-science-stack/DN-bleeding-foxy-base-l4t-pytorch-r35.4.1?logo=docker&color=blue&link=https%3A%2F%2Fhub.docker.com%2Frepository%2Fdocker%2Fnorlabulaval%2Fdockerized-vaul-dependencies-python-science-stack&#41;)

[//]: # (  ![Docker Image Version &#40;tag&#41;]&#40;https://img.shields.io/docker/v/norlabulaval/dockerized-vaul-dependencies-python-science-stack/DN-bleeding-foxy-core-l4t-pytorch-r35.4.1?logo=docker&color=blue&link=https%3A%2F%2Fhub.docker.com%2Frepository%2Fdocker%2Fnorlabulaval%2Fdockerized-vaul-dependencies-python-science-stack&#41;)

[//]: # ()
[//]: # (#### [dockerized-vaul-dependencies-prompt]&#40;https://hub.docker.com/repository/docker/norlabulaval/dockerized-vaul-dependencies-prompt&#41;   )

[//]: # (  ![Docker Image Version &#40;tag&#41;]&#40;https://img.shields.io/docker/v/norlabulaval/dockerized-vaul-dependencies-prompt/DN-bleeding-foxy-desktop-l4t-pytorch-r35.4.1?logo=docker&color=blue&link=https%3A%2F%2Fhub.docker.com%2Frepository%2Fdocker%2Fnorlabulaval%2Fdockerized-vaul-dependencies-prompt&#41;)

[//]: # (  ![Docker Image Version &#40;tag&#41;]&#40;https://img.shields.io/docker/v/norlabulaval/dockerized-vaul-dependencies-prompt/DN-bleeding-foxy-base-l4t-pytorch-r35.4.1?logo=docker&color=blue&link=https%3A%2F%2Fhub.docker.com%2Frepository%2Fdocker%2Fnorlabulaval%2Fdockerized-vaul-dependencies-prompt&#41;)

[//]: # (  ![Docker Image Version &#40;tag&#41;]&#40;https://img.shields.io/docker/v/norlabulaval/dockerized-vaul-dependencies-prompt/DN-bleeding-foxy-core-l4t-pytorch-r35.4.1?logo=docker&color=blue&link=https%3A%2F%2Fhub.docker.com%2Frepository%2Fdocker%2Fnorlabulaval%2Fdockerized-vaul-dependencies-prompt&#41;)

[//]: # ()
[//]: # (#### [dockerized-vaul-dependencies-core]&#40;https://hub.docker.com/repository/docker/norlabulaval/dockerized-vaul-dependencies-core&#41;   )

[//]: # (  ![Docker Image Version &#40;tag&#41;]&#40;https://img.shields.io/docker/v/norlabulaval/dockerized-vaul-dependencies-core/DN-bleeding-foxy-desktop-l4t-pytorch-r35.4.1?logo=docker&color=blue&link=https%3A%2F%2Fhub.docker.com%2Frepository%2Fdocker%2Fnorlabulaval%2Fdockerized-vaul-dependencies-core&#41;)

[//]: # (  ![Docker Image Version &#40;tag&#41;]&#40;https://img.shields.io/docker/v/norlabulaval/dockerized-vaul-dependencies-core/DN-bleeding-foxy-base-l4t-pytorch-r35.4.1?logo=docker&color=blue&link=https%3A%2F%2Fhub.docker.com%2Frepository%2Fdocker%2Fnorlabulaval%2Fdockerized-vaul-dependencies-core&#41;)

[//]: # (  ![Docker Image Version &#40;tag&#41;]&#40;https://img.shields.io/docker/v/norlabulaval/dockerized-vaul-dependencies-core/DN-bleeding-foxy-core-l4t-pytorch-r35.4.1?logo=docker&color=blue&link=https%3A%2F%2Fhub.docker.com%2Frepository%2Fdocker%2Fnorlabulaval%2Fdockerized-vaul-dependencies-core&#41;)


[//]: # (### Dockerized-Norlab Perception)

[//]: # (#### [dockerized-norlab-perception-stack]&#40;https://hub.docker.com/repository/docker/norlabulaval/dockerized-norlab-perception-stack&#41;  )

[//]: # (  ![Docker Image Version &#40;tag&#41;]&#40;https://img.shields.io/docker/v/norlabulaval/dockerized-norlab-perception-stack/DN-bleeding-foxy-desktop-l4t-pytorch-r35.4.1?logo=docker&color=blue&link=https%3A%2F%2Fhub.docker.com%2Frepository%2Fdocker%2Fnorlabulaval%2Fdockerized-norlab-perception-stack&#41;)

[//]: # (  ![Docker Image Version &#40;tag&#41;]&#40;https://img.shields.io/docker/v/norlabulaval/dockerized-norlab-perception-stack/DN-bleeding-foxy-base-l4t-pytorch-r35.4.1?logo=docker&color=blue&link=https%3A%2F%2Fhub.docker.com%2Frepository%2Fdocker%2Fnorlabulaval%2Fdockerized-norlab-perception-stack&#41;)

[//]: # (  ![Docker Image Version &#40;tag&#41;]&#40;https://img.shields.io/docker/v/norlabulaval/dockerized-norlab-perception-stack/DN-bleeding-foxy-core-l4t-pytorch-r35.4.1?logo=docker&color=blue&link=https%3A%2F%2Fhub.docker.com%2Frepository%2Fdocker%2Fnorlabulaval%2Fdockerized-norlab-perception-stack&#41;)

[//]: # ()
[//]: # (#### [dockerized-norlab-perception-libpointmatcher]&#40;https://hub.docker.com/repository/docker/norlabulaval/dockerized-norlab-perception-libpointmatcher&#41;   )

[//]: # (  ![Docker Image Version &#40;tag&#41;]&#40;https://img.shields.io/docker/v/norlabulaval/dockerized-norlab-perception-libpointmatcher/DN-bleeding-foxy-desktop-l4t-pytorch-r35.4.1?logo=docker&color=blue&link=https%3A%2F%2Fhub.docker.com%2Frepository%2Fdocker%2Fnorlabulaval%2Fdockerized-norlab-perception-libpointmatcher&#41;)

[//]: # (  ![Docker Image Version &#40;tag&#41;]&#40;https://img.shields.io/docker/v/norlabulaval/dockerized-norlab-perception-libpointmatcher/DN-bleeding-foxy-base-l4t-pytorch-r35.4.1?logo=docker&color=blue&link=https%3A%2F%2Fhub.docker.com%2Frepository%2Fdocker%2Fnorlabulaval%2Fdockerized-norlab-perception-libpointmatcher&#41;)

[//]: # (  ![Docker Image Version &#40;tag&#41;]&#40;https://img.shields.io/docker/v/norlabulaval/dockerized-norlab-perception-libpointmatcher/DN-bleeding-foxy-core-l4t-pytorch-r35.4.1?logo=docker&color=blue&link=https%3A%2F%2Fhub.docker.com%2Frepository%2Fdocker%2Fnorlabulaval%2Fdockerized-norlab-perception-libpointmatcher&#41;)

[//]: # (#### [dockerized-norlab-perception-icp-mapper]&#40;https://hub.docker.com/repository/docker/norlabulaval/dockerized-norlab-perception-icp-mapper&#41;  )

[//]: # (  ![Docker Image Version &#40;tag&#41;]&#40;https://img.shields.io/docker/v/norlabulaval/dockerized-norlab-perception-icp-mapper/DN-bleeding-foxy-desktop-l4t-pytorch-r35.4.1?logo=docker&color=blue&link=https%3A%2F%2Fhub.docker.com%2Frepository%2Fdocker%2Fnorlabulaval%2Fdockerized-norlab-perception-icp-mapper&#41;)

[//]: # (  ![Docker Image Version &#40;tag&#41;]&#40;https://img.shields.io/docker/v/norlabulaval/dockerized-norlab-perception-icp-mapper/DN-bleeding-foxy-base-l4t-pytorch-r35.4.1?logo=docker&color=blue&link=https%3A%2F%2Fhub.docker.com%2Frepository%2Fdocker%2Fnorlabulaval%2Fdockerized-norlab-perception-icp-mapper&#41;)

[//]: # (  ![Docker Image Version &#40;tag&#41;]&#40;https://img.shields.io/docker/v/norlabulaval/dockerized-norlab-perception-icp-mapper/DN-bleeding-foxy-core-l4t-pytorch-r35.4.1?logo=docker&color=blue&link=https%3A%2F%2Fhub.docker.com%2Frepository%2Fdocker%2Fnorlabulaval%2Fdockerized-norlab-perception-icp-mapper&#41;)


<br>

[//]: # (<details>)

[//]: # (  <summary style="font-weight: bolder;font-size: x-large;"><b> Build matrix summary › [ Services ... ] x [ build tag ... ] </b></summary>)

[//]: # ()
[//]: # ([//]: # &#40;## Build matrix summary › `[ Services ... ] x [ build tag ... ]`&#41;)
[//]: # ()
[//]: # (![]&#40;visual/crawl_1.png&#41;)

[//]: # (![]&#40;visual/crawl_2.png&#41;)

[//]: # ()
[//]: # (</details>)



---

# For developer and maintainer

## Clone repository
```shell
 git clone --recurse-submodule https://github.com/norlab-ulaval/dockerized-norlab.git
```

## Usage example
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


<br>
