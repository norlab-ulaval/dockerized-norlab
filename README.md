[//]: # ( ==== Logo ================================================== )
<div align="center">
<br>
<br>
<a href="https://norlab.ulaval.ca">
<img src="visual/norlab_logo_acronym_dark.png" width="200">
</a>
<br>

[//]: # ( ==== Title ================================================= )
# _Dockerized-NorLab_

</div>

[//]: # ( ==== Hyperlink ============================================= )
<div align="center">
<p>
<sup>
<a href="https://http://132.203.26.125:8111">NorLab TeamCity GUI</a>
(VPN/intranet access) &nbsp; • &nbsp;  
<a href="https://hub.docker.com/repositories/norlabulaval">norlabulaval</a>
(Docker Hub) &nbsp;
<a href="https://github.com/dusty-nv/jetson-containers">Jetson-Containers</a> 
&nbsp; • &nbsp;
</sup>
</p>  

[//]: # ( ==== Description =========================================== )

**Containerized development workflow for NorLab perception or control projects
<br>
leveraging [_docker_](https://www.docker.com) and [_nvidia-docker_](https://github.com/NVIDIA/nvidia-docker)
technology..**
<br>
<br>

[//]: # ( ==== Badges ================================================ )

[![semantic-release: conventional commits](https://img.shields.io/badge/semantic--release-conventional_commits-453032?logo=semantic-release)](https://github.com/semantic-release/semantic-release)
<img alt="GitHub release (with filter)" src="https://img.shields.io/github/v/release/norlab-ulaval/dockerized-norlab">

<img src="https://img.shields.io/static/v1?label=JetBrains TeamCity&message=CI/CD&color=green?style=plastic&logo=teamcity" />

<br>
<br>

[//]: # ( ==== Maintainer ============================================ )

Maintainer: [Luc Coupal](https://redleader962.github.io)

</div>
<br>

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
 

