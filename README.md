<div align="center">
<br>
<br>
<a href="https://norlab.ulaval.ca">
<img src="visual/norlab_logo_acronym_dark.png" width="200">
</a>
<br>
<br>

# _Dockerized-NorLab_

</div>


[//]: # (<b>Project related link: </b> &nbsp; )
<div align="center">
<p>
<sup>
<a href="https://hub.docker.com/repositories/norlabulaval">norlabulaval</a> (Docker Hub) 
&nbsp; • &nbsp;  
<a href="https://github.com/dusty-nv/jetson-containers">Jetson-Containers</a> 
&nbsp; • &nbsp;  
<a href="https://norlab.youtrack.cloud/agiles/121-26/current?query=aggregate%20Subtask%20of:%20NMO-136,%20NMO-316,%20NMO-317,%20NMO-322,%20NMO-324,%20NMO-321">agile board</a> (YouTrack)
</sup>
</p>
</div>

Maintainer: [Luc Coupal](https://redleader962.github.io) 

### What
Containerized development workflow for perception3D, control or robot low level project leveraging [_nvidia-docker_](https://github.com/NVIDIA/nvidia-docker) technology.

#### Features
- Multi ros distros: `foxy`, `humble` ...
- Multi ros packages: `core`, `base`, `desktop`
- Multi packages: `libpoitmatcher`, `pytorch`, `torchvision`, `numba`, `stable-baseline`, `f1tenth-gym` ...
- Multi OS versions:
  - Jetpack: `r35.2.1`, `r32.7.1` ...
  - Ubuntu: `focal` (in progress: `bionic` and `jammy`)
- Multi architectures: `l4t/arm64` (in progress: `linux/arm64`, `x86`)
 
### Why
1. custom dependency management; 
2. development environment consistency; 
3. codebase stabilization trough continuous integration pipeline;
4. easy deployment to robots compute box;
5. results reproducibility.
 

