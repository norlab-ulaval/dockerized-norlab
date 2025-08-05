# Repository Guidelines

dockerized-norlab guidelines and instructions

## Repository Description

Containerized development workflow for NorLab robotic software engineering project
leveraging docker and NVIDIA Container Toolkit technology.

### Features
- CUDA support via _NVIDIA Container Toolkit_
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
 
Recommanded reading by [Tobit Flatscher](https://github.com/2b-t)
- [Why should I use Docker when developing robotics software?](https://github.com/2b-t/docker-for-robotics/blob/main/doc/Motivation.md#2-why-should-i-use-docker-when-developing-robotics-software)
- [Why is Docker important in particular for academic and research institutions?](https://github.com/2b-t/docker-for-robotics/blob/main/doc/Motivation.md#3-why-is-docker-important-in-particular-for-academic-and-research-institutions)



## Prime directive:

Always comply with guidelines and instructions.

## Repository Guidelines Instructions

- First, review _A2G Framework Guidelines_ specified in
  `.junie/ai_agent_guidelines/guidelines.a2g_framework.md` for additional guidelines.
- Then proceed with the remaining repository guidelines instructions.

## Repository Organization

- `.junie/` contains AI agent related files.
- `.junie/ai_agent_guidelines` contains _AI Agent Guidelines (A2G)_ with entrypoint at `.junie/ai_agent_guidelines/README.md`.
- `.env.build_matrix.main*` contain build matrix configuration related environment variables
- `build_matrix_config/` contain mode based build matrix configuration dotenv files. Files are organized by mode: `dev/` -> _development_, `prod/` -> _production_ and `test/` -> _testing_ 
- `dockerized-norlab-images/` contain Dockerfiles, docker compose yaml files and any ressources either available inside image at build time or available inside container at runtime
- `dockerized-norlab-scripts/` contain build system scripts, installer and other highlevel utilities 
- `tests/` contain tests files
- `tests/tests_bats/` contain N2ST bats framework files that are mainly used for unit-testing
- `tests/tests_docker_dryrun_and_config/` contain integration test (see details below)
- `utilities/` contain external libraries such as N2ST and NBS
- `utilities/tmp/dockerized-norlab-project-mock` is use for cloning a fresh copy of a mock "super project" from https://github.com/norlab-ulaval/dockerized-norlab-project-mock.git on test execution.
  `dockerized-norlab-project-mock` is a mock of how a user would install and uses DNP. We refer to this as a "super project" or the "user side".

## Repository Terminology

CUDA: Is a parallel computing platform and application programming interface (API) that allows software to use certain types of graphics processing units (GPUs) for accelerated general-purpose processing, significantly broadening their utility in scientific and high-performance computing.
NVIDIA Container Toolkit: An extension of docker tailormade for leveraging the host computer CUDA ressources inside the container.
