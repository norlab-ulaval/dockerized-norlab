# Repository Guidelines

_dockerized-norlab_ guidelines and instructions


## Repository Description

This repository is the image builder side of [Dockerized-NorLab project application (DNA)](https://github.com/norlab-ulaval/dockerized-norlab-project).
Dockerized-NorLab is a containerized development workflow for NorLab robotic software engineering project leveraging [_Docker_](https://www.docker.com) and [_NVIDIA Container Toolkit_](https://github.com/NVIDIA/nvidia-container-toolkit).

Refer to the [Features](../README.md#features) section in the README.md for details.

Refer to the [Why](../README.md#why) section in the README.md for details.


## Repository Guidelines Instructions

1. First, review and learn _A2G Framework Guidelines_ specified in
   `.junie/ai_agent_guidelines/guidelines.a2g_framework.md`.
2. Then review the remaining repository guidelines below.
3. **AI agents must follow the mandatory compliance requirements specified below.**


## Prime directive

Always comply with _A2G Framework Guidelines_, _Repository Guidelines_ and _AI operator_ instructions.


## AI Agent Compliance Requirements

All AI agents must:

1. **Always** review A2G guidelines before starting any task
2. **Always** follow A2G file placement decision tree
3. **Always** check workflow mode in `.junie/a2g_config.yml`
4. **Always** apply A2G task verb interpretation protocols

See A2G general guidelines for complete procedures and requirements.


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


#### Test execution scripts:
- `tests/run_bats_core_test_in_n2st.bash`
- `tests/run_all_dryrun_and_tests_scripts.bash` 
- `tests/run_all_docker_dryrun_and_config_tests.bash`


## Repository Terminology

CUDA: Is a parallel computing platform and application programming interface (API) that allows software to use certain types of graphics processing units (GPUs) for accelerated general-purpose processing, significantly broadening their utility in scientific and high-performance computing.
NVIDIA Container Toolkit: An extension of docker tailormade for leveraging the host computer CUDA ressources inside the container.


## Repository Specific Additional Guidelines

Proceed with _AI operator_ instructions
