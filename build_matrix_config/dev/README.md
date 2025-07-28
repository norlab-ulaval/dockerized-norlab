
# Development build matrix configuration 
 
The following dotenv files are used in conjuction with `dn_build_all.bash` 
- `.env.build_matrix.main.dev_subset`

The following dotenv files are used in conjuction with `dn_build_over_single_build_matrix.bash` 
- `.env.build_matrix.dev`
- `.env.build_matrix.main.dev`

## Usage examples

### DN build over main build matrix (i.e., build all)

#### To build
```markdown
export NBS_OVERRIDE_BUILD_MATRIX_MAIN=build_matrix_config/dev/.env.build_matrix.main.dev_subset && \
source dockerized-norlab-scripts/build_script/dn_build_all.bash --fail-fast
```

#### To build and push
```markdown
export NBS_OVERRIDE_BUILD_MATRIX_MAIN=build_matrix_config/dev/.env.build_matrix.main.dev_subset && \
export NBS_OVERRIDE_ADD_DOCKER_CMD_AND_FLAG="build --push" && \
source dockerized-norlab-scripts/build_script/dn_build_all.bash --fail-fast
```

### DN build over single build matrix

#### To build
```markdown
bash dockerized-norlab-scripts/build_script/dn_build_over_single_build_matrix.bash \
    build_matrix_config/dev/.env.build_matrix.main.dev
```

#### To build and push
```markdown
export NBS_OVERRIDE_ADD_DOCKER_CMD_AND_FLAG="build --push" && \
bash dockerized-norlab-scripts/build_script/dn_build_over_single_build_matrix.bash \
    build_matrix_config/dev/.env.build_matrix.main.dev
```

### Execution options

#### Via run configuration + shell script config

- Run config › _[DN] development ( build over single build matrix .env.build_matrix.dev )_ → _run › run_config_buidl_over_single_build_matrix.bash (dev)_
- Configuration file: `run_config_buidl_over_single_build_matrix.bash`

#### Via run configuration

Check the following run configurations
- _[DN] development ( build .env.build_matrix.main.dev_subset )_
- _[DN] development ( build .env.build_matrix.main.dev )_

