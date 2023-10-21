### `dn_container_env_variable` directory
This directory is used to expose environment variable fetched from running container by executing
the [`dn_expose_container_env_variables.bash`](https://github.com/norlab-ulaval/dockerized-norlab/blob/6a086487af5374a9f49337e729d101ce17d2cca8/dockerized-norlab-images/core-images/dn-project/project-develop/dn_expose_container_env_variables.bash) script (from inside the container).
Those environment variables can be used in many ways e.g.: In IDE such as JetBrains PyCharm inside 
run configuration via the plugin [EnvFile](https://github.com/Ashald/EnvFile).
