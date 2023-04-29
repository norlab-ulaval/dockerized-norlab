#!/bin/bash

# ....Build all images..............................................................................................

docker compose -f docker-compose.ros2.jetson.build.yaml build
#docker compose -f /home/snow/dockerized-norlab/docker-compose.ros2.jetson.build.yaml build --no-cache
