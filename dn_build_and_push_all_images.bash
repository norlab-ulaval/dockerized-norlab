#!/bin/bash

# ....Build all images..............................................................................................

docker compose -f docker-compose.ros2.jetson.build.yaml build --push
