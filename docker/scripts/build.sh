#!/bin/bash
# usage: build.sh
# Builds the image and the ROS 2 workspace. Run once, and again after changing
# anything under src/ that is not a Python file or a launch file.
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/utils.sh"
set -e

$DOCKER_COMPOSE_COMMAND -f "$COMPOSE_DIR/docker-compose-start-all.yaml" build
$DOCKER_COMPOSE_COMMAND -f "$COMPOSE_DIR/docker-compose-gripper.yaml" run --rm --no-deps gripper \
    colcon build --symlink-install
