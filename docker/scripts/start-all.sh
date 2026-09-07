#!/bin/bash
# usage: start-all.sh [up|down|exec|debug|<any docker compose subcommand>]
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/utils.sh"

# include: gives no useful service order, so say which one to land in.
export COMPOSE_MAIN_SERVICE=gripper
main "${1:-up}" "$COMPOSE_DIR/docker-compose-start-all.yaml" "${@:2}"
