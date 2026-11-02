#!/bin/bash
# usage: start-all.sh [--dry-run] [up|down|exec|debug|<any docker compose subcommand>]
# --dry-run: uros_node on the board keeps the torque off.
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/utils.sh"

if [[ "$1" == "--dry-run" ]]; then export UROS_ARGS=--dry-run; shift; fi

# include: gives no useful service order, so say which one to land in.
export COMPOSE_MAIN_SERVICE=gripper
main "${1:-up}" "$COMPOSE_DIR/docker-compose-start-all.yaml" "${@:2}"
