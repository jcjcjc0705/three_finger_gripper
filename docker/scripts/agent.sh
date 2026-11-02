#!/bin/bash
# usage: agent.sh [--dry-run] [up|down|exec|debug|<any docker compose subcommand>]
# Starts the Agent and uros_node on the board. --dry-run: torque stays off.
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/utils.sh"

if [[ "$1" == "--dry-run" ]]; then export UROS_ARGS=--dry-run; shift; fi
main "${1:-up}" "$COMPOSE_DIR/docker-compose-agent.yaml" "${@:2}"
