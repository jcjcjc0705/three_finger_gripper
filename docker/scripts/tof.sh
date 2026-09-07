#!/bin/bash
# usage: tof.sh [up|down|exec|debug|<any docker compose subcommand>]
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/utils.sh"
main "${1:-up}" "$COMPOSE_DIR/docker-compose-tof.yaml" "${@:2}"
