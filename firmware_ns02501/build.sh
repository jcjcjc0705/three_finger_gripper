#!/bin/bash
# usage: build.sh [make target ...]
#   build.sh           build every program for the board
#   build.sh check     run the offline tests
#   build.sh deploy    stripped copies in deploy/, ready to copy to the board
set -e
cd "$(dirname "${BASH_SOURCE[0]}")"
docker build -q -t ns02501-cross . > /dev/null
exec docker run --rm --user "$(id -u):$(id -g)" -v "$PWD":/firmware -w /firmware \
    ns02501-cross make -j"$(nproc)" "$@"
