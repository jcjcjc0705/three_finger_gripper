#!/bin/bash

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
COMPOSE_DIR="$REPO_ROOT/docker/compose"

# Every compose file here shares one project name so the services show up
# together, which also makes each of them look like an orphan to the others.
export COMPOSE_IGNORE_ORPHANS=True

if docker compose version &> /dev/null; then
    DOCKER_COMPOSE_COMMAND="docker compose"
elif command -v docker-compose &> /dev/null; then
    DOCKER_COMPOSE_COMMAND="docker-compose"
else
    echo "Neither 'docker compose' nor 'docker-compose' is installed."
    exit 1
fi

cleanup() {
    local compose_file="$1"
    echo
    echo "Shutting down docker compose services..."
    echo "Stopping services for $compose_file..."
    # uros_node has to be told to stop, or the motors keep their torque with nothing
    # controlling them. Everything else can go at once.
    if $DOCKER_COMPOSE_COMMAND -f "$compose_file" config --services | grep -qx node; then
        $DOCKER_COMPOSE_COMMAND -f "$compose_file" stop --timeout 20 node
    fi
    $DOCKER_COMPOSE_COMMAND -f "$compose_file" down --timeout 0
    exit 0
}

container_state() {
    docker inspect -f '{{.State.Running}} {{.State.StartedAt}}' "$1" 2>/dev/null
}

# The shell `up` leaves you in. Leaving it stops everything, but it also ends when the
# service is restarted from another terminal, and that must not stop everything.
# Returns 0 if you left it, and 1 if the service stopped and did not come back.
stay_in_service() {
    local compose_file="$1" service="$2" container before
    while true; do
        container=$($DOCKER_COMPOSE_COMMAND -f "$compose_file" ps -q "$service")
        before=$(container_state "$container")
        $DOCKER_COMPOSE_COMMAND -f "$compose_file" exec -it "$service" bash
        sleep 1
        [[ "$(container_state "$container")" == "$before" ]] && return 0

        echo "$service was restarted, reconnecting..."
        local waited=0
        until [[ "$(container_state "$($DOCKER_COMPOSE_COMMAND -f "$compose_file" ps -q "$service")")" == true* ]]; do
            if (( ++waited > 30 )); then
                echo "$service has not come back. Everything else is still running: 'down' stops it."
                return 1
            fi
            sleep 1
        done
    done
}

main() {
    local mode="$1"
    local compose_file="$2"
    shift 2 2>/dev/null

    if [[ ! -f "$compose_file" ]]; then
        echo "Error: Compose file not found: $compose_file" >&2
        exit 1
    fi

    # Which service `up` and `exec` drop you into. Defaults to the first one
    # the file declares, which is what a single-service file wants. A file that
    # only includes others has no order worth relying on, so it can name one.
    local service
    service="${COMPOSE_MAIN_SERVICE:-$($DOCKER_COMPOSE_COMMAND -f "$compose_file" config --services | head -1)}"
    local container_name
    container_name=$(grep -oP 'container_name:\s*\K\S+' "$compose_file" | head -1)
    local debug_name="${container_name:-$service}_debug"

    case "$mode" in
        down)
            docker rm -f "$debug_name" &> /dev/null || true
            cleanup "$compose_file"
            ;;

        exec)
            $DOCKER_COMPOSE_COMMAND -f "$compose_file" exec -it "$service" bash
            ;;

        debug)
            if ! docker ps --format '{{.Names}}' | grep -qx "$debug_name"; then
                $DOCKER_COMPOSE_COMMAND -f "$compose_file" stop "$service"
                $DOCKER_COMPOSE_COMMAND -f "$compose_file" run --rm -d --name "$debug_name" \
                    --entrypoint bash "$service" -c "tail -f /dev/null"
            fi
            docker exec -it "$debug_name" bash
            ;;

        up | "")
            trap 'cleanup "$compose_file"' SIGINT

            echo "Starting services for $compose_file..."
            $DOCKER_COMPOSE_COMMAND -f "$compose_file" up -d

            if stay_in_service "$compose_file" "$service"; then
                cleanup "$compose_file"
            fi
            ;;

        *)
            $DOCKER_COMPOSE_COMMAND -f "$compose_file" "$mode" "$@"
            ;;
    esac
}
