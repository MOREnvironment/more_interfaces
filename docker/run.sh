#!/usr/bin/env bash
# Build the image and run the build and the tests of the interface package inside it, on the host user's uid.
#   docker/run.sh build        build the image
#   docker/run.sh colcon       colcon build of the package into the cache
#   docker/run.sh test         colcon build, then the build and the tests of the interfaces
#   docker/run.sh shell        a shell in the environment
#   docker/run.sh run <cmd>    one command in the environment with the built workspace sourced
# CACHE_DIR (default: ~/.cache/more_interfaces_build) holds build/, install/ and log/.
set -euo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo="$(dirname "$here")"
image="more_interfaces:lyrical"
cache="${CACHE_DIR:-$HOME/.cache/more_interfaces_build}"
mkdir -p "$cache"

run() {
    docker run --rm --user "$(id -u):$(id -g)" -e HOME=/tmp -v "$repo":/ws/src/more_interfaces:ro -v "$cache":/cache -w /cache \
        "$@"
}
build_ws() {
    docker image inspect "$image" >/dev/null 2>&1 || docker build -t "$image" "$here"
    run "$image" colcon --log-base /cache/log build --base-paths /ws/src --build-base /cache/build --install-base /cache/install \
        --cmake-args -DBUILD_TESTING=ON
}

case "${1:-test}" in
    build) docker build -t "$image" "$here" ;;
    colcon) build_ws ;;
    test)
        build_ws
        run "$image" bash -c 'rm -rf /cache/build/*/test_results && colcon --log-base /cache/log test --base-paths /ws/src --build-base /cache/build --install-base /cache/install \
            --event-handlers console_direct+ && colcon test-result --test-result-base /cache/build --verbose' ;;
    shell) run -it "$image" bash ;;
    run) shift; run "$image" "$@" ;;
    *) echo "usage: $0 build|colcon|test|shell|run <cmd>" >&2; exit 2 ;;
esac
