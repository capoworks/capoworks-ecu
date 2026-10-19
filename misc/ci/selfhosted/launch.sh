#!/usr/bin/env bash
set -euo pipefail
umask 077

readonly image=capoworks-actions-runner:2.329.0
readonly state="${HOME:?}/.local/share/capoworks-actions-runner"
readonly mode=${1:-}
readonly token=${2:-}

if [[ ! "$token" =~ ^[0-9]{14}-[0-9a-f]{8}$ ]]; then
    echo "invalid runner token" >&2
    exit 2
fi
case "$mode" in
    launch|cleanup) ;;
    *) echo "usage: $0 launch|cleanup TOKEN" >&2; exit 2 ;;
esac

mkdir -p "$state"
chmod 0700 "$state"
exec 9>"$state/launch.lock"
flock 9

container="capoworks-jit-$token"
cleanup() {
    local status=$?
    docker rm -f "$container" >/dev/null 2>&1 || true
    exit "$status"
}
trap cleanup EXIT
trap 'exit 129' HUP
trap 'exit 130' INT
trap 'exit 143' TERM

if [[ "$mode" == cleanup ]]; then
    docker rm -f "$container" >/dev/null 2>&1 || true
    exit 0
fi

IFS= read -r encoded_jit
if IFS= read -r extra; then
    echo "expected exactly one base64 line on stdin" >&2
    exit 2
fi
if [[ ! "$encoded_jit" =~ ^[A-Za-z0-9+/]+={0,2}$ ]] || (( ${#encoded_jit} > 262144 )); then
    echo "invalid JIT input" >&2
    exit 2
fi

docker create --rm --interactive --name "$container" \
    --cap-drop ALL \
    --cap-add NET_ADMIN \
    --cap-add SETUID \
    --cap-add SETGID \
    --cap-add SETPCAP \
    --cpus 2 \
    --memory 4g \
    --memory-swap 8g \
    --pids-limit 512 \
    --network bridge \
    "$image" runner >/dev/null

printf '%s\n' "$encoded_jit" | docker start --attach --interactive "$container"
