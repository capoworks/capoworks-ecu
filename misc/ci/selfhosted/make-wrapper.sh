#!/bin/sh
set -eu

jobs=${JOBS:-2}
case "$jobs" in
    ''|*[!0-9]*) jobs=2 ;;
esac
[ "$jobs" -gt 0 ] || jobs=2

# GNU make uses the last -j option, capping legacy -j20 callers.
exec /usr/bin/make "$@" "-j$jobs"
