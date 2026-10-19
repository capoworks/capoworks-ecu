# Ephemeral self-hosted Actions runner

This kit gives `build-firmware.yaml` an ephemeral runner on the Jellyfin Docker
host. A local systemd user controller uses the coordinator's existing `gh`
login to mint a one-job JIT configuration, sends it over SSH stdin, and starts a
fresh container. The JIT runner deregisters after one job; the container,
writable layer, checkpoint data, and runner process are then discarded.

There is no PAT or reusable runner credential. The JIT configuration is not put
in arguments, environment variables, files, or logs outside its required
inside-container `run.sh --jitconfig` argument. GitHub authentication remains in
the current local `gh` profile. If the workstation is off or SSH is unavailable,
the runner is unavailable; Jellyfin cannot operate it standalone.

The launcher serializes access with `flock`. The controller also takes a local
lock, retries failed cycles after 10 seconds, and kills its SSH child and asks
the remote launcher to remove the exact container on shutdown. Runner records
left registered by a failed launch are deleted with the same local `gh` login.

## Build the image on Jellyfin

Docker must be usable by `deffie` without a host service change. Copy this
directory to a build context on Jellyfin, then supply both a digest-pinned
Ubuntu 24.04 base and the Actions runner version plus its published SHA-256.

```bash
UBUNTU_DIGEST=<ubuntu-24.04-index-sha256>
RUNNER_VERSION=<runner-version>
RUNNER_SHA256=<actions-runner-linux-x64-archive-sha256>
docker build \
  --build-arg UBUNTU_DIGEST="$UBUNTU_DIGEST" \
  --build-arg RUNNER_VERSION="$RUNNER_VERSION" \
  --build-arg RUNNER_SHA256="$RUNNER_SHA256" \
  -t "capoworks-actions-runner:$RUNNER_VERSION" .
```

The Dockerfile rejects an unpinned or malformed base digest, a malformed
version/checksum, and a downloaded archive whose `sha256sum` does not match.
The launcher currently expects tag `capoworks-actions-runner:2.329.0`; either
build that exact version or update the `image=` line in `launch.sh` together
with the build tag.

The image is Ubuntu 24.04 with Java 21, the firmware cross-build packages,
GitHub runner native libraries, and `/opt/ci-bin/make`. The wrapper appends
`-j${JOBS:-2}` after caller arguments so legacy `make -j20` is capped. Defaults
are `JOBS=2`, `NUMJOBS=-j2`, `JAVA_TOOL_OPTIONS=-Xmx1024m`, Gradle workers 2,
and no Gradle daemon.

## Install the remote launcher

```bash
ssh jellyfin-ts 'install -d -m 700 ~/.local/share/capoworks-actions-runner'
scp launch.sh jellyfin-ts:/home/deffie/.local/share/capoworks-actions-runner/launch.sh
ssh jellyfin-ts 'chmod 700 ~/.local/share/capoworks-actions-runner/launch.sh'
```

The launcher creates a container without host mounts, the Docker socket, ports,
host networking, devices, or privileged mode. It uses the default bridge with
2 CPUs, 4 GiB RAM, 8 GiB RAM plus swap, and 512 PIDs. The entrypoint is root only
long enough to install fail-closed output rules, then drops to UID/GID 1001 with
all bounding, inheritable, and ambient capabilities removed and no-new-privs
set. Docker adds only `NET_ADMIN`, `SETUID`, `SETGID`, and `SETPCAP` to the
entrypoint; they are absent during the job. Private IPv4 ranges, CGNAT,
link-local, multicast, documentation, and reserved destinations are rejected;
loopback and Docker DNS are allowed. If IPv6 is enabled, only `::1` is allowed.

Run the built-in post-drop probe:

```bash
docker run --rm --cap-drop ALL \
  --cap-add NET_ADMIN --cap-add SETUID --cap-add SETGID --cap-add SETPCAP \
  --cpus 2 --memory 4g --memory-swap 8g --pids-limit 512 \
  --network bridge \
  capoworks-actions-runner:2.329.0 probe
```

It must report UID/GID 1001, zero effective capabilities, public network OK,
private network blocked, and the capped make wrapper. Hardware actions are not
part of this runner.

## Install and start the local controller

The coordinator needs an authenticated `gh` profile with repository
administration access and a non-interactive SSH alias named `jellyfin-ts`.

```bash
install -d -m 700 ~/.local/share/capoworks-actions-controller ~/.config/systemd/user
install -m 700 controller.py ~/.local/share/capoworks-actions-controller/controller.py
install -m 600 capoworks-actions-controller.service ~/.config/systemd/user/
systemctl --user daemon-reload
systemctl --user enable --now capoworks-actions-controller.service
loginctl enable-linger "$USER"  # may require coordinator/polkit setup
```

Inspect sanitized status with `systemctl --user status
capoworks-actions-controller.service` and `journalctl --user -u
capoworks-actions-controller.service`. Do not enable shell tracing.

## Stop, remove, and update

```bash
systemctl --user disable --now capoworks-actions-controller.service
ssh jellyfin-ts 'docker ps -aq --filter name=capoworks-jit- | xargs -r docker rm -f'
ssh jellyfin-ts 'docker image rm capoworks-actions-runner:2.329.0'
rm -f ~/.config/systemd/user/capoworks-actions-controller.service
rm -rf ~/.local/share/capoworks-actions-controller
ssh jellyfin-ts 'rm -rf ~/.local/share/capoworks-actions-runner'
systemctl --user daemon-reload
```

For an update, rebuild with the new runner version and its verified hash, update
the launcher image tag if needed, reinstall `launch.sh` and `controller.py`, run
`systemctl --user daemon-reload`, and restart the service. Normal container
completion removes the writable layer automatically; the explicit remote
cleanup exists for interruption and orphan recovery.
