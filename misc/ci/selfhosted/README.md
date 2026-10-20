# Ephemeral self-hosted Actions runner

This kit gives `build-firmware.yaml` an ephemeral runner on a self-hosted remote
build host. A local systemd user controller uses the coordinator's existing
`gh` login to mint a one-job JIT configuration, sends it over SSH stdin, and
starts a fresh container. The JIT runner deregisters after one job; the
container, writable layer, checkpoint data, and runner process are then
discarded.

There is no PAT or reusable runner credential. The JIT configuration is not put
in arguments, environment variables, files, or logs outside its required
inside-container `run.sh --jitconfig` argument. GitHub authentication remains in
the current local `gh` profile. If the workstation is off or SSH is unavailable,
the runner is unavailable; the remote build host cannot operate it standalone.

The launcher serializes access with `flock`. The controller also takes a local
lock, retries failed cycles after 10 seconds, and kills its SSH child and asks
the remote launcher to remove the exact container on shutdown. Runner records
left registered by a failed launch are deleted with the same local `gh` login.

## Build the image on the remote build host

Docker must be usable by the remote account without a host service change.
Copy this directory to a build context on the remote build host, then supply
both a digest-pinned Ubuntu 24.04 base and the Actions runner version plus its
published SHA-256.

```bash
UBUNTU_DIGEST=68ee22004a9109b44973313b02ff7b1de250f1de0f5ecb628d73f579aaff33ac
RUNNER_VERSION=2.338.0
RUNNER_SHA256=af4b794c1bc41d73d40535e3fe092a39f9679cd8d965954c2aca25a05ca41d32
docker build \
  --build-arg UBUNTU_DIGEST="$UBUNTU_DIGEST" \
  --build-arg RUNNER_VERSION="$RUNNER_VERSION" \
  --build-arg RUNNER_SHA256="$RUNNER_SHA256" \
  -t "capoworks-actions-runner:$RUNNER_VERSION" .
```

The Dockerfile rejects an unpinned or malformed base digest, a malformed
version/checksum, and a downloaded archive whose `sha256sum` does not match.
The launcher expects tag `capoworks-actions-runner:2.338.0`. The Ubuntu 24.04
base index digest and official runner archive checksum above are the verified
inputs for that tag.

The image is Ubuntu 24.04 with Java 21, the firmware cross-build packages,
GitHub runner native libraries, and `/opt/ci-bin/make`. The wrapper appends
`-j${JOBS:-2}` after caller arguments so legacy `make -j20` is capped. Defaults
are `JOBS=2`, `NUMJOBS=-j2`, `JAVA_TOOL_OPTIONS=-Xmx1024m`, Gradle workers 2,
and no Gradle daemon.

## Install the remote launcher

```bash
USER_HOST=user@ssh-host
REMOTE_PATH=/absolute/path/to/capoworks-actions-runner
ssh "$USER_HOST" "install -d -m 700 '$REMOTE_PATH'"
scp launch.sh "$USER_HOST:$REMOTE_PATH/launch.sh"
ssh "$USER_HOST" "chmod 700 '$REMOTE_PATH/launch.sh'"
```

The launcher creates a container without host mounts, the Docker socket, ports,
host networking, devices, or privileged mode. It uses the default bridge with
2 CPUs, an 8 GiB total RAM-plus-swap limit (4 GiB RAM and 4 GiB swap), and 512
PIDs. The entrypoint is root only long enough to install fail-closed output
rules, then drops to UID/GID 1001 with all bounding, inheritable, permitted,
effective, and ambient capabilities removed and no-new-privs set. Docker adds
only `NET_ADMIN`, `SETUID`, `SETGID`, and `SETPCAP` to the entrypoint; they are
absent during the job. Private IPv4 ranges, CGNAT, link-local, multicast,
documentation, and reserved destinations are rejected; loopback and Docker DNS
are allowed. If IPv6 is enabled, only `::1` is allowed; startup fails closed if
the IPv6 output policy cannot be installed.

Run the built-in post-drop probe:

```bash
docker run --rm --cap-drop ALL \
  --cap-add NET_ADMIN --cap-add SETUID --cap-add SETGID --cap-add SETPCAP \
  --cpus 2 --memory 4g --memory-swap 8g --pids-limit 512 \
  --network bridge \
  capoworks-actions-runner:2.338.0 probe
```

It must report UID/GID 1001, zero inheritable/permitted/effective/bounding/
ambient capabilities, `NoNewPrivs=1`, no job `NET_ADMIN` capability, public
network OK, both the bridge gateway and private network blocked, and `-j2` as
the last job flag in `MAKEFLAGS`. Hardware actions are not part of this runner.

## Install and start the local controller

The coordinator needs an authenticated `gh` profile with repository
administration access and non-interactive SSH access to the remote build host.
Configure the SSH destination and the installed launcher path in the local
environment file. These deployment values must remain outside the repository.

```bash
install -d -m 700 ~/.local/share/capoworks-actions-controller ~/.config/systemd/user
install -m 700 controller.py ~/.local/share/capoworks-actions-controller/controller.py
install -m 600 capoworks-actions-controller.service ~/.config/systemd/user/
install -m 600 /dev/null ~/.config/capoworks-actions-runner.env
```

Set both required variables in `~/.config/capoworks-actions-runner.env`:

```bash
CI_RUNNER_SSH_HOST=user@ssh-host
CI_RUNNER_REMOTE_LAUNCHER=/absolute/path/to/capoworks-actions-runner/launch.sh
```

The SSH destination may be a host alias or `user@host`. The launcher path must
be absolute and contain no whitespace. The controller validates both settings
at startup without logging their values. Do not put credentials in this file.

```bash
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
ssh "$USER_HOST" 'docker ps -aq --filter name=capoworks-jit- | xargs -r docker rm -f'
ssh "$USER_HOST" 'docker image rm capoworks-actions-runner:2.338.0'
rm -f ~/.config/systemd/user/capoworks-actions-controller.service
rm -f ~/.config/capoworks-actions-runner.env
rm -rf ~/.local/share/capoworks-actions-controller
ssh "$USER_HOST" "rm -rf '$REMOTE_PATH'"
systemctl --user daemon-reload
```

For an update, rebuild with the new runner version and its verified hash, update
the launcher image tag if needed, reinstall `launch.sh` and `controller.py`, run
`systemctl --user daemon-reload`, and restart the service. Normal container
completion removes the writable layer automatically; the explicit remote
cleanup exists for interruption and orphan recovery.
