#!/usr/bin/env python3
"""Create one-job GitHub Actions runners and launch each on Jellyfin."""

import fcntl, json, os, re, secrets, signal, subprocess, time
from pathlib import Path

REPO = "repos/capoworks/capoworks-ecu"
REMOTE = "jellyfin-ts"
REMOTE_LAUNCHER = "/home/deffie/.local/share/capoworks-actions-runner/launch.sh"
SSH = [
    "ssh", "-T", "-o", "BatchMode=yes", "-o", "ConnectTimeout=15",
    "-o", "ServerAliveInterval=30", "-o", "ServerAliveCountMax=3", REMOTE,
]
STATE = Path.home() / ".local/share/capoworks-actions-controller"
STOP = False


def stop(_signum, _frame):
    global STOP
    STOP = True


def api(method, path, fields=()):
    try:
        result = subprocess.run(
            ["gh", "api", "--method", method, path, *fields],
            text=True, capture_output=True, timeout=30, check=False,
        )
    except (OSError, subprocess.TimeoutExpired):
        raise RuntimeError(f"gh api {method} {path} failed; response redacted") from None
    if result.returncode:
        raise RuntimeError(f"gh api {method} {path} failed; response redacted")
    return result.stdout


def remove_orphan(name, runner_id):
    try:
        api("GET", f"{REPO}/actions/runners/{runner_id}")
    except RuntimeError:
        return
    try:
        api("DELETE", f"{REPO}/actions/runners/{runner_id}")
        print(f"removed orphan runner record {name}", flush=True)
    except RuntimeError:
        print(f"orphan cleanup failed for {name}", flush=True)


def run_job():
    token = time.strftime("%Y%m%d%H%M%S", time.gmtime()) + "-" + secrets.token_hex(4)
    name = f"capoworks-jellyfin-{token}"
    fields = (
        "-f", f"name={name}", "-F", "runner_group_id=1",
        "-f", "work_folder=_work", "-f", "labels[]=self-hosted",
        "-f", "labels[]=Linux", "-f", "labels[]=X64",
        "-f", "labels[]=ubuntu-latest-selfhost",
    )
    data = json.loads(api(
        "POST", f"{REPO}/actions/runners/generate-jitconfig", fields
    ))
    encoded = data["encoded_jit_config"]
    runner_id = data["runner"]["id"]

    proc = None
    try:
        if not re.fullmatch(r"[A-Za-z0-9+/]+={0,2}", encoded):
            raise RuntimeError("GitHub returned an invalid JIT configuration")
        if STOP:
            return
        print(f"starting ephemeral runner {name}", flush=True)
        proc = subprocess.Popen(
            [*SSH, REMOTE_LAUNCHER, "launch", token],
            stdin=subprocess.PIPE, text=True, start_new_session=True,
        )
        try:
            proc.stdin.write(encoded + "\n")
            proc.stdin.close()
        except BrokenPipeError:
            pass
        while proc.poll() is None and not STOP:
            time.sleep(0.5)
        if proc.poll() is None:
            os.killpg(proc.pid, signal.SIGTERM)
            try:
                proc.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
                proc.wait()
        if proc.returncode:
            print(f"ephemeral runner exited with status {proc.returncode}", flush=True)
    finally:
        if proc is not None:
            try:
                subprocess.run(
                    [*SSH, REMOTE_LAUNCHER, "cleanup", token],
                    timeout=30, check=False,
                )
            except (OSError, subprocess.TimeoutExpired):
                print(f"remote cleanup did not complete for {name}", flush=True)
        remove_orphan(name, runner_id)


def main():
    os.umask(0o077)
    STATE.mkdir(mode=0o700, parents=True, exist_ok=True)
    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    with (STATE / "controller.lock").open("w") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        while not STOP:
            try:
                run_job()
            except Exception as error:
                print(f"controller cycle failed ({type(error).__name__}); retrying in 10s", flush=True)
            if not STOP:
                time.sleep(10)


if __name__ == "__main__":
    main()
