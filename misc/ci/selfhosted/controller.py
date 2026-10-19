#!/usr/bin/env python3
"""Create one-job GitHub Actions runners and launch each on a remote build host."""

import fcntl, json, os, re, secrets, signal, subprocess, time
from pathlib import Path

REPO = "repos/capoworks/capoworks-ecu"
SSH_HOST_VARIABLE = "CI_RUNNER_SSH_HOST"
REMOTE_LAUNCHER_VARIABLE = "CI_RUNNER_REMOTE_LAUNCHER"
SSH_DESTINATION = re.compile(r"[A-Za-z0-9][A-Za-z0-9._@:+-]*")
REMOTE_PATH = re.compile(r"/[A-Za-z0-9._+/-]+")
STATE = Path.home() / ".local/share/capoworks-actions-controller"
STOP = False


def stop(_signum, _frame):
    global STOP
    STOP = True


def runner_config():
    remote = os.environ.get(SSH_HOST_VARIABLE, "")
    launcher = os.environ.get(REMOTE_LAUNCHER_VARIABLE, "")
    if not remote:
        raise RuntimeError(f"{SSH_HOST_VARIABLE} is required")
    if not SSH_DESTINATION.fullmatch(remote):
        raise RuntimeError(f"{SSH_HOST_VARIABLE} is invalid")
    if not launcher:
        raise RuntimeError(f"{REMOTE_LAUNCHER_VARIABLE} is required")
    if not REMOTE_PATH.fullmatch(launcher):
        raise RuntimeError(f"{REMOTE_LAUNCHER_VARIABLE} must be an absolute path")
    return remote, launcher


def api(method, path, fields=()):
    try:
        result = subprocess.run(
            ["gh", "api", "--method", method, path, *fields],
            text=True, capture_output=True, timeout=20, check=False,
        )
    except (OSError, subprocess.TimeoutExpired):
        raise RuntimeError(f"gh api {method} {path} failed; response redacted") from None
    if result.returncode:
        raise RuntimeError(f"gh api {method} {path} failed; response redacted")
    return result.stdout


def remove_orphan(name, runner_id=None):
    try:
        data = json.loads(api("GET", f"{REPO}/actions/runners?per_page=100"))
        matches = [
            runner for runner in data.get("runners", [])
            if runner.get("name") == name
        ]
        if not matches:
            return
        if len(matches) != 1:
            raise RuntimeError("multiple runner records matched the unique name")
        matched_id = matches[0].get("id")
        if (
            not isinstance(matched_id, int)
            or isinstance(matched_id, bool)
            or matched_id <= 0
        ):
            raise RuntimeError("runner response did not contain a valid ID")
        if runner_id is not None and runner_id != matched_id:
            raise RuntimeError("runner ID did not match the unique name")
        api("DELETE", f"{REPO}/actions/runners/{matched_id}")
        print(f"removed orphan runner record {name}", flush=True)
    except (RuntimeError, json.JSONDecodeError, AttributeError, TypeError):
        print(f"orphan cleanup failed for {name}", flush=True)


def run_job(remote, remote_launcher):
    token = time.strftime("%Y%m%d%H%M%S", time.gmtime()) + "-" + secrets.token_hex(4)
    name = f"capoworks-ci-{token}"
    runner_id = None
    proc = None
    ssh = [
        "ssh", "-T", "-o", "BatchMode=yes", "-o", "ConnectTimeout=15",
        "-o", "ServerAliveInterval=30", "-o", "ServerAliveCountMax=3", remote,
    ]
    try:
        if STOP:
            return
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
        received_id = data["runner"]["id"]
        if (
            not isinstance(received_id, int)
            or isinstance(received_id, bool)
            or received_id <= 0
        ):
            raise RuntimeError("GitHub returned an invalid runner ID")
        runner_id = received_id
        if (
            not re.fullmatch(r"[A-Za-z0-9+/]+={0,2}", encoded)
            or len(encoded) > 262144
        ):
            raise RuntimeError("GitHub returned an invalid JIT configuration")
        if STOP:
            return
        print(f"starting ephemeral runner {name}", flush=True)
        proc = subprocess.Popen(
            [*ssh, remote_launcher, "launch", token],
            stdin=subprocess.PIPE, stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL, text=True, start_new_session=True,
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
                    [*ssh, remote_launcher, "cleanup", token],
                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                    timeout=30, check=False,
                )
            except (OSError, subprocess.TimeoutExpired):
                print(f"remote cleanup did not complete for {name}", flush=True)
        remove_orphan(name, runner_id)


def main():
    os.umask(0o077)
    try:
        remote, remote_launcher = runner_config()
    except RuntimeError as error:
        print(f"controller configuration error: {error}", flush=True)
        return 2
    STATE.mkdir(mode=0o700, parents=True, exist_ok=True)
    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    with (STATE / "controller.lock").open("w") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        while not STOP:
            try:
                run_job(remote, remote_launcher)
            except Exception as error:
                print(f"controller cycle failed ({type(error).__name__}); retrying in 10s", flush=True)
            if not STOP:
                time.sleep(10)


if __name__ == "__main__":
    main()
