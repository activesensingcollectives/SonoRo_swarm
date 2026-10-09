#!/usr/bin/env python3
"""Manage a fleet of Raspberry Pi robots from one workstation.

The tool is designed for the shared Wi‑Fi network used by the SonoRo robots.
It expects a common IPv4 prefix (for example 192.168.1.) and lets you execute
one command, copy the repo, and synchronize the clock on every Pi at the same
moment.
"""

from __future__ import annotations

import argparse
import concurrent.futures
import os
import shlex
import shutil
import socket
import subprocess
import sys
import time
import threading
from typing import List, Sequence

DEFAULT_USERNAME = "thymio"
DEFAULT_REMOTE_DIR = "/home/thymio/SonoRo_swarm"
DEFAULT_STOP_COMMAND = (
    "pids=$(pgrep -f 'sonoro_swarm|SonoRo_swarm|AudioProcessor|RobotMove' || true); "
    'if [ -n "$pids" ]; then '
    "kill $pids 2>/dev/null || true; "
    "sleep 1; "
    "remaining=$(pgrep -f 'sonoro_swarm|SonoRo_swarm|AudioProcessor|RobotMove' || true); "
    'if [ -n "$remaining" ]; then kill -9 $remaining 2>/dev/null || true; fi; '
    'echo "SonoRo robots stopped (pids: $pids)"; '
    "else echo 'No matching robot processes found'; fi"
)
RESPAINKER_RULE = (
    'SUBSYSTEM=="usb", ATTRS{idVendor}=="2886", ATTRS{idProduct}=="0018", MODE="0666"'
)


def normalize_ip_prefix(base_ip: str) -> str:
    """Return a valid IPv4 prefix with an ending dot."""
    cleaned = base_ip.strip().strip(" \t\n")
    if not cleaned:
        raise ValueError("Base IP must not be empty.")
    if cleaned.endswith("."):
        return cleaned
    if cleaned.count(".") == 3:
        return cleaned.rsplit(".", 1)[0] + "."
    if cleaned.count(".") == 2:
        return cleaned + "."
    raise ValueError(
        "Base IP must be in the form 192.168.1. or 192.168.1, not " f"'{base_ip}'."
    )


def detect_local_ip_prefix() -> str | None:
    """Derive the current LAN prefix from the workstation's active IPv4 address."""
    try:
        result = subprocess.run(
            ["ip", "-o", "-4", "addr", "show", "scope", "global"],
            capture_output=True,
            text=True,
            check=False,
        )
        for line in result.stdout.splitlines():
            parts = line.split()
            for index, token in enumerate(parts):
                if token == "inet" and index + 1 < len(parts):
                    ip = parts[index + 1].split("/")[0]
                    if ip and ip != "127.0.0.1":
                        return ip.rsplit(".", 1)[0] + "."
    except FileNotFoundError:
        pass

    try:
        host_ip = socket.gethostbyname(socket.gethostname())
        if host_ip and host_ip != "127.0.0.1":
            return host_ip.rsplit(".", 1)[0] + "."
    except OSError:
        pass
    return None


def parse_ips_from_file(path: str) -> List[str]:
    with open(path, "r", encoding="utf-8") as handle:
        content = handle.read()

    hosts: List[str] = []
    for chunk in content.replace("\r", "\n").split("\n"):
        for value in chunk.split(","):
            item = value.strip()
            if item:
                hosts.append(item)
    return hosts


def parse_hosts(args: argparse.Namespace) -> List[str]:
    if args.ips_file:
        return parse_ips_from_file(args.ips_file)
    if args.ip_list:
        return [host.strip() for host in args.ip_list.split(",") if host.strip()]
    if not args.base_ip:
        detected_prefix = detect_local_ip_prefix()
        if detected_prefix is None:
            raise ValueError("Provide either --base-ip, --ip-list, or --ips-file.")
        args.base_ip = detected_prefix

    prefix = normalize_ip_prefix(args.base_ip)
    if args.start > args.end:
        raise ValueError("--start must be less than or equal to --end.")
    return [f"{prefix}{address}" for address in range(args.start, args.end + 1)]


def generate_ssh_key(key_file: str) -> str:
    """Create a fleet SSH key if it does not exist and return the public key path."""
    expanded = os.path.expanduser(key_file)
    key_dir = os.path.dirname(expanded)
    if key_dir:
        os.makedirs(key_dir, exist_ok=True)

    if not os.path.exists(expanded):
        subprocess.run(
            ["ssh-keygen", "-t", "ed25519", "-N", "", "-f", expanded],
            check=True,
            capture_output=True,
            text=True,
        )

    pub_key_path = f"{expanded}.pub"
    if not os.path.exists(pub_key_path):
        raise FileNotFoundError(f"Public key was not created at {pub_key_path}.")
    return pub_key_path


def setup_ssh_for_host(
    host: str, user: str, public_key_path: str
) -> tuple[str, int, str, str]:
    """Install the SSH public key on a single host."""
    with open(public_key_path, "r", encoding="utf-8") as key_file:
        public_key = key_file.read().strip()
    remote_script = (
        "mkdir -p ~/.ssh && chmod 700 ~/.ssh && "
        'grep -F -q -- "{public_key}" ~/.ssh/authorized_keys 2>/dev/null || '
        "printf '%s\\n' \"{public_key}\" >> ~/.ssh/authorized_keys && "
        "chmod 700 ~/.ssh && chmod 600 ~/.ssh/authorized_keys"
    ).format(public_key=public_key)

    cmd = [
        "ssh",
        "-o",
        "StrictHostKeyChecking=no",
        "-o",
        "UserKnownHostsFile=/dev/null",
        "-o",
        "LogLevel=ERROR",
        "-o",
        "ConnectTimeout=10",
        f"{user}@{host}",
        f"bash -lc {shlex.quote(remote_script)}",
    ]
    completed = subprocess.run(cmd, capture_output=True, text=True, timeout=180)
    return (
        host,
        completed.returncode,
        completed.stdout.strip(),
        completed.stderr.strip(),
    )


def setup_ssh_fleet(
    hosts: Sequence[str], user: str, key_file: str, workers: int
) -> None:
    public_key_path = generate_ssh_key(key_file)
    with concurrent.futures.ThreadPoolExecutor(max_workers=max(1, workers)) as executor:
        futures = [
            executor.submit(setup_ssh_for_host, host, user, public_key_path)
            for host in hosts
        ]
        for future in concurrent.futures.as_completed(futures):
            host, status, stdout, stderr = future.result()
            print(
                f"[{host}] ssh setup -> {'OK' if status == 0 else f'ERROR ({status})'}"
            )
            if stdout:
                print(stdout)
            if stderr:
                print(stderr)


def setup_sudo_for_host(
    host: str, user: str, key_file: str | None = None
) -> tuple[str, int, str, str]:
    """Configure passwordless sudo on a host by creating /etc/sudoers.d/010_nopasswd."""
    sudo_script = (
        'echo "$USER ALL=(ALL) NOPASSWD: ALL" | sudo tee /etc/sudoers.d/010_nopasswd >/dev/null && '
        "sudo chmod 0440 /etc/sudoers.d/010_nopasswd && "
        'echo "Passwordless sudo configured successfully."'
    )
    cmd = [
        "ssh",
        "-t",
        "-o",
        "StrictHostKeyChecking=no",
        "-o",
        "UserKnownHostsFile=/dev/null",
        "-o",
        "LogLevel=ERROR",
        "-o",
        "ConnectTimeout=10",
    ]
    resolved_key = resolve_key_file(key_file)
    if resolved_key:
        cmd.extend(["-i", resolved_key])
    cmd.extend([f"{user}@{host}", f"bash -lc {shlex.quote(sudo_script)}"])

    print(
        f"\n--- Setting up passwordless sudo on {host} (enter sudo password if prompted) ---"
    )
    ret = subprocess.run(cmd)
    return (host, ret.returncode, "", "")


def setup_sudo_fleet(
    hosts: Sequence[str], user: str, key_file: str | None = None
) -> None:
    for host in hosts:
        setup_sudo_for_host(host, user, key_file)


def scan_host_keys_fleet(hosts: Sequence[str]) -> None:
    """Fetch host keys with ssh-keyscan and append only new entries to ~/.ssh/known_hosts.

    This is idempotent and safe to run multiple times. It deduplicates entries.
    """
    ssh_keyscan = shutil.which("ssh-keyscan")
    if not ssh_keyscan:
        print(
            "ssh-keyscan not found on this machine. Install OpenSSH client to use --scan-host-keys."
        )
        return

    known_hosts_file = os.path.expanduser("~/.ssh/known_hosts")
    os.makedirs(os.path.dirname(known_hosts_file), exist_ok=True)

    # Read existing lines (if any)
    existing = set()
    if os.path.exists(known_hosts_file):
        with open(known_hosts_file, "r", encoding="utf-8", errors="ignore") as kh:
            for line in kh:
                existing.add(line.rstrip("\n"))

    # Collect candidate entries
    new_lines = []
    for host in hosts:
        try:
            proc = subprocess.run(
                [ssh_keyscan, "-H", host], capture_output=True, text=True, timeout=10
            )
            if proc.returncode != 0 or not proc.stdout:
                print(f"ssh-keyscan failed for {host}: {proc.stderr.strip()}")
                continue
            for line in proc.stdout.splitlines():
                if line and line not in existing:
                    existing.add(line)
                    new_lines.append(line)
        except subprocess.TimeoutExpired:
            print(f"ssh-keyscan timed out for {host}")
        except Exception as exc:
            print(f"ssh-keyscan error for {host}: {exc}")

    if new_lines:
        # Append unique new lines atomically
        with open(known_hosts_file, "a", encoding="utf-8") as kh:
            for line in new_lines:
                kh.write(line + "\n")
        print(f"Added {len(new_lines)} host key(s) to {known_hosts_file}")
    else:
        print("No new host keys to add.")


def read_respeaker_rule(rule_file: str) -> str:
    expanded = os.path.abspath(os.path.expanduser(rule_file))
    with open(expanded, "r", encoding="utf-8") as handle:
        lines = handle.read().splitlines()

    candidates = []
    for line in lines:
        stripped = line.strip()
        if not stripped or stripped.startswith("#"):
            continue
        candidates.append(stripped)

    if not candidates:
        raise ValueError(f"No ReSpeaker permission rule found in {expanded}.")

    if len(candidates) == 1:
        return candidates[0]

    return "\n".join(candidates)


def grant_respeaker_permission_command(rule_file: str) -> str:
    rule = read_respeaker_rule(rule_file)
    return (
        "sudo mkdir -p /etc/udev/rules.d && "
        f"printf '%s\\n' {shlex.quote(rule)} | sudo tee /etc/udev/rules.d/99-respeaker.rules >/dev/null && "
        "sudo udevadm control --reload-rules && "
        "sudo udevadm trigger && "
        "echo 'ReSpeaker permissions granted'"
    )


def resolve_key_file(key_file: str | None) -> str | None:
    """Return an existing absolute path for key_file or None."""
    if not key_file:
        return None
    expanded = os.path.expanduser(key_file)
    abs_path = os.path.abspath(expanded)
    if os.path.exists(abs_path):
        return abs_path
    return None


def wrap_remote_command(
    command: str,
    remote_dir: str | None = None,
    conda_env: str | None = None,
    detach: bool = False,
    log_file: str = "fleet_run.log",
    start_time: float | None = None,
) -> str:
    """Wraps a shell command with directory navigation, PYTHONPATH, Conda activation, and optional detach."""
    parts = []

    if start_time:
        parts.append(f'export SWARM_START_TIME="{start_time}"')

    if conda_env:
        # Robust Conda sourcing for non-interactive shells
        conda_preamble = (
            'for _c in "$HOME/miniforge3" "$HOME/miniforge-pypy3" "$HOME/miniconda3" "$HOME/miniforge" "$HOME/mambaforge" "$HOME/anaconda3" "/opt/conda" "$HOME/.conda"; do '
            'if [ -f "$_c/etc/profile.d/conda.sh" ]; then . "$_c/etc/profile.d/conda.sh"; break; fi; '
            "done; "
            f"if command -v conda >/dev/null 2>&1; then conda activate {shlex.quote(conda_env)}; fi"
        )
        parts.append(conda_preamble)

    if remote_dir:
        parts.append(
            f"mkdir -p {shlex.quote(remote_dir)} && cd {shlex.quote(remote_dir)}"
        )
        parts.append(f'export PYTHONPATH="{remote_dir}:${{PYTHONPATH:-}}"')
        parts.append("export PYTHONUNBUFFERED=1")

    if detach:
        target_log = f"{remote_dir}/{log_file}" if remote_dir else log_file
        target_pid = f"{remote_dir}/fleet_run.pid" if remote_dir else "fleet_run.pid"
        wrapped_body = " && ".join(parts + [command]) if parts else command
        return (
            f"nohup bash -c {shlex.quote(wrapped_body)} > {shlex.quote(target_log)} 2>&1 & "
            f"PID=$!; echo $PID > {shlex.quote(target_pid)}; echo Started background process PID: $PID"
        )

    parts.append(command)
    return " && ".join(parts)


def build_stop_command(
    remote_dir: str | None = None,
    conda_env: str | None = None,
) -> str:
    """Construct a comprehensive stop command that terminates processes and halts robot motors."""
    pid_file = f"{remote_dir}/fleet_run.pid" if remote_dir else "fleet_run.pid"
    env_name = conda_env or "robat"
    py_zero_motors = (
        'python3 -u -c "'
        "import time, os, sys\n"
        "if os.path.exists('/home/sonoro/SonoRo_swarm'):\n"
        "    sys.path.extend(['/home/sonoro/SonoRo_swarm', '/home/sonoro/SonoRo_swarm/Scripts/attraction-repulsion', '/home/sonoro/SonoRo_swarm/Scripts/attraction', '/home/sonoro/SonoRo_swarm/Scripts/repulsion'])\n"
        "try:\n"
        "    from thymiodirect import Connection, Thymio\n"
        "    port = Connection.serial_default_port()\n"
        "    if port:\n"
        "        th = Thymio(serial_port=port)\n"
        "        th.connect()\n"
        "        time.sleep(0.8)\n"
        "        for nid in list(th.nodes()):\n"
        "            node = th[nid]\n"
        "            node['motor.left.target'] = 0\n"
        "            node['motor.right.target'] = 0\n"
        "            node['leds.top'] = [0, 0, 0]\n"
        "            node['leds.bottom.left'] = [0, 0, 0]\n"
        "            node['leds.bottom.right'] = [0, 0, 0]\n"
        "            node['leds.circle'] = [0, 0, 0, 0, 0, 0, 0, 0]\n"
        "        time.sleep(0.3)\n"
        "except Exception:\n"
        "    pass\n"
        "try:\n"
        "    import usb.core\n"
        "    from usb_pixel_ring_v2 import PixelRing\n"
        "    dev = usb.core.find(idVendor=0x2886, idProduct=0x0018)\n"
        "    if dev:\n"
        "        pr = PixelRing(dev)\n"
        "        pr.set_brightness(0x01)\n"
        "        pr.mono(0xFF0000)\n"
        "except Exception:\n"
        "    pass\n"
        "os._exit(0)\n"
        '" 2>/dev/null || true'
    )

    conda_part = (
        'for _c in "$HOME/miniforge3" "$HOME/miniforge-pypy3" "$HOME/miniconda3" "$HOME/miniforge" "$HOME/mambaforge" "$HOME/anaconda3" "/opt/conda" "$HOME/.conda"; do '
        'if [ -f "$_c/etc/profile.d/conda.sh" ]; then . "$_c/etc/profile.d/conda.sh"; break; fi; '
        "done; "
        f"if command -v conda >/dev/null 2>&1; then conda activate {shlex.quote(env_name)} 2>/dev/null || true; fi; "
    )

    stop_script = (
        f'pid_file="{pid_file}"; '
        'pids=""; '
        'if [ -f "$pid_file" ]; then pids=$(cat "$pid_file" 2>/dev/null); fi; '
        'raw_pids=$(pgrep -f "[s]onoro_swarm|[S]onoRo_swarm|[A]udioProcessor|[R]obotMove" 2>/dev/null || true); '
        "for p in $raw_pids; do "
        '  if [ "$p" != "$$" ] && [ "$p" != "$PPID" ]; then '
        "    comm=$(cat /proc/$p/comm 2>/dev/null || true); "
        '    if [ "$comm" != "bash" ] && [ "$comm" != "sh" ]; then pids="$pids $p"; fi; '
        "  fi; "
        "done; "
        'pids=$(echo "$pids" | tr " " "\\n" | sort -u | tr "\\n" " " | xargs); '
        'if [ -n "$pids" ]; then '
        '  echo "Stopping robot processes (PIDs: $pids)..."; '
        "  kill -INT $pids 2>/dev/null || true; "
        "  sleep 0.3; "
        "  kill -TERM $pids 2>/dev/null || true; "
        "  sleep 0.2; "
        "  kill -9 $pids 2>/dev/null || true; "
        '  rm -f "$pid_file"; '
        '  echo "Process shutdown complete."; '
        "else "
        '  echo "No active robot processes found."; '
        "fi; "
        "fuser -k -9 /dev/ttyACM0 2>/dev/null || true; "
        "sleep 0.2; "
        f"{conda_part}{py_zero_motors}; exit 0"
    )
    return stop_script


def pre_clean_command(remote_dir: str | None = None) -> str:
    """Fast pre-run cleanup to kill lingering robot processes and free /dev/ttyACM0."""
    pid_file = f"{remote_dir}/fleet_run.pid" if remote_dir else "fleet_run.pid"
    return (
        f'pid_file="{pid_file}"; '
        'if [ -f "$pid_file" ]; then '
        '  pids=$(cat "$pid_file" 2>/dev/null); '
        '  if [ -n "$pids" ]; then kill -9 $pids 2>/dev/null || true; fi; '
        '  rm -f "$pid_file"; '
        "fi; "
        'raw_pids=$(pgrep -f "[s]onoro_swarm|[S]onoRo_swarm|[A]udioProcessor|[R]obotMove" 2>/dev/null || true); '
        "for p in $raw_pids; do "
        '  if [ "$p" != "$$" ] && [ "$p" != "$PPID" ]; then '
        "    comm=$(cat /proc/$p/comm 2>/dev/null || true); "
        '    if [ "$comm" != "bash" ] && [ "$comm" != "sh" ]; then kill -9 $p 2>/dev/null || true; fi; '
        "  fi; "
        "done; "
        "fuser -k -9 /dev/ttyACM0 2>/dev/null || true; "
        "sleep 0.3; "
        "exit 0"
    )


def grant_respeaker_permissions_fleet(
    hosts: Sequence[str],
    user: str,
    workers: int,
    rule_file: str,
    key_file: str | None = None,
) -> None:
    command = grant_respeaker_permission_command(rule_file)
    with concurrent.futures.ThreadPoolExecutor(max_workers=max(1, workers)) as executor:
        futures = [
            executor.submit(
                remote_run, host, user, command, key_file=key_file, timeout=25
            )
            for host in hosts
        ]
        for future in concurrent.futures.as_completed(futures):
            host, status, stdout, stderr = future.result()
            print(
                f"[{host}] ReSpeaker permissions -> {'OK' if status == 0 else f'ERROR ({status})'}"
            )
            if stdout:
                print(stdout)
            if stderr:
                print(stderr)


def _ssh_command(
    host: str,
    user: str,
    command: str,
    key_file: str | None = None,
    connect_timeout: int = 10,
) -> list[str]:
    cmd = [
        "ssh",
        "-n",
        "-o",
        "BatchMode=yes",
        "-o",
        "StrictHostKeyChecking=no",
        "-o",
        "UserKnownHostsFile=/dev/null",
        "-o",
        "LogLevel=ERROR",
        "-o",
        f"ConnectTimeout={connect_timeout}",
    ]
    resolved_key = resolve_key_file(key_file)
    if resolved_key:
        cmd.extend(["-i", resolved_key])
    cmd.extend([f"{user}@{host}", f"bash -lc {shlex.quote(command)}"])
    return cmd


def remote_run(
    host: str,
    user: str,
    command: str,
    conda_env: str | None = None,
    remote_dir: str | None = None,
    key_file: str | None = None,
    detach: bool = False,
    timeout: int | None = 180,
    start_time: float | None = None,
    stream_output: bool = False,
) -> tuple[str, int, str, str]:
    """Run a command on a remote Raspberry Pi, optionally in a conda environment.

    Returns (host, returncode, stdout, stderr). Returns non-zero returncodes and
    stderr messages for timeout or other exceptions instead of raising.
    """
    full_command = wrap_remote_command(
        command,
        remote_dir=remote_dir,
        conda_env=conda_env,
        detach=detach,
        start_time=start_time,
    )

    effective_timeout = 30 if detach else timeout

    try:
        if stream_output and not detach:
            proc = subprocess.Popen(
                _ssh_command(host, user, full_command, key_file=key_file),
                stdin=subprocess.DEVNULL,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
            )
            stdout_lines = []
            try:
                if proc.stdout:
                    for line in iter(proc.stdout.readline, ""):
                        line_str = line.rstrip("\r\n")
                        if line_str:
                            stdout_lines.append(line_str)
                            print(f"[{host}] {line_str}", flush=True)
                proc.wait(timeout=effective_timeout)
            finally:
                if proc.poll() is None:
                    try:
                        proc.kill()
                        proc.wait(timeout=1.0)
                    except Exception:
                        pass
            return (
                host,
                proc.returncode or 0,
                "\n".join(stdout_lines),
                "",
            )
        else:
            completed = subprocess.run(
                _ssh_command(host, user, full_command, key_file=key_file),
                stdin=subprocess.DEVNULL,
                capture_output=True,
                text=True,
                timeout=effective_timeout,
            )
            return (
                host,
                completed.returncode,
                completed.stdout.strip(),
                completed.stderr.strip(),
            )
    except subprocess.TimeoutExpired as exc:
        return (host, 124, "", f"Timeout after {exc.timeout}s")
    except Exception as exc:  # defensive: never raise from worker threads
        return (host, 1, "", f"Exception: {exc}")


def ntp_sync_command(reference_epoch: int | None = None) -> str:
    """Synchronize the robot clock directly from the controller's UTC epoch."""
    if reference_epoch is None:
        reference_epoch = int(time.time())

    return (
        f"timeout 4 sudo -n date -u -s @{reference_epoch} >/dev/null 2>&1 || true; "
        "date -u"
    )


def deploy_repo(
    host: str,
    user: str,
    repo_path: str,
    remote_dir: str,
    key_file: str | None = None,
) -> tuple[str, int, str, str]:
    """Deploy the current repository to a remote host."""
    resolved_key = resolve_key_file(key_file)
    key_opt = f" -i {shlex.quote(resolved_key)}" if resolved_key else ""
    ssh_opts = (
        f"ssh -o BatchMode=yes -o StrictHostKeyChecking=no "
        f"-o UserKnownHostsFile=/dev/null -o LogLevel=ERROR -o ConnectTimeout=10{key_opt}"
    )
    if shutil.which("rsync"):
        cmd = [
            "rsync",
            "-a",
            "-v",
            "--exclude=.git",
            "--exclude=.venv",
            "--exclude=__pycache__",
            "--exclude=*.pyc",
            "-e",
            ssh_opts,
            f"{repo_path}/",
            f"{user}@{host}:{remote_dir}/",
        ]
        completed = subprocess.run(cmd, capture_output=True, text=True, timeout=300)
        return (
            host,
            completed.returncode,
            completed.stdout.strip(),
            completed.stderr.strip(),
        )

    remote_extract = f"bash -lc {shlex.quote(f'mkdir -p {remote_dir} && tar -xzf - -C {remote_dir}') }"
    tar_command = (
        f"tar -czf - --exclude=.git --exclude=.venv --exclude=__pycache__ --exclude='*.pyc' "
        f"-C {shlex.quote(repo_path)} . | {ssh_opts} {user}@{host} {remote_extract}"
    )
    completed = subprocess.run(
        tar_command, shell=True, capture_output=True, text=True, timeout=300
    )
    return (
        host,
        completed.returncode,
        completed.stdout.strip(),
        completed.stderr.strip(),
    )


def list_hosts(hosts: Sequence[str]) -> None:
    for host in hosts:
        print(host)


def start_countdown_timer(target_time: float) -> None:
    """Display an active countdown timer until the synchronized swarm start timestamp."""

    def _run() -> None:
        while True:
            remaining = target_time - time.time()
            if remaining <= 0:
                print(
                    "\r[Fleet Manager] Swarm started! (T=0.0s)                      \n",
                    flush=True,
                )
                break
            print(
                f"\r[Fleet Manager] Synchronized start in {remaining:.1f}s...   ",
                end="",
                flush=True,
            )
            time.sleep(0.1)

    threading.Thread(target=_run, daemon=True).start()


def run_fleet(
    hosts: Sequence[str],
    user: str,
    command: str,
    workers: int,
    conda_env: str | None = None,
    remote_dir: str | None = None,
    key_file: str | None = None,
    detach: bool = False,
    timeout: int | None = 180,
    start_time: float | None = None,
    stream_output: bool = False,
) -> None:
    with concurrent.futures.ThreadPoolExecutor(max_workers=max(1, workers)) as executor:
        futures = {}
        for host in hosts:
            futures[
                executor.submit(
                    remote_run,
                    host,
                    user,
                    command,
                    conda_env=conda_env,
                    remote_dir=remote_dir,
                    key_file=key_file,
                    detach=detach,
                    timeout=timeout,
                    start_time=start_time,
                    stream_output=stream_output,
                )
            ] = host

        for future in concurrent.futures.as_completed(futures):
            host = futures[future]
            try:
                host, status, stdout, stderr = future.result()
            except Exception as exc:
                print(f"[{host}] RUN -> EXCEPTION: {exc}")
                continue

            tag = f"[{host}]"
            if not stream_output or detach:
                if status == 0:
                    print(f"{tag} OK")
                    if stdout:
                        print(stdout)
                else:
                    print(f"{tag} FAILED ({status})")
                    if stdout:
                        print(stdout)
                    if stderr:
                        print(stderr)
            else:
                if status != 0:
                    print(f"{tag} Exited with code {status}")


def sync_time_fleet(
    hosts: Sequence[str],
    user: str,
    workers: int,
    reference_epoch: int | None = None,
    key_file: str | None = None,
) -> None:
    if reference_epoch is None:
        reference_epoch = int(time.time())

    with concurrent.futures.ThreadPoolExecutor(max_workers=max(1, workers)) as executor:
        futures = {}
        for host in hosts:
            cmd = ntp_sync_command(reference_epoch)
            print(f"Starting time sync on {host}")
            futures[
                executor.submit(
                    remote_run, host, user, cmd, key_file=key_file, timeout=25
                )
            ] = host

        for future in concurrent.futures.as_completed(futures):
            host = futures[future]
            try:
                host, status, stdout, stderr = future.result()
            except Exception as exc:
                print(f"[{host}] time sync -> EXCEPTION: {exc}")
                continue

            print(
                f"[{host}] time sync -> {'OK' if status == 0 else f'ERROR ({status})'}"
            )
            if stdout:
                print(stdout)
            if stderr:
                print(stderr)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Launch or sync a Raspberry Pi fleet from one workstation."
    )
    parser.add_argument("--base-ip", help="Shared fleet IP prefix, e.g. 192.168.1.")
    parser.add_argument(
        "--ip-list",
        help="Comma-separated list of hosts, e.g. 192.168.1.10,192.168.1.11",
    )
    parser.add_argument(
        "--ips-file",
        help="Path to a text file containing one IP per line or comma-separated hosts.",
    )
    parser.add_argument(
        "--start",
        type=int,
        default=10,
        help="First IP number in the range to use with --base-ip.",
    )
    parser.add_argument(
        "--end",
        type=int,
        default=20,
        help="Last IP number in the range to use with --base-ip.",
    )
    parser.add_argument(
        "--username",
        default=DEFAULT_USERNAME,
        help="SSH user on all Pis (default: thymio).",
    )
    parser.add_argument(
        "--remote-dir",
        default=DEFAULT_REMOTE_DIR,
        help="Destination folder on each robot.",
    )
    parser.add_argument("--command", help="Command to execute on every robot.")
    parser.add_argument(
        "--run",
        dest="run_command",
        help="Alias for --command and useful in shell completion.",
    )
    parser.add_argument(
        "--conda-env",
        help="Conda environment name to run the command in (e.g., robat).",
    )
    parser.add_argument(
        "--stop",
        action="store_true",
        help="Stop the running robot processes on every host with a built-in stop command.",
    )
    parser.add_argument(
        "--stop-command",
        default=DEFAULT_STOP_COMMAND,
        help="Custom stop command to execute on each robot. Used with --stop.",
    )
    parser.add_argument(
        "--setup-ssh",
        action="store_true",
        help="Create a fleet SSH key and install it on every robot so no password is required.",
    )
    parser.add_argument(
        "--setup-sudo",
        action="store_true",
        help="Configure passwordless sudo on all robots so fleet operations can run privileged tasks without prompts.",
    )
    parser.add_argument(
        "--key-file",
        default="~/.ssh/id_ed25519_sonoro_fleet",
        help="SSH private key path to create and install for passwordless access.",
    )
    parser.add_argument(
        "--grant-respeaker-permissions",
        action="store_true",
        help="Write the ReSpeaker USB udev rule to every robot and reload udev.",
    )
    parser.add_argument(
        "--respeaker-rule-file",
        default="respeaker_permission.txt",
        help="Text file containing the ReSpeaker udev permission rule.",
    )
    parser.add_argument(
        "--sync",
        action="store_true",
        help="Copy the current repository to every robot before running commands.",
    )
    parser.add_argument(
        "--time-sync",
        action="store_true",
        help="Synchronize all Pi clocks with NTP or the controller clock before launching the command.",
    )
    parser.add_argument(
        "--scan-host-keys",
        action="store_true",
        help="Run ssh-keyscan for every discovered host and append keys to ~/.ssh/known_hosts.",
    )
    parser.add_argument(
        "--detach",
        "-d",
        action="store_true",
        help="Run the command in background on each robot and exit immediately.",
    )
    parser.add_argument(
        "--timeout",
        type=int,
        default=180,
        help="Execution timeout in seconds for synchronous commands (default: 180).",
    )
    parser.add_argument(
        "--start-delay",
        type=float,
        default=2.0,
        help="Delay in seconds before synchronized swarm start (default: 6.0).",
    )
    parser.add_argument(
        "--no-sync-start",
        action="store_true",
        help="Disable the synchronized UTC start barrier.",
    )
    parser.add_argument(
        "--stream",
        action="store_true",
        help="Stream live output from all robots in real time.",
    )
    parser.add_argument(
        "--clean",
        action="store_true",
        help="Run explicit pre-clean across all robots to kill lingering processes and free ports.",
    )
    parser.add_argument(
        "--no-clean",
        action="store_true",
        help="Disable automatic pre-run cleanup before starting commands.",
    )
    parser.add_argument(
        "--workers",
        type=int,
        default=20,
        help="Concurrent SSH workers to use.",
    )
    parser.add_argument(
        "--list", action="store_true", help="Print the discovered robot hosts and exit."
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.run_command and args.command:
        print("Please use either --command or --run, not both.", file=sys.stderr)
        return 2
    if args.run_command:
        args.command = args.run_command

    try:
        hosts = parse_hosts(args)
    except ValueError as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 2

    try:
        if args.list:
            list_hosts(hosts)
            return 0

        if args.scan_host_keys:
            scan_host_keys_fleet(hosts)
            return 0

        if args.setup_ssh:
            setup_ssh_fleet(hosts, args.username, args.key_file, args.workers)

        if args.setup_sudo:
            setup_sudo_fleet(hosts, args.username, key_file=args.key_file)

        if args.grant_respeaker_permissions:
            rule_file = os.path.abspath(os.path.expanduser(args.respeaker_rule_file))
            if not os.path.exists(rule_file):
                fallback = os.path.abspath(
                    os.path.join(os.path.dirname(__file__), "respeaker_permission.txt")
                )
                if os.path.exists(fallback):
                    rule_file = fallback
            grant_respeaker_permissions_fleet(
                hosts, args.username, args.workers, rule_file, key_file=args.key_file
            )

        if args.sync:
            repo_path = os.path.abspath(os.path.dirname(__file__))
            with concurrent.futures.ThreadPoolExecutor(
                max_workers=max(1, args.workers)
            ) as executor:
                futures = [
                    executor.submit(
                        deploy_repo,
                        host,
                        args.username,
                        repo_path,
                        args.remote_dir,
                        key_file=args.key_file,
                    )
                    for host in hosts
                ]
                for future in concurrent.futures.as_completed(futures):
                    host, status, stdout, stderr = future.result()
                    print(
                        f"[{host}] deploy -> {'OK' if status == 0 else f'ERROR ({status})'}"
                    )
                    if stdout:
                        print(stdout)
                    if stderr:
                        print(stderr)

        if args.time_sync:
            sync_time_fleet(
                hosts,
                args.username,
                args.workers,
                reference_epoch=int(time.time()),
                key_file=args.key_file,
            )

        if args.stop:
            stop_cmd = (
                args.stop_command
                if args.stop_command != DEFAULT_STOP_COMMAND
                else build_stop_command(
                    remote_dir=args.remote_dir, conda_env=args.conda_env
                )
            )
            run_fleet(
                hosts,
                args.username,
                stop_cmd,
                args.workers,
                conda_env=None,
                remote_dir=args.remote_dir,
                key_file=args.key_file,
                detach=False,
                timeout=30,
                stream_output=False,
            )

        if args.clean or (args.command and not args.no_clean):
            pre_cmd = pre_clean_command(remote_dir=args.remote_dir)
            run_fleet(
                hosts,
                args.username,
                pre_cmd,
                args.workers,
                conda_env=None,
                remote_dir=args.remote_dir,
                key_file=args.key_file,
                detach=False,
                timeout=15,
                stream_output=False,
            )

        if args.command:
            effective_timeout = (
                None if (not args.detach and args.timeout == 180) else args.timeout
            )
            start_time = None
            if not args.no_sync_start:
                start_time = round(time.time() + args.start_delay, 2)
                print(
                    f"\n[Fleet Manager] Synchronized start barrier scheduled for UTC {start_time} (in {args.start_delay}s)"
                )
                start_countdown_timer(start_time)
            run_fleet(
                hosts,
                args.username,
                args.command,
                args.workers,
                conda_env=args.conda_env,
                remote_dir=args.remote_dir,
                key_file=args.key_file,
                detach=args.detach,
                timeout=effective_timeout,
                start_time=start_time,
                stream_output=args.stream,
            )
            if args.detach:
                ips_arg = (
                    f"--ips-file {args.ips_file}"
                    if args.ips_file
                    else (f"--ip-list {args.ip_list}" if args.ip_list else "")
                )
                print("\n[Fleet Manager] Swarm processes started in background.")
                print(
                    f"To stop all robots at any time, run:\n  python fleet_manager.py {ips_arg} --stop"
                )

        if (
            not args.command
            and not args.sync
            and not args.time_sync
            and not args.list
            and not args.stop
            and not args.setup_ssh
            and not args.setup_sudo
            and not args.grant_respeaker_permissions
        ):
            print(
                "No action requested. Use --list, --sync, --time-sync, --stop, --setup-ssh, --grant-respeaker-permissions, or --command.",
                file=sys.stderr,
            )
            return 2

    except KeyboardInterrupt:
        print("\n\n[Ctrl+C detected] Halting all robots across the fleet...")
        stop_cmd = build_stop_command(
            remote_dir=args.remote_dir, conda_env=args.conda_env
        )
        run_fleet(
            hosts,
            args.username,
            stop_cmd,
            args.workers,
            conda_env=None,
            remote_dir=args.remote_dir,
            key_file=args.key_file,
            detach=False,
            timeout=10,
            stream_output=False,
        )
        print("All robots halted successfully.")
        return 130

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
