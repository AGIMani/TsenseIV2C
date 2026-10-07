#!/usr/bin/env python3
"""Replace a matching local Linux application instance before starting it."""
import fcntl
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

def matches(pid, command, cwd):
    try:
        proc = Path("/proc") / str(pid)
        if proc.stat().st_uid != os.getuid():
            return False
        args = (proc / "cmdline").read_bytes().decode().split("\0")
        return (len(args) >= 2 and args[:2] == command[:2]
                and (proc / "cwd").resolve() == cwd)
    except (OSError, ValueError):
        return False

def main():
    group, *command = sys.argv[1:]
    if not command or "/" in group:
        raise SystemExit("Usage: restart_local_service.py GROUP EXECUTABLE SCRIPT [ARGS]")
    # Diagnostic invocations should never replace a running application.
    if any(arg in command[2:] for arg in ("--check", "--help", "-h")):
        os.execv(command[0], command)
    state = Path.home() / ".local/state/hand-launchers"
    state.mkdir(parents=True, exist_ok=True)
    cwd = Path.cwd().resolve()
    with (state / (group + ".lock")).open("a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        old = [int(p.name) for p in Path("/proc").iterdir()
               if p.name.isdigit() and int(p.name) != os.getpid()
               and matches(int(p.name), command, cwd)]
        for pid in old:
            print(f"Stopping previous {group} instance (PID {pid})...", flush=True)
            try:
                os.kill(pid, signal.SIGINT)
            except ProcessLookupError:
                pass
        deadline = time.monotonic() + 12
        while old and time.monotonic() < deadline:
            old = [pid for pid in old if matches(pid, command, cwd)]
            if old:
                time.sleep(0.1)
        for pid in old:
            if matches(pid, command, cwd):
                os.kill(pid, signal.SIGTERM)
        deadline = time.monotonic() + 5
        while old and time.monotonic() < deadline:
            old = [pid for pid in old if matches(pid, command, cwd)]
            if old:
                time.sleep(0.1)
        if old:
            raise SystemExit(f"Previous instance did not exit: {old}. New launch cancelled.")
        child = subprocess.Popen(command, start_new_session=True)
        def forward(sig, _frame):
            if child.poll() is None:
                try:
                    os.killpg(child.pid, sig)
                except ProcessLookupError:
                    pass
        signal.signal(signal.SIGINT, forward)
        signal.signal(signal.SIGTERM, forward)
        # Keep concurrent clicks serialized until the new PID is visible.
        fcntl.flock(lock, fcntl.LOCK_UN)
    code = child.wait()
    return 0 if code in (-signal.SIGINT, -signal.SIGTERM) else code

if __name__ == "__main__":
    sys.exit(main())
