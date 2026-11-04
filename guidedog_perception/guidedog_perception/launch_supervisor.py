# SPDX-License-Identifier: Apache-2.0
"""Own and clean up a ROS/Gazebo process tree, including detached Ruby children."""

import os
from pathlib import Path
import signal
import subprocess
import sys
import time


def process_table():
    """Read Linux parent IDs and start times without examining unrelated data."""
    result = {}
    for path in Path('/proc').iterdir():
        if not path.name.isdigit():
            continue
        try:
            fields = (path / 'stat').read_text().rsplit(')', 1)[1].split()
            result[int(path.name)] = (int(fields[1]), fields[19], fields[0])
        except (OSError, IndexError, ValueError):
            pass
    return result


def remember_descendants(pid, known):
    """Remember start times to avoid signalling a reused PID during cleanup."""
    table = process_table()
    members = {pid} if pid in table else set()
    while True:
        children = {child for child, row in table.items() if row[0] in members}
        if children <= members:
            break
        members.update(children)
    known.update({member: table[member][1] for member in members})


def alive_owned(known):
    """Return only still-live processes with matching original start times."""
    table = process_table()
    return [pid for pid, start in known.items()
            if pid in table and table[pid][1] == start and table[pid][2] != 'Z']


def stop_owned(process, known):
    """Stop the launch first, then clean up its surviving detached descendants."""
    remember_descendants(process.pid, known)
    if process.poll() is None:
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            pass
    for sig, grace in ((signal.SIGINT, 1.0), (signal.SIGTERM, 2.0), (signal.SIGKILL, 0.2)):
        remaining = alive_owned(known)
        if not remaining:
            break
        for pid in remaining:
            try:
                os.kill(pid, sig)
            except ProcessLookupError:
                pass
        end = time.monotonic() + grace
        while time.monotonic() < end and alive_owned(known):
            time.sleep(0.05)
    return alive_owned(known)


def main():
    """Forward a user stop once and reap the entire owned simulation tree."""
    if len(sys.argv) < 3:
        raise SystemExit('usage: launch_supervisor PACKAGE FILE.launch.py [name:=value ...]')
    stopping = False

    def stop(signum, frame):
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    process = subprocess.Popen(['ros2', 'launch', *sys.argv[1:]], start_new_session=True)
    known = {}
    try:
        while process.poll() is None and not stopping:
            remember_descendants(process.pid, known)
            time.sleep(0.5)
    finally:
        remaining = stop_owned(process, known)
    if remaining:
        raise SystemExit(f'Simulation children still running: {remaining}')
    if not stopping and process.returncode:
        raise SystemExit(process.returncode)


if __name__ == '__main__':
    main()
