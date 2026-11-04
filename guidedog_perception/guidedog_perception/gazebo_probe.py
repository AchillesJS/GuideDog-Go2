# SPDX-License-Identifier: Apache-2.0
"""Capture bounded native Gazebo frequency measurements without ROS decoding."""

import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time


def main():
    """Capture two native frequency probes and stop only owned child processes."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--directory', required=True)
    parser.add_argument('--duration', type=float, default=60.0)
    args = parser.parse_args()
    if not 0 < args.duration <= 3600:
        parser.error('duration must be between 0 and 3600 seconds')
    directory = Path(args.directory).expanduser()
    directory.mkdir(parents=True, exist_ok=True)
    stopped = False

    def stop(signum, frame):
        nonlocal stopped
        stopped = True

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    jobs = []
    result = {}
    started = time.monotonic()
    try:
        for name, topic in (('rgb', '/rgb_image'), ('lidar', '/unitree_lidar/points')):
            output = (directory / f'gazebo-{name}.log').open('w')
            command = ['gz', 'topic', '-f', '-t', topic, '-d', str(args.duration)]
            try:
                process = subprocess.Popen(
                    command, stdout=output, stderr=subprocess.STDOUT,
                    stdin=subprocess.DEVNULL, start_new_session=True)
            except OSError:
                output.close()
                raise
            jobs.append((name, process, output))
            result[name] = {'topic': topic, 'command': command}
        while not stopped and any(process.poll() is None for _, process, _ in jobs):
            if time.monotonic() - started >= args.duration + 5:
                break
            time.sleep(0.1)
    finally:
        for name, process, output in jobs:
            if process.poll() is None:
                try:
                    os.killpg(process.pid, signal.SIGINT)
                    process.wait(timeout=3)
                except (ProcessLookupError, subprocess.TimeoutExpired):
                    if process.poll() is None:
                        os.killpg(process.pid, signal.SIGKILL)
                        process.wait()
            output.close()
            result[name]['exit_code'] = process.returncode
        result['elapsed_wall_sec'] = time.monotonic() - started
        result['interrupted'] = stopped
        (directory / 'gazebo-probes.json').write_text(json.dumps(result, indent=2) + '\n')


if __name__ == '__main__':
    main()
