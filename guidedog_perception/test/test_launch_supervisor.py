# SPDX-License-Identifier: Apache-2.0
"""Verify cleanup of a detached child without signalling unrelated processes."""

import os
import signal
import subprocess
import sys
import time

from guidedog_perception.launch_supervisor import (
    alive_owned, remember_descendants, stop_owned)
import pytest


def test_detached_child_is_cleaned_after_parent_exits():
    """Reap descendants even when the launch parent finishes before its children."""
    child_code = ('import subprocess,time; '
                  'subprocess.Popen(["sleep","60"],start_new_session=True); time.sleep(60)')
    process = subprocess.Popen([sys.executable, '-c', child_code], start_new_session=True)
    unrelated = subprocess.Popen(['sleep', '60'], start_new_session=True)
    known = {}
    try:
        end = time.monotonic() + 3
        while time.monotonic() < end:
            remember_descendants(process.pid, known)
            if len(known) >= 2:
                break
            time.sleep(0.02)
        assert len(known) >= 2
        assert stop_owned(process, known) == []
        assert alive_owned(known) == []
        assert unrelated.poll() is None
    finally:
        stop_owned(process, known)
        unrelated.terminate()
        unrelated.wait(timeout=3)


@pytest.mark.parametrize('signum', [signal.SIGINT, signal.SIGTERM, signal.SIGHUP])
def test_supervisor_signals_and_duplicate_launch(tmp_path, signum):
    """A terminal hangup must reap detached children and a duplicate cannot launch."""
    fake = tmp_path / 'ros2'
    marker = tmp_path / 'started'
    fake.write_text('#!/usr/bin/env python3\n'
                    'import os,subprocess,time\n'
                    'from pathlib import Path\n'
                    'subprocess.Popen(["sleep","60"],start_new_session=True)\n'
                    'Path(os.environ["START_MARKER"]).write_text(str(os.getpid()))\n'
                    'time.sleep(60)\n')
    fake.chmod(0o755)
    env = dict(os.environ, PATH=f'{tmp_path}:{os.environ["PATH"]}',
               XDG_RUNTIME_DIR=str(tmp_path), START_MARKER=str(marker),
               ROS_DOMAIN_ID='98', GZ_PARTITION='supervisor_signal_test')
    args = [sys.executable, '-m', 'guidedog_perception.launch_supervisor', 'test', 'fake.py']
    process = subprocess.Popen(args, env=env, start_new_session=True)
    unrelated = subprocess.Popen(['sleep', '60'], start_new_session=True)
    known = {}
    second = None
    second_known = {}
    try:
        end = time.monotonic()+5
        while time.monotonic() < end:
            remember_descendants(process.pid, known)
            if marker.exists() and len(known) >= 3:
                break
            time.sleep(0.02)
        assert marker.exists() and len(known) >= 3
        if signum == signal.SIGINT:
            initial = marker.read_text()
            second = subprocess.Popen(args, env=env, stdout=subprocess.PIPE,
                                      stderr=subprocess.PIPE, text=True, start_new_session=True)
            remember_descendants(second.pid, second_known)
            _, error = second.communicate(timeout=3)
            assert second.returncode != 0 and 'already running' in error
            assert marker.read_text() == initial
        process.send_signal(signum)
        assert process.wait(timeout=8) == 0
        assert alive_owned(known) == []
        assert unrelated.poll() is None
    finally:
        stop_owned(process, known)
        if second is not None:
            stop_owned(second, second_known)
        unrelated.terminate()
        unrelated.wait(timeout=3)
