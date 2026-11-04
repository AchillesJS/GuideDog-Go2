# SPDX-License-Identifier: Apache-2.0
"""Verify cleanup of a detached child without signalling unrelated processes."""

import subprocess
import sys
import time

from guidedog_perception.launch_supervisor import (
    alive_owned, remember_descendants, stop_owned)


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
