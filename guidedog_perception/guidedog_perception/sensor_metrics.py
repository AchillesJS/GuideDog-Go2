# SPDX-License-Identifier: Apache-2.0
"""Bounded, ROS-independent sensor timing measurements."""

from collections import deque
import math
import re
import statistics


def native_frequency(text):
    """Read the latest finite native CLI rate, without inferring message loss."""
    matches = re.findall(r'^average rate:\s+([0-9.eE+-]+)\s*$', text, re.MULTILINE)
    for value in reversed(matches):
        try:
            rate = float(value)
        except ValueError:
            continue
        if math.isfinite(rate) and rate >= 0:
            return rate
    return None


class StreamMetrics:
    """Keep only timestamps and metadata, never image or point-cloud payloads."""

    def __init__(self, window_sec=10.0, capacity=2048):
        """Allocate timestamp windows with a fixed upper storage bound."""
        self.window_sec = window_sec
        self.samples = deque(maxlen=capacity)
        self.count = 0
        self.frame_id = ''
        self.last_stamp_ns = None
        self.last_received = None
        self.last_advance = None
        self.timestamp_resets = 0
        self.duplicate_stamps = 0
        self.max_gap_sec = 0.0
        self.ages = deque(maxlen=capacity)

    def observe(self, received, stamp_ns, frame_id='', age_sec=None):
        """Record a monotonic receipt time and a source-clock timestamp."""
        if self.last_received is not None:
            self.max_gap_sec = max(self.max_gap_sec, received - self.last_received)
        if self.last_stamp_ns is not None and stamp_ns < self.last_stamp_ns:
            self.timestamp_resets += 1
            self.samples.clear()
            self.ages.clear()
        if stamp_ns == self.last_stamp_ns:
            self.duplicate_stamps += 1
        else:
            self.last_advance = received
        self.count += 1
        self.last_received = received
        self.last_stamp_ns = stamp_ns
        self.frame_id = frame_id
        self.samples.append((received, stamp_ns))
        if age_sec is not None:
            self.ages.append((received, age_sec))

    def snapshot(self, now):
        """Report rates in distinct clock domains and freshness in wall seconds."""
        cutoff = now - self.window_sec
        while self.samples and self.samples[0][0] < cutoff:
            self.samples.popleft()
        while self.ages and self.ages[0][0] < cutoff:
            self.ages.popleft()
        periods = [b[0] - a[0] for a, b in zip(self.samples, list(self.samples)[1:])]
        wall_span = self.samples[-1][0] - self.samples[0][0] if periods else 0
        stamp_span = (self.samples[-1][1] - self.samples[0][1]) / 1e9 if periods else 0
        age_values = sorted(value for _, value in self.ages)
        return {
            'messages_total': self.count,
            'samples_in_window': len(self.samples),
            'received_hz_wall': len(periods) / wall_span if wall_span > 0 else None,
            'received_hz_source_time': len(periods) / stamp_span if stamp_span > 0 else None,
            'source_time_per_wall_second': stamp_span / wall_span if wall_span > 0 else None,
            'since_last_message_wall_sec': (
                now - self.last_received if self.last_received is not None else None),
            'since_stamp_advanced_wall_sec': (
                now - self.last_advance if self.last_advance is not None else None),
            'max_gap_wall_sec': max(
                self.max_gap_sec,
                now - self.last_received if self.last_received is not None else 0),
            'jitter_wall_sec': statistics.pstdev(periods) if periods else None,
            'age_at_receipt_p95_sec': (
                age_values[min(len(age_values) - 1, int(0.95 * len(age_values)))]
                if age_values else None),
            'timestamp_resets': self.timestamp_resets,
            'duplicate_stamps': self.duplicate_stamps,
            'frame_id': self.frame_id,
        }


def assess_sensor(stats, *, stale_after, clock_ok, age_sec, tf_ok, expected_frame=''):
    """Return diagnostic issues; these thresholds are not motion safety limits."""
    issues = []
    if not stats['messages_total']:
        return ['no_messages']
    if stats['since_last_message_wall_sec'] > stale_after:
        issues.append('stale_stream')
    if not clock_ok:
        issues.append('clock_unavailable_or_stalled')
    if age_sec is not None:
        if age_sec < -0.05:
            issues.append('timestamp_in_future')
        elif age_sec > stale_after:
            issues.append('old_timestamp')
    if not stats['frame_id']:
        issues.append('empty_frame_id')
    elif expected_frame and stats['frame_id'] != expected_frame:
        issues.append('unexpected_frame_id')
    if not tf_ok:
        issues.append('tf_unavailable_at_message_time')
    if stats['timestamp_resets']:
        issues.append('source_clock_reset_observed')
    return issues
