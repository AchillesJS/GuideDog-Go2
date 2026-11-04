# SPDX-License-Identifier: Apache-2.0
"""Timing regressions: distinct clocks, pauses, reset and stale data."""

from guidedog_perception.sensor_metrics import assess_sensor, native_frequency, StreamMetrics
import pytest


def issues(stats, **overrides):
    """Assess with an otherwise healthy clock and transform."""
    options = {'stale_after': 2.0, 'clock_ok': True, 'age_sec': 0.1,
               'tf_ok': True, 'expected_frame': 'camera_link'}
    options.update(overrides)
    return assess_sensor(stats, **options)


def test_wall_and_source_rates_remain_distinct():
    """A slow simulator must not be interpreted as message loss."""
    stream = StreamMetrics()
    for i in range(5):
        stream.observe(10 + i * 0.5, i * 100_000_000, 'camera_link', 0.01)
    stats = stream.snapshot(12)
    assert stats['received_hz_wall'] == pytest.approx(2)
    assert stats['received_hz_source_time'] == pytest.approx(10)
    assert stats['source_time_per_wall_second'] == pytest.approx(0.2)
    assert issues(stats) == []


def test_staleness_uses_wall_time_even_if_ros_time_stops():
    """A frozen ROS clock must not freeze the stale-data deadline."""
    stream = StreamMetrics()
    stream.observe(0, 5_000_000_000, 'camera_link')
    stats = stream.snapshot(3)
    assert 'stale_stream' in issues(stats, clock_ok=False, age_sec=0.0)
    assert 'clock_unavailable_or_stalled' in issues(stats, clock_ok=False)


def test_expired_samples_do_not_show_an_old_healthy_rate():
    """Historical messages must leave the rolling rate window."""
    stream = StreamMetrics(window_sec=2)
    stream.observe(0, 0, 'camera_link')
    stream.observe(1, 1_000_000_000, 'camera_link')
    stats = stream.snapshot(5)
    assert stats['samples_in_window'] == 0
    assert stats['received_hz_wall'] is None
    assert stats['messages_total'] == 2
    assert 'stale_stream' in issues(stats)


def test_source_reset_starts_a_new_rate_window():
    """Do not calculate frequency across a simulation reset."""
    stream = StreamMetrics()
    stream.observe(0, 10_000_000_000, 'camera_link', 0.1)
    stream.observe(1, 11_000_000_000, 'camera_link', 0.1)
    stream.observe(2, 0, 'camera_link', 0.1)
    stats = stream.snapshot(2)
    assert stats['timestamp_resets'] == 1
    assert stats['received_hz_source_time'] is None
    assert 'source_clock_reset_observed' in issues(stats)


def test_missing_frames_and_tf_are_not_healthy():
    """Require data, a valid frame and a usable transform."""
    stream = StreamMetrics()
    assert issues(stream.snapshot(0)) == ['no_messages']
    stream.observe(0, 0)
    found = issues(stream.snapshot(0), tf_ok=False)
    assert 'empty_frame_id' in found
    assert 'tf_unavailable_at_message_time' in found
    stream.observe(1, 1_000_000_000, 'wrong_frame')
    assert 'unexpected_frame_id' in issues(stream.snapshot(1))


@pytest.mark.parametrize('age, expected', [(-0.2, 'timestamp_in_future'), (3, 'old_timestamp')])
def test_clock_domain_errors_are_reported(age, expected):
    """Do not hide old or future timestamps by clipping their age."""
    stream = StreamMetrics()
    stream.observe(0, 0, 'camera_link')
    assert expected in issues(stream.snapshot(0), age_sec=age)


def test_memory_is_bounded_and_payloads_are_not_retained():
    """Long runs must not accumulate an unbounded timestamp history."""
    stream = StreamMetrics(capacity=8)
    for i in range(100):
        stream.observe(i / 100, i * 1_000_000, 'camera_link', 0.0)
    assert len(stream.samples) == 8
    assert len(stream.ages) == 8
    assert stream.count == 100


def test_native_probe_handles_partial_logs_and_missing_values():
    """Unavailable or incomplete native measurements must remain unknown."""
    assert native_frequency('no publisher yet\naverage rate: ') is None
    assert native_frequency('average rate: 7.2\nmin: 0.1s\naverage rate: 9.3\n') == 9.3
    assert native_frequency('average rate: 2e1\n') == 20
    assert native_frequency('average rate: -1\n') is None
