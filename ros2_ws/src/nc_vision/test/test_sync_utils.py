import message_filters
from sensor_msgs.msg import Image

from nc_vision.sync_utils import (
    StampRewindGuard,
    clear_synchronizer_queues,
    stamp_to_ns,
)


def _image(seconds: float) -> Image:
    message = Image()
    message.header.stamp.sec = int(seconds)
    message.header.stamp.nanosec = int(round((seconds % 1) * 1e9))
    return message


def _make_sync():
    rgb = message_filters.SimpleFilter()
    depth = message_filters.SimpleFilter()
    sync = message_filters.ApproximateTimeSynchronizer(
        [rgb, depth], queue_size=10, slop=0.10
    )
    pairs = []
    sync.registerCallback(lambda r, d: pairs.append(stamp_to_ns(r.header.stamp)))
    return rgb, depth, sync, pairs


def _fill_with_stale_depth(depth) -> None:
    # Previous cycle: 10 depth frames whose RGB partners never matched.
    for index in range(10):
        depth.signalMessage(_image(118.0 + 0.08 * index))


def test_stale_frames_block_pairs_after_time_rewind():
    rgb, depth, _sync, pairs = _make_sync()
    _fill_with_stale_depth(depth)
    for seconds in (1.0, 1.5, 2.0):  # after reset, time restarts near zero
        depth.signalMessage(_image(seconds))
        rgb.signalMessage(_image(seconds))
    assert pairs == []


def test_clearing_queues_restores_pairs_after_time_rewind():
    rgb, depth, sync, pairs = _make_sync()
    _fill_with_stale_depth(depth)
    assert clear_synchronizer_queues(sync) == 10
    for seconds in (1.0, 1.5, 2.0):
        depth.signalMessage(_image(seconds))
        rgb.signalMessage(_image(seconds))
    assert len(pairs) == 3


def test_rewind_guard_flags_only_backward_jumps():
    guard = StampRewindGuard(tolerance_sec=1.0)
    assert not guard.update(stamp_to_ns(_image(117.0).header.stamp))
    assert not guard.update(stamp_to_ns(_image(118.0).header.stamp))
    assert not guard.update(stamp_to_ns(_image(117.5).header.stamp))
    assert guard.update(stamp_to_ns(_image(0.6).header.stamp))
    assert not guard.update(stamp_to_ns(_image(0.7).header.stamp))
