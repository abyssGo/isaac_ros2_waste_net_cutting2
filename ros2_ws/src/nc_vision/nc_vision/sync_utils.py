"""RGB-D synchronizer helpers that survive Isaac Sim cycle resets.

``message_filters.ApproximateTimeSynchronizer`` keeps up to ``queue_size``
unmatched messages per topic and, when a queue overflows, drops the message
with the *smallest* stamp.  Isaac Sim's ``world.reset()`` rewinds simulation
time to zero, so after a reset every new frame is older than the frames left
over from the previous cycle.  Once a queue is full of those stale frames the
new frame is dropped the moment it arrives, and no RGB-D pair is produced
until simulation time catches up with the stale stamps (about one cycle,
~120 s).  Clearing the queues on station arrival and on a backward stamp jump
removes that stall.
"""

from __future__ import annotations


def stamp_to_ns(stamp) -> int:
    """Convert a builtin_interfaces/Time to integer nanoseconds."""
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def clear_synchronizer_queues(synchronizer) -> int:
    """Drop every queued message; return how many were removed."""
    removed = 0
    with synchronizer.lock:
        for queue in synchronizer.queues:
            removed += len(queue)
            queue.clear()
    return removed


class StampRewindGuard:
    """Detect a backward jump in camera stamps (simulation reset)."""

    def __init__(self, tolerance_sec: float = 1.0) -> None:
        self.tolerance_ns = int(tolerance_sec * 1e9)
        self.latest_ns: int | None = None

    def update(self, stamp_ns: int) -> bool:
        """Record ``stamp_ns``; return True when time moved backwards."""
        rewound = (
            self.latest_ns is not None
            and stamp_ns < self.latest_ns - self.tolerance_ns
        )
        if self.latest_ns is None or rewound or stamp_ns > self.latest_ns:
            self.latest_ns = stamp_ns
        return rewound
