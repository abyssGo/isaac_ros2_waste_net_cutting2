"""Helpers shared by robot1_node / robot2_node.

sleep_sim_time
    Wait in *simulation* time. The nodes run with use_sim_time, but the old
    holds used ``time.sleep`` (wall clock): when Isaac Sim runs slower than real
    time (heavier scene, slower PC) less simulated time passed during a hold and
    Robot 2's suction check fired while the bottle was still moving
    (reports/factory_background/README.md section 3).

pull_into_reach
    M0609 reach check for Robot 1's horizontal cutting pose. Cut points at the
    top corners of the net can lie 6.5-11.4 mm outside the arm's reach
    (reports/ik_analysis/). The point is moved along its cut line toward the
    other cut point until it is reachable, or reported unreachable so the node
    can skip it at once instead of failing IK twice (~20 s).
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Optional, Sequence, Tuple

Vector3 = Tuple[float, float, float]


def sleep_sim_time(clock, seconds: float, wall_timeout_sec: Optional[float] = None,
                   poll_sec: float = 0.005) -> bool:
    """Block until ``clock`` (a node clock driven by /clock) advanced ``seconds``.

    Returns False if the wall-clock safety timeout expired first (simulation
    paused or /clock missing). Default timeout: 20 x seconds + 5 s.
    """
    if seconds <= 0.0:
        return True
    if wall_timeout_sec is None:
        wall_timeout_sec = 20.0 * seconds + 5.0
    start_ns = clock.now().nanoseconds
    target_ns = start_ns + int(seconds * 1e9)
    wall_deadline = time.monotonic() + wall_timeout_sec
    while True:
        now_ns = clock.now().nanoseconds
        if now_ns >= target_ns:
            return True
        if now_ns < start_ns:
            # Simulation reset rewound the clock: restart the wait from here.
            start_ns = now_ns
            target_ns = start_ns + int(seconds * 1e9)
        if time.monotonic() >= wall_deadline:
            return False
        time.sleep(poll_sec)


@dataclass(frozen=True)
class ReachModel:
    """Spherical reach of the wrist centre around the shoulder (metres, world frame)."""

    shoulder: Vector3
    arm_length: float              # upper arm + forearm
    wrist_from_tcp: Vector3        # TCP -> wrist centre for the fixed tool orientation

    def margin(self, tcp: Sequence[float]) -> float:
        """arm_length - |wrist - shoulder|; negative = out of reach."""
        wrist = [float(tcp[i]) + self.wrist_from_tcp[i] for i in range(3)]
        return self.arm_length - math.dist(wrist, self.shoulder)


@dataclass(frozen=True)
class ReachResult:
    """Outcome of pull_into_reach (point in world metres)."""

    point: Vector3
    shift_m: float
    margin_before: float
    margin_after: float
    reachable: bool


def pull_into_reach(model: ReachModel, point: Sequence[float], toward: Sequence[float],
                    min_margin: float, max_shift_ratio: float) -> ReachResult:
    """Move ``point`` toward ``toward`` (its partner cut point) until reachable.

    The shift is limited to ``max_shift_ratio`` of the segment length so the cut
    stays near the object edge. Bisection on the segment (margin grows toward a
    reachable partner).
    """
    p = tuple(float(v) for v in point)
    q = tuple(float(v) for v in toward)
    before = model.margin(p)
    if before >= min_margin:
        return ReachResult(p, 0.0, before, before, True)

    def at(t: float) -> Vector3:
        return tuple(p[i] + t * (q[i] - p[i]) for i in range(3))

    length = math.dist(p, q)
    t_hi = max_shift_ratio
    if length <= 0.0 or model.margin(at(t_hi)) < min_margin:
        return ReachResult(p, 0.0, before, before, False)
    t_lo = 0.0
    for _ in range(40):
        mid = 0.5 * (t_lo + t_hi)
        if model.margin(at(mid)) >= min_margin:
            t_hi = mid
        else:
            t_lo = mid
    moved = at(t_hi)
    return ReachResult(moved, t_hi * length, before, model.margin(moved), True)
