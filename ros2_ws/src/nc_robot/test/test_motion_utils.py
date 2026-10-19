import math

from nc_robot.motion_utils import ReachModel, pull_into_reach, sleep_sim_time

# Same numbers as robot1_node (M0609, horizontal cutting pose, robot1 base at y = -2.0).
MODEL = ReachModel(shoulder=(0.0, -2.0, 0.6345), arm_length=0.779, wrist_from_tcp=(0.251, 0.0, 0.0056))


def test_reach_margin_matches_logged_ik_results():
    # reports/gpu_logs/robot1_contact_reach.csv: IK failed at y=-2.437 (margin -8.8 mm),
    # succeeded at y=-2.427 (P1 of the same bottle, next cycle).
    failed = MODEL.margin((-0.690, -2.437, 1.112))
    succeeded = MODEL.margin((-0.690, -2.427, 1.112))
    assert failed < -0.006
    assert succeeded > failed


def test_reachable_point_is_unchanged():
    point = (-0.690, -2.194, 1.000)
    result = pull_into_reach(MODEL, point, (-0.690, -1.876, 1.000), 0.003, 0.4)
    assert result.reachable and result.shift_m == 0.0 and result.point == point


def test_out_of_reach_point_is_pulled_toward_partner():
    p1, p2 = (-0.690, -2.437, 1.112), (-0.690, -2.437, 0.817)   # bottle P1 / P2 (run10 cycle 1)
    result = pull_into_reach(MODEL, p1, p2, 0.003, 0.4)
    assert result.reachable
    assert abs(result.margin_after - 0.003) < 1e-6
    assert 0.0 < result.shift_m < 0.4 * math.dist(p1, p2)
    assert result.point[2] < p1[2] and result.point[1] == p1[1]


def test_unreachable_segment_is_reported():
    far = (-0.690, -3.0, 1.4)
    result = pull_into_reach(MODEL, far, (-0.690, -3.0, 1.3), 0.003, 0.4)
    assert not result.reachable and result.point == far


class FakeClock:
    """Clock whose time advances 0.1 s of sim time per call to now()."""

    class _T:
        def __init__(self, ns):
            self.nanoseconds = ns

    def __init__(self, step_ns=100_000_000, start_ns=0):
        self.ns, self.step = start_ns, step_ns

    def now(self):
        self.ns += self.step
        return self._T(self.ns)


def test_sleep_sim_time_waits_for_sim_clock():
    clock = FakeClock()
    assert sleep_sim_time(clock, 0.5, poll_sec=0.0)
    assert clock.ns >= 600_000_000


def test_sleep_sim_time_times_out_when_clock_stops():
    clock = FakeClock(step_ns=0)
    assert not sleep_sim_time(clock, 0.5, wall_timeout_sec=0.05, poll_sec=0.001)
