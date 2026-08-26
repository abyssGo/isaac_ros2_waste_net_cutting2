#!/usr/bin/env python3
"""NetClean standalone에서 사용하는 시간 보간 도구.

Isaac Sim 모듈에 의존하지 않으므로 일반 Python에서도 단위 시험할 수 있다.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

import numpy as np


QUINTIC_MAX_DERIVATIVE = 1.875


def quintic_blend(value: float) -> float:
    """0~1 구간의 jerk-free quintic time scaling을 반환한다."""
    t = min(1.0, max(0.0, float(value)))
    return 10.0 * t**3 - 15.0 * t**4 + 6.0 * t**5


def duration_from_velocity_limits(
    start: Sequence[float],
    target: Sequence[float],
    velocity_limits: Sequence[float],
    minimum_duration: float,
    maximum_duration: float,
) -> float:
    """Quintic 궤적의 관절 속도가 제한값을 넘지 않도록 시간을 계산한다."""
    q0 = np.asarray(start, dtype=float)
    q1 = np.asarray(target, dtype=float)
    limits = np.asarray(velocity_limits, dtype=float)

    if q0.shape != q1.shape or q0.shape != limits.shape:
        raise ValueError(
            "start, target, velocity_limits의 shape가 같아야 합니다: "
            f"{q0.shape}, {q1.shape}, {limits.shape}"
        )
    if np.any(~np.isfinite(q0)) or np.any(~np.isfinite(q1)):
        raise ValueError("관절값에 NaN 또는 inf가 포함되어 있습니다.")
    if np.any(~np.isfinite(limits)) or np.any(limits <= 0.0):
        raise ValueError("velocity_limits는 양의 유한값이어야 합니다.")
    if minimum_duration <= 0.0 or maximum_duration < minimum_duration:
        raise ValueError("동작 시간 범위가 잘못되었습니다.")

    required = QUINTIC_MAX_DERIVATIVE * np.abs(q1 - q0) / limits
    duration = max(float(minimum_duration), float(np.max(required, initial=0.0)))
    if duration > float(maximum_duration):
        raise ValueError(
            "속도 제한을 지키려면 허용된 최대 동작시간보다 오래 걸립니다: "
            f"required={duration:.3f}s, maximum={maximum_duration:.3f}s"
        )
    return duration


@dataclass
class QuinticJointTrajectory:
    """고정 시작/목표 관절 사이의 quintic trajectory."""

    start: np.ndarray
    target: np.ndarray
    duration: float
    elapsed: float = 0.0

    @classmethod
    def create(
        cls,
        start: Sequence[float],
        target: Sequence[float],
        velocity_limits: Sequence[float],
        minimum_duration: float,
        maximum_duration: float,
    ) -> "QuinticJointTrajectory":
        q0 = np.asarray(start, dtype=float).copy()
        q1 = np.asarray(target, dtype=float).copy()
        duration = duration_from_velocity_limits(
            q0,
            q1,
            velocity_limits,
            minimum_duration,
            maximum_duration,
        )
        return cls(start=q0, target=q1, duration=duration)

    @property
    def complete(self) -> bool:
        return self.elapsed >= self.duration

    def sample(self, dt: float) -> np.ndarray:
        if dt < 0.0:
            raise ValueError("dt는 음수일 수 없습니다.")
        self.elapsed = min(self.duration, self.elapsed + float(dt))
        ratio = self.elapsed / max(self.duration, 1.0e-9)
        blend = quintic_blend(ratio)
        return self.start + blend * (self.target - self.start)
