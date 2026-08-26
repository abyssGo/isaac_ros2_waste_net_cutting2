"""Typed helpers for publishing the popup JSON status payload.

The existing vision nodes can import these classes.  No custom ROS interface is
required because the serialized payload is sent as ``std_msgs/msg/String``.
"""

from dataclasses import asdict, dataclass, field
import json
from typing import List, Optional, Sequence


NON_NET_CLASSES = ("plastic_bottle", "can", "buoy")


@dataclass
class DetectionDebug:
    """One detected non-net object's data for the popup."""

    object_id: str
    class_name: str
    confidence: float
    bbox: Sequence[int]
    action_included: bool = False
    action_state: str = "EXCLUDED"
    task_index: int = 0
    reject_reason: str = ""
    depth_m: Optional[float] = None
    camera_xyz: Optional[Sequence[float]] = None
    world_xyz: Optional[Sequence[float]] = None

    def __post_init__(self) -> None:
        if self.class_name not in NON_NET_CLASSES:
            raise ValueError(
                f"class_name must be one of {NON_NET_CLASSES}: {self.class_name}"
            )
        if len(self.bbox) != 4:
            raise ValueError("bbox must be [x1, y1, x2, y2]")
        self.confidence = float(self.confidence)
        self.bbox = [int(v) for v in self.bbox]
        if self.camera_xyz is not None:
            if len(self.camera_xyz) != 3:
                raise ValueError("camera_xyz must contain exactly three values")
            self.camera_xyz = [float(v) for v in self.camera_xyz]
        if self.world_xyz is not None:
            if len(self.world_xyz) != 3:
                raise ValueError("world_xyz must contain exactly three values")
            self.world_xyz = [float(v) for v in self.world_xyz]


@dataclass
class ValidationMetrics:
    """Offline metrics from the labelled YOLO validation dataset."""

    precision: Optional[float] = None
    recall: Optional[float] = None
    map50: Optional[float] = None
    map50_95: Optional[float] = None


@dataclass
class DebugStatus:
    """Full state displayed by one debug popup."""

    phase: str = "WAITING"
    scan: str = "INITIAL SCAN"
    station: str = "WAITING"
    action_server: str = "WAITING"
    current_task: int = 0
    total_tasks: int = 0
    detections: List[DetectionDebug] = field(default_factory=list)
    inference_ms: Optional[float] = None
    fps: Optional[float] = None
    validation: ValidationMetrics = field(default_factory=ValidationMetrics)
    message: str = "Waiting for vision status"
    model_name: str = "best.pt"
    device: str = "UNKNOWN"

    def to_dict(self) -> dict:
        return asdict(self)

    def to_json(self) -> str:
        return json.dumps(self.to_dict(), ensure_ascii=False, separators=(",", ":"))
