"""Copy the relevant parts into vision1_node.py and vision2_node.py.

This is an integration snippet, not a standalone node. Replace the example
variables with values from the actual YOLO result and Action feedback.
"""

from std_msgs.msg import String

from nc_vision_debug.debug_payload import (
    DebugStatus,
    DetectionDebug,
    ValidationMetrics,
)


def add_debug_publishers(self, vision_number: int) -> None:
    """Call once from the existing vision node's __init__."""
    self._debug_status_pub = self.create_publisher(
        String,
        f"/vision{vision_number}/debug/status",
        10,
    )


def publish_vision1_example(self) -> None:
    """Publish after Vision 1 inference and Action target filtering."""
    detections = [
        DetectionDebug(
            object_id="plastic_bottle_01",
            class_name="plastic_bottle",
            confidence=0.87,
            bbox=[350, 201, 492, 371],
            action_included=True,
            action_state="IN_PROGRESS",
            task_index=1,
        ),
        DetectionDebug(
            object_id="buoy_01",
            class_name="buoy",
            confidence=0.42,
            bbox=[610, 220, 760, 390],
            action_included=False,
            action_state="EXCLUDED",
            reject_reason="LOW CONFIDENCE",
        ),
    ]
    status = DebugStatus(
        phase="DETECTING",
        scan="INITIAL SCAN",
        station="ARRIVED",
        action_server="ACTIVE",
        current_task=1,
        total_tasks=1,
        detections=detections,
        inference_ms=18.4,
        fps=31.2,
        # Replace with real values from the labelled validation result.
        validation=ValidationMetrics(),
        message="CUTTING OBJECT 1/1 - plastic_bottle_01 - ACTION IN PROGRESS",
        model_name="best.pt",
        device="CUDA:0",
    )
    self._debug_status_pub.publish(String(data=status.to_json()))


def publish_vision2_example(self) -> None:
    """Publish after Vision 2 RGB-D conversion and Action target filtering."""
    detections = [
        DetectionDebug(
            object_id="plastic_bottle_01",
            class_name="plastic_bottle",
            confidence=0.87,
            bbox=[350, 201, 492, 371],
            action_included=True,
            action_state="COMPLETE",
            task_index=1,
            depth_m=1.240,
            camera_xyz=[-0.612, 2.104, 0.823],
        ),
        DetectionDebug(
            object_id="can_01",
            class_name="can",
            confidence=0.91,
            bbox=[520, 230, 660, 410],
            action_included=True,
            action_state="REMOVING",
            task_index=2,
            depth_m=1.180,
            camera_xyz=[-0.530, 2.080, 0.790],
        ),
        DetectionDebug(
            object_id="buoy_01",
            class_name="buoy",
            confidence=0.84,
            bbox=[720, 250, 880, 430],
            action_included=True,
            action_state="QUEUED",
            task_index=3,
            depth_m=1.205,
            camera_xyz=[-0.440, 2.115, 0.815],
        ),
    ]
    status = DebugStatus(
        phase="ROBOT WORKING",
        scan="INITIAL SCAN",
        station="ARRIVED",
        action_server="ACTIVE",
        current_task=2,
        total_tasks=3,
        detections=detections,
        inference_ms=17.9,
        fps=32.1,
        validation=ValidationMetrics(),
        message="REMOVING OBJECT 2/3 - can_01 - SUCTION ACTION IN PROGRESS",
        model_name="best.pt",
        device="CUDA:0",
    )
    self._debug_status_pub.publish(String(data=status.to_json()))
