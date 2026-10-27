"""sensor_msgs/Image helpers that do not depend on cv_bridge's OpenCV build.

``cv_bridge.cv2_to_imgmsg`` looks up ``cv2.CV_8UC3`` from whatever OpenCV the
Python process imported.  Jazzy's cv_bridge is compiled against OpenCV 4.6, so
when a pip OpenCV 5 wheel shadows the system package the lookup fails with
``KeyError: 16`` and every debug/popup frame is dropped.  Building the message
by hand avoids that mismatch for the single encoding NetClean publishes.
"""

from __future__ import annotations

import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Header


def bgr8_to_imgmsg(
    image: np.ndarray,
    header: Header | None = None,
    frame_id: str | None = None,
) -> Image:
    """Wrap an HxWx3 uint8 BGR array in a ``bgr8`` sensor_msgs/Image."""
    array = np.asarray(image)
    if array.ndim != 3 or array.shape[2] != 3:
        raise ValueError(
            f"bgr8 image must have shape (H, W, 3); got {array.shape}"
        )
    if array.dtype != np.uint8:
        raise ValueError(f"bgr8 image must be uint8; got {array.dtype}")

    array = np.ascontiguousarray(array)
    message = Image()
    if header is not None:
        message.header.stamp = header.stamp
        message.header.frame_id = header.frame_id
    if frame_id is not None:
        message.header.frame_id = frame_id
    message.height = int(array.shape[0])
    message.width = int(array.shape[1])
    message.encoding = "bgr8"
    message.is_bigendian = 0
    message.step = int(array.shape[1] * 3)
    message.data = array.tobytes()
    return message
