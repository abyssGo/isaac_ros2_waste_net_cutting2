import numpy as np
import pytest
from std_msgs.msg import Header

from nc_vision.image_msg_utils import bgr8_to_imgmsg


def test_bgr8_message_layout_matches_array():
    image = np.arange(4 * 5 * 3, dtype=np.uint8).reshape(4, 5, 3)
    header = Header()
    header.frame_id = "camera1_optical_frame"
    header.stamp.sec = 12
    header.stamp.nanosec = 34

    message = bgr8_to_imgmsg(image, header=header)

    assert (message.height, message.width) == (4, 5)
    assert message.encoding == "bgr8"
    assert message.step == 15
    assert message.is_bigendian == 0
    assert bytes(message.data) == image.tobytes()
    assert message.header.frame_id == "camera1_optical_frame"
    assert (message.header.stamp.sec, message.header.stamp.nanosec) == (12, 34)


def test_frame_id_override_and_non_contiguous_input():
    image = np.zeros((6, 8, 3), dtype=np.uint8)[::2, ::2]  # non-contiguous view
    message = bgr8_to_imgmsg(image, frame_id="vision_debug_dashboard")
    assert message.header.frame_id == "vision_debug_dashboard"
    assert len(message.data) == 3 * 4 * 3


@pytest.mark.parametrize(
    "bad",
    [np.zeros((4, 5), dtype=np.uint8), np.zeros((4, 5, 3), dtype=np.float32)],
)
def test_rejects_non_bgr8_arrays(bad):
    with pytest.raises(ValueError):
        bgr8_to_imgmsg(bad)
