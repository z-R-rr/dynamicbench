import numpy as np

from dynamicbench.data.reset_image_audit import audit_reset_image


def test_reset_audit_rejects_cube_at_previous_lift_position():
    camera = dict(
        head_position=[0, 0, 0],
        head_quaternion_wxyz=[1, 0, 0, 0],
        head_lens=dict(focal_length=10, horizontal_aperture=20),
    )
    image = np.full((200, 200, 3), 160, np.uint8)
    image[95:105, 95:105] = [240, 160, 20]
    current = audit_reset_image(image, [0, 0, -1], camera)
    assert current["passed"]
    # The same RGB is stale when the state says the cube has reset elsewhere.
    stale = audit_reset_image(image, [0.3, 0, -1], camera)
    assert not stale["passed"]
    assert stale["center_error_px"] > 25


def test_reset_audit_requires_visible_cube():
    camera = dict(
        head_position=[0, 0, 0],
        head_quaternion_wxyz=[1, 0, 0, 0],
        head_lens=dict(focal_length=10, horizontal_aperture=20),
    )
    result = audit_reset_image(np.zeros((200, 200, 3), np.uint8), [0, 0, -1], camera)
    assert not result["passed"]
    assert result["center_error_px"] is None
