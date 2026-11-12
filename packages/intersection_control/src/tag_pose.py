"""Where a detected AprilTag is relative to the robot.

    from tag_pose import tag_in_robot

    for det in msg.detections:              # apriltag_detector_node/detections
        x, y, theta = tag_in_robot(det)     # the tag, in the robot frame

Robot frame: origin on the wheel axle, x forward, y left. Your odometry uses
the same frame.

Tag frame: x points out of the printed face, y to the tag's left. A tag dead
ahead and facing the robot is at (d, 0, pi). config/parking_lot.yaml uses this
frame too.

The detector reports each tag in the camera's optical frame (x right, y down,
z out of the lens), using AprilTag's own tag axes (x right, y down, z into the
tag). This module converts both and adds the camera's offset from the axle.
"""

import math

# How far the camera sits ahead of the wheel axle (m).
CAMERA_X = 0.10

# How far the camera tilts down (rad). 0 treats it as level.
CAMERA_PITCH = 0.0


def tag_in_robot(det):
    """The detected tag's pose in the robot frame, as (x, y, theta).

    det is one duckietown_msgs/AprilTagDetection. theta is the direction the
    tag's printed face points, in (-pi, pi].
    """
    t = det.transform.translation
    q = det.transform.rotation

    # AprilTag's z axis, which points into the tag, in the optical frame.
    # This is the third column of the rotation matrix.
    zx = 2 * (q.x * q.z + q.w * q.y)
    zy = 2 * (q.y * q.z - q.w * q.x)
    zz = 1 - 2 * (q.x * q.x + q.y * q.y)

    x, y, _ = _optical_to_robot(t.x, t.y, t.z)
    face_x, face_y, _ = _optical_to_robot(-zx, -zy, -zz)   # out of the tag
    return x + CAMERA_X, y, math.atan2(face_y, face_x)


def _optical_to_robot(ox, oy, oz):
    """Rotate a vector from the camera's optical frame into the robot frame."""
    # For a level camera: forward is z, left is -x, up is -y.
    fwd, left, up = oz, -ox, -oy
    # Then tilt it down by CAMERA_PITCH about the robot's y axis.
    c, s = math.cos(CAMERA_PITCH), math.sin(CAMERA_PITCH)
    return c * fwd + s * up, left, c * up - s * fwd
