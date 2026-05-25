from time import time, sleep

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import PointStamped
import tf2_ros
import tf2_geometry_msgs  # noqa: F401 — registers PointStamped transform support

COLORS = ('red', 'green', 'blue')


class TransformNode(Node):
    """
    Projects pixel-space cube detections into the robot base frame.

    Intrinsics are read from camera/camera_info (published by camera_node)

    For each colour the node:
      1. Receives a PointStamped in camera_optical_link frame (x=col, y=row, z=0).
      2. Builds a unit ray through the pinhole camera model using live CameraInfo.
      3. Looks up the live TF from camera_link to base_link.
      4. Intersects the ray with the table plane (z = table_z in base_link).
      5. Publishes the result as a PointStamped on vision/<color>_position_3d.
    """

    def __init__(self):
        super().__init__('transform_node')

        # table_z: height of the table surface in base_link frame (metres).
        self.declare_parameter('table_z', 0.0)
        self.declare_parameter('camera_frame', 'camera_link')
        self.declare_parameter('base_frame', 'base_link')

        self._table_z = self.get_parameter('table_z').value
        self._camera_frame = self.get_parameter('camera_frame').value
        self._base_frame = self.get_parameter('base_frame').value

        # Intrinsics are populated once the first CameraInfo arrives
        self._fx: float | None = None
        self._fy: float | None = None
        self._cx: float | None = None
        self._cy: float | None = None

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self.get_logger().info('Waiting for TF chain to stabilize...')
        sleep(2.0)

        self._sub_info = self.create_subscription(
            CameraInfo, 'camera/camera_info', self._camera_info_callback, 1)

        self._subs = {}
        self._pubs = {}
        for color in COLORS:
            self._subs[color] = self.create_subscription(
                PointStamped,
                f'vision/{color}_position',
                lambda msg, c=color: self._pixel_callback(msg, c),
                10,
            )
            self._pubs[color] = self.create_publisher(
                PointStamped,
                f'vision/{color}_position_3d',
                10,
            )

        self.get_logger().info(
            f'Transform node started — waiting for camera_info '
            f'(table_z={self._table_z:.3f} m)'
        )

        self.get_logger().info('Waiting for TF to stabilize...')
        time.sleep(1.0)  # Give UR driver time to publish base_link→tool0

    def _camera_info_callback(self, msg: CameraInfo):
        if self._fx is not None:
            return  # already initialised
        K = msg.k  # 3x3 intrinsic matrix
        if K[0] == 0.0:
            self.get_logger().warn(
                'Received CameraInfo with fx=0 — calibration not loaded yet. '
                'Pass calibration_url to camera_node to fix this.',
                throttle_duration_sec=5.0,
            )
            return
        self._fx, self._cx = K[0], K[2]
        self._fy, self._cy = K[4], K[5]
        self.get_logger().info(
            f'Camera intrinsics loaded: '
            f'fx={self._fx:.1f} fy={self._fy:.1f} '
            f'cx={self._cx:.1f} cy={self._cy:.1f}'
        )

    def _pixel_callback(self, msg: PointStamped, color: str):
        if self._fx is None:
            self.get_logger().warn(
                'Camera intrinsics not yet received — dropping detection',
                throttle_duration_sec=5.0,
            )
            return

        px = msg.point.x
        py = msg.point.y
        ray_cam = np.array([
            (px - self._cx) / self._fx,
            (py - self._cy) / self._fy,
            1.0,
        ])

        try:
            tf = self._tf_buffer.lookup_transform(
                self._base_frame,
                self._camera_frame,
                msg.header.stamp,
                timeout=rclpy.duration.Duration(seconds=0.5),
            )
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(
                f'TF lookup failed for {color}: {e}',
                throttle_duration_sec=5.0,
            )
            return

        t = tf.transform.translation
        q = tf.transform.rotation
        origin = np.array([t.x, t.y, t.z])
        R = self._quat_to_matrix(q.x, q.y, q.z, q.w)

        ray_base = R @ ray_cam

        if abs(ray_base[2]) < 1e-6:
            self.get_logger().warn(
                f'Ray for {color} is parallel to table plane — skipping')
            return
        s = (self._table_z - origin[2]) / ray_base[2]
        if s < 0:
            self.get_logger().warn(
                f'Table intersection for {color} is behind the camera — skipping')
            return

        point_base = origin + s * ray_base

        out = PointStamped()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self._base_frame
        out.point.x = float(point_base[0])
        out.point.y = float(point_base[1])
        out.point.z = float(point_base[2])
        self._pubs[color].publish(out)

    @staticmethod
    def _quat_to_matrix(x, y, z, w) -> np.ndarray:
        return np.array([
            [1 - 2*(y*y + z*z),     2*(x*y - z*w),     2*(x*z + y*w)],
            [    2*(x*y + z*w), 1 - 2*(x*x + z*z),     2*(y*z - x*w)],
            [    2*(x*z - y*w),     2*(y*z + x*w), 1 - 2*(x*x + y*y)],
        ])


def main(args=None):
    rclpy.init(args=args)
    node = TransformNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
