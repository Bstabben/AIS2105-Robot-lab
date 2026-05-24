import os
import glob
import yaml
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2

INTEGRATED_KEYWORDS = ('integrated', 'chicony')
_CONSECUTIVE_FAIL_LIMIT = 10


def _find_usb_camera() -> str:
    """Return the first usable camera device path, preferring /dev/v4l/by-id symlinks."""
    by_id = sorted(glob.glob('/dev/v4l/by-id/*video-index0'))
    for path in by_id:
        name = os.path.basename(path).lower()
        if not any(kw in name for kw in INTEGRATED_KEYWORDS):
            cap = cv2.VideoCapture(path, cv2.CAP_V4L2)
            if cap.isOpened():
                cap.release()
                return path

    # Fall back to /dev/video* , skip index 0 which is usually the integrated webcam
    for i in range(10):
        path = f'/dev/video{i}'
        if not os.path.exists(path):
            continue
        cap = cv2.VideoCapture(path, cv2.CAP_V4L2)
        if cap.isOpened():
            cap.release()
            return path

    raise RuntimeError(
        'No usable camera found in /dev/v4l/by-id/ or /dev/video0-9. '
        'Check that the camera is plugged in.')


def _load_camera_info(path: str) -> CameraInfo:
    """Parse a calibration.yaml produced by cameracalibrator into a CameraInfo msg."""
    with open(path) as f:
        data = yaml.safe_load(f)
    msg = CameraInfo()
    msg.width = data['image_width']
    msg.height = data['image_height']
    msg.distortion_model = data.get('distortion_model', 'plumb_bob')
    msg.k = data['camera_matrix']['data']
    msg.d = data['distortion_coefficients']['data']
    msg.r = data['rectification_matrix']['data']
    msg.p = data['projection_matrix']['data']
    return msg


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')

        self.declare_parameter('device_id', 'auto')
        self.declare_parameter('frame_rate', 30.0)
        self.declare_parameter('frame_width', 640)
        self.declare_parameter('frame_height', 480)
        self.declare_parameter('camera_frame', 'camera_link')
        # Absolute path to calibration.yaml produced by cameracalibrator.
        # Leave empty before calibration, node will warn and publish empty CameraInfo.
        self.declare_parameter('calibration_url', '')

        device_id = self.get_parameter('device_id').value
        if device_id == 'auto':
            try:
                device_id = _find_usb_camera()
                self.get_logger().info(f'Auto-detected USB camera: {device_id}')
            except RuntimeError as e:
                self.get_logger().fatal(str(e))
                raise
        frame_rate = self.get_parameter('frame_rate').value
        width = self.get_parameter('frame_width').value
        height = self.get_parameter('frame_height').value
        self._frame_id = self.get_parameter('camera_frame').value
        calibration_url = self.get_parameter('calibration_url').value

        self._camera_info = CameraInfo()
        if calibration_url:
            # Strip file:// prefix if present
            path = calibration_url.replace('file://', '')
            try:
                self._camera_info = _load_camera_info(path)
                self.get_logger().info(f'Loaded calibration from {path}')
            except Exception as e:
                self.get_logger().error(f'Failed to load calibration from {path}: {e}')
        else:
            self.get_logger().warn(
                'calibration_url not set — publishing empty CameraInfo. '
                'Run camera calibration and pass calibration_file:= to the launch file.'
            )

        self._bridge = CvBridge()
        self._pub_image = self.create_publisher(Image, 'camera/image_raw', 10)
        self._pub_info = self.create_publisher(CameraInfo, 'camera/camera_info', 10)
        self._fail_count = 0

        self._cap = cv2.VideoCapture(device_id, cv2.CAP_V4L2)
        if not self._cap.isOpened():
            self.get_logger().error(f'Failed to open camera device {device_id}')
            raise RuntimeError(f'Cannot open camera device {device_id}')

        self._cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self._cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        period = 1.0 / frame_rate
        self._timer = self.create_timer(period, self._timer_callback)
        self.get_logger().info(
            f'Camera node started: device={device_id}, '
            f'{int(self._cap.get(cv2.CAP_PROP_FRAME_WIDTH))}x'
            f'{int(self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT))} @ {frame_rate} Hz'
        )

    def _timer_callback(self):
        ret, frame = self._cap.read()
        if not ret:
            self._fail_count += 1
            if self._fail_count == 1:
                self.get_logger().warn('Failed to capture frame')
            elif self._fail_count == _CONSECUTIVE_FAIL_LIMIT:
                self.get_logger().error(
                    f'Camera read failed {_CONSECUTIVE_FAIL_LIMIT} times in a row — '
                    'check USB connection or device path.')
            return
        self._fail_count = 0

        stamp = self.get_clock().now().to_msg()

        img_msg = self._bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        img_msg.header.stamp = stamp
        img_msg.header.frame_id = self._frame_id
        self._pub_image.publish(img_msg)

        self._camera_info.header.stamp = stamp
        self._camera_info.header.frame_id = self._frame_id
        self._pub_info.publish(self._camera_info)

    def destroy_node(self):
        if self._cap.isOpened():
            self._cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
