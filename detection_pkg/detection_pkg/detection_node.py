import json
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
import cv2
import numpy as np


# RGB ranges — each entry is a single [lower, upper] pair.
# Values are [R, G, B], each 0-255.
DEFAULT_RGB = {
    'red':   [([150,   0,   0], [255, 100, 100])],
    'green': [([  0,  80,   0], [120, 255, 120])],
    'blue':  [([  0,   0,  90], [120, 120, 255])],
}


class DetectionNode(Node):
    def __init__(self):
        super().__init__('detection_node')

        # RGB parameters (declared per color so they are tunable via yaml)
        self._rgb = {}
        for color, ranges in DEFAULT_RGB.items():
            bounds = []
            for i, (lo, hi) in enumerate(ranges):
                self.declare_parameter(f'rgb.{color}.range{i}.lower', lo)
                self.declare_parameter(f'rgb.{color}.range{i}.upper', hi)
                lower = self.get_parameter(f'rgb.{color}.range{i}.lower').value
                upper = self.get_parameter(f'rgb.{color}.range{i}.upper').value
                bounds.append((np.array(lower, dtype=np.uint8),
                                np.array(upper, dtype=np.uint8)))
            self._rgb[color] = bounds

        self.declare_parameter('min_contour_area', 500)
        self.declare_parameter('publish_debug_image', True)

        self._min_area = self.get_parameter('min_contour_area').value
        self._publish_debug = self.get_parameter('publish_debug_image').value

        self._K: np.ndarray | None = None
        self._D: np.ndarray | None = None

        self._bridge = CvBridge()

        self._sub_info = self.create_subscription(
            CameraInfo, 'camera/camera_info', self._camera_info_cb, 1)
        self._sub = self.create_subscription(
            Image, 'camera/image_raw', self._image_callback, 1)
        self._pub_detections = self.create_publisher(String, 'vision/detections', 10)

        self._pub_position = {
            color: self.create_publisher(PointStamped, f'vision/{color}_position', 10)
            for color in DEFAULT_RGB
        }

        self._pub_debug = self.create_publisher(Image, 'vision/debug_image', 10)

        self.get_logger().info('Detection node started')

    def _camera_info_cb(self, msg: CameraInfo):
        if self._K is not None or msg.k[0] == 0.0:
            return
        self._K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self._D = np.array(msg.d, dtype=np.float64)
        self.get_logger().info('Camera intrinsics loaded for undistortion')

    def _image_callback(self, msg: Image):
        try:
            self._process(msg)
        except Exception as e:
            self.get_logger().error(f'Detection callback error: {e}', throttle_duration_sec=2.0)

    def _process(self, msg: Image):
        frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        if self._K is not None and self._D is not None:
            frame = cv2.undistort(frame, self._K, self._D)

        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        blurred = cv2.GaussianBlur(rgb, (9, 9), 0)

        # Morphological kernels (built once, reused per color)
        k_close  = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))
        k_open   = cv2.getStructuringElement(cv2.MORPH_RECT,    (3, 3))
        k_dilate = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))

        detections = {}
        debug_frame = frame.copy()

        for color, ranges in self._rgb.items():
            mask = np.zeros(rgb.shape[:2], dtype=np.uint8)
            for lower, upper in ranges:
                mask |= cv2.inRange(blurred, lower, upper)

            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE,  k_close,  iterations=2)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,   k_open,   iterations=1)
            mask = cv2.morphologyEx(mask, cv2.MORPH_DILATE, k_dilate, iterations=1)

            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            best = None
            best_area = self._min_area
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area > best_area:
                    best_area = area
                    best = cnt

            if best is not None:
                x, y, w, h = cv2.boundingRect(best)
                cx = int(x + w / 2)
                cy = int(y + h / 2)
                detections[color] = {
                    'center_px': [cx, cy],
                    'bbox_px':   [x, y, w, h],
                    'area_px2':  int(best_area),
                }

                pt = PointStamped()
                pt.header = msg.header
                pt.point.x = float(cx)
                pt.point.y = float(cy)
                pt.point.z = 0.0
                self._pub_position[color].publish(pt)

                if self._publish_debug:
                    color_bgr = {'red': (0, 0, 255),
                                 'green': (0, 255, 0),
                                 'blue': (255, 0, 0)}[color]
                    cv2.rectangle(debug_frame, (x, y), (x + w, y + h), color_bgr, 2)
                    cv2.circle(debug_frame, (cx, cy), 5, color_bgr, -1)
                    cv2.putText(debug_frame, color, (x, y - 8),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, color_bgr, 2)

        payload = {
            'stamp': {
                'sec': msg.header.stamp.sec,
                'nanosec': msg.header.stamp.nanosec,
            },
            'detections': detections,
        }
        out_msg = String()
        out_msg.data = json.dumps(payload)
        self._pub_detections.publish(out_msg)

        debug_msg = self._bridge.cv2_to_imgmsg(debug_frame, encoding='bgr8')
        debug_msg.header = msg.header
        self._pub_debug.publish(debug_msg)


def main(args=None):
    rclpy.init(args=args)
    node = DetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
