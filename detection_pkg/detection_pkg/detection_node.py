import json
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
import cv2
import numpy as np


# HSV ranges — each entry is a (lower, upper) pair.
# Values are [H, S, V]: H 0-180, S 0-255, V 0-255 (OpenCV convention).
DEFAULT_HSV = {
    'red':   [([  0,  60,  40], [ 10, 255, 255]),
              ([170,  60,  40], [180, 255, 255])],
    'green': [([ 35,  80,  50], [ 85, 255, 255])],
    'blue':  [([100, 150,  50], [130, 255, 255])],
}


class DetectionNode(Node):
    def __init__(self):
        super().__init__('detection_node')

        self._hsv = {}
        for color, ranges in DEFAULT_HSV.items():
            bounds = []
            for i, (lo, hi) in enumerate(ranges):
                self.declare_parameter(f'hsv.{color}.range{i}.lower', lo)
                self.declare_parameter(f'hsv.{color}.range{i}.upper', hi)
                lower = self.get_parameter(f'hsv.{color}.range{i}.lower').value
                upper = self.get_parameter(f'hsv.{color}.range{i}.upper').value
                bounds.append((np.array(lower, dtype=np.uint8),
                                np.array(upper, dtype=np.uint8)))
            self._hsv[color] = bounds

        self.declare_parameter('min_contour_area', 500)
        self.declare_parameter('publish_debug_image', True)

        self._min_area = self.get_parameter('min_contour_area').value
        self._publish_debug = self.get_parameter('publish_debug_image').value

        self._K: np.ndarray | None = None
        self._D: np.ndarray | None = None

        self._bridge = CvBridge()

        # Blob detector — more stable than raw contours for compact colour regions
        params = cv2.SimpleBlobDetector_Params()
        params.filterByArea = True
        params.minArea = float(self._min_area)
        params.maxArea = 50000.0
        params.filterByCircularity = False   # cubes are not circular
        params.filterByConvexity = True
        params.minConvexity = 0.6            # rejects fragmented / L-shaped noise
        params.filterByInertia = False
        self._blob_detector = cv2.SimpleBlobDetector_create(params)

        self._sub_info = self.create_subscription(
            CameraInfo, 'camera/camera_info', self._camera_info_cb, 1)
        self._sub = self.create_subscription(
            Image, 'camera/image_raw', self._image_callback, 1)
        self._pub_detections = self.create_publisher(String, 'vision/detections', 10)

        self._pub_position = {
            color: self.create_publisher(PointStamped, f'vision/{color}_position', 10)
            for color in DEFAULT_HSV
        }

        # Use sensor-data QoS so rqt_image_view and image_tools can subscribe
        self._pub_debug = self.create_publisher(
            Image, 'vision/debug_image', qos_profile_sensor_data)

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

        # Blur in BGR space BEFORE converting to HSV — avoids hue wrapping
        # artefacts that occur when blurring the H channel directly (especially
        # bad for red, which sits at the 0°/180° boundary).
        blurred_bgr = cv2.GaussianBlur(frame, (9, 9), 0)
        hsv = cv2.cvtColor(blurred_bgr, cv2.COLOR_BGR2HSV)

        # Morphological kernels
        k_close = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))
        k_open  = cv2.getStructuringElement(cv2.MORPH_RECT,    (3, 3))

        detections = {}
        debug_frame = frame.copy()

        for color, ranges in self._hsv.items():
            mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
            for lower, upper in ranges:
                mask |= cv2.inRange(hsv, lower, upper)

            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  k_open,  iterations=1)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k_close, iterations=2)

            # Blob detector expects dark blobs on white — invert the mask
            keypoints = self._blob_detector.detect(cv2.bitwise_not(mask))

            if keypoints:
                # Pick the largest blob
                best = max(keypoints, key=lambda k: k.size)
                cx = int(best.pt[0])
                cy = int(best.pt[1])
                r  = int(best.size / 2)
                detections[color] = {
                    'center_px': [cx, cy],
                    'bbox_px':   [cx - r, cy - r, r * 2, r * 2],
                    'area_px2':  int(np.pi * r * r),
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
                    cv2.circle(debug_frame, (cx, cy), max(r, 5), color_bgr, 2)
                    cv2.circle(debug_frame, (cx, cy), 4, color_bgr, -1)
                    cv2.putText(debug_frame, color, (cx - r, cy - r - 8),
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
