import threading
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionClient
from std_srvs.srv import Trigger
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PointStamped, PoseStamped
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import PositionIKRequest

JOINT_NAMES = [
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint',
]

TRAJ_CONTROLLER = '/scaled_joint_trajectory_controller/follow_joint_trajectory'


def _wait_for_future(future, timeout=15.0):
    event = threading.Event()
    future.add_done_callback(lambda _: event.set())
    if future.done():
        event.set()
    if not event.wait(timeout=timeout):
        return None
    return future.result()


class MotionNode(Node):
    def __init__(self):
        super().__init__('motion_node')

        self.declare_parameter('home_joints',     [0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0])
        self.declare_parameter('overview_joints', [0.0, -1.2, 1.0, -1.35, -1.5708, 0.0])
        self.declare_parameter('search_count', 3)
        self.declare_parameter('search_joints_0', [0.3,  -1.2, 1.0, -1.35, -1.5708, 0.0])
        self.declare_parameter('search_joints_1', [-0.3, -1.2, 1.0, -1.35, -1.5708, 0.0])
        self.declare_parameter('search_joints_2', [0.0,  -1.0, 1.2, -1.5,  -1.5708, 0.0])
        self.declare_parameter('approach_height', 0.10)
        self.declare_parameter('approach_quat',   [1.0, 0.0, 0.0, 0.0])
        self.declare_parameter('group_name',   'ur_manipulator')
        self.declare_parameter('base_frame',   'base_link')
        self.declare_parameter('end_effector', 'tool0')
        self.declare_parameter('move_duration_sec', 6)

        self._home_joints     = list(self.get_parameter('home_joints').value)
        self._overview_joints = list(self.get_parameter('overview_joints').value)
        self._approach_height = self.get_parameter('approach_height').value
        self._approach_quat   = list(self.get_parameter('approach_quat').value)
        search_count          = self.get_parameter('search_count').value
        self._search_joints   = [
            list(self.get_parameter(f'search_joints_{i}').value)
            for i in range(search_count)
        ]
        self._group_name     = self.get_parameter('group_name').value
        self._base_frame     = self.get_parameter('base_frame').value
        self._end_effector   = self.get_parameter('end_effector').value
        self._move_duration  = self.get_parameter('move_duration_sec').value

        self._cube_pos: dict[str, PointStamped | None] = {
            'red': None, 'green': None, 'blue': None
        }
        self._current_joints: JointState | None = None
        self._joints_lock = threading.Lock()

        cb = ReentrantCallbackGroup()

        self._traj_client = ActionClient(
            self, FollowJointTrajectory, TRAJ_CONTROLLER, callback_group=cb)

        self._ik_client = self.create_client(
            GetPositionIK, '/compute_ik', callback_group=cb)

        self.create_service(Trigger, 'robot/move_home',     self._svc_home,     callback_group=cb)
        self.create_service(Trigger, 'robot/move_overview', self._svc_overview,  callback_group=cb)
        self.create_service(Trigger, 'robot/move_to_red',    self._svc_red,      callback_group=cb)
        self.create_service(Trigger, 'robot/move_to_green',  self._svc_green,    callback_group=cb)
        self.create_service(Trigger, 'robot/move_to_blue',   self._svc_blue,     callback_group=cb)

        for i in range(len(self._search_joints)):
            self.create_service(
                Trigger, f'robot/move_to_search_{i}',
                lambda req, res, idx=i: self._svc_search(req, res, idx),
                callback_group=cb,
            )

        self.create_subscription(
            JointState, '/joint_states', self._joint_state_cb, 10, callback_group=cb)

        for color in ('red', 'green', 'blue'):
            self.create_subscription(
                PointStamped,
                f'vision/{color}_position_3d',
                lambda msg, c=color: self._position_cb(msg, c),
                10,
                callback_group=cb,
            )

        self.get_logger().info('Motion node ready')

    # ── callbacks ─────────────────────────────────────────────────────────────

    def _joint_state_cb(self, msg: JointState):
        with self._joints_lock:
            self._current_joints = msg

    def _position_cb(self, msg: PointStamped, color: str):
        self._cube_pos[color] = msg

    # ── motion helpers ────────────────────────────────────────────────────────

    def _compute_duration(self, target_joints: list[float]) -> int:
        with self._joints_lock:
            js = self._current_joints
        if js is None:
            return self._move_duration
        name_to_pos = dict(zip(js.name, js.position))
        max_delta = max(
            abs(t - name_to_pos.get(n, 0.0))
            for n, t in zip(JOINT_NAMES, target_joints)
        )
        return max(int(max_delta / 1.0) + 2, 3)

    def _execute_joints(self, joints: list[float]) -> bool:
        self.get_logger().info('_execute_joints: waiting for traj server...')
        if not self._traj_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Trajectory controller unavailable')
            return False
        self.get_logger().info('_execute_joints: server ready, sending goal')

        traj = JointTrajectory()
        traj.joint_names = JOINT_NAMES
        point = JointTrajectoryPoint()
        point.positions = list(joints)
        point.velocities = [0.0] * 6
        dur = self._compute_duration(joints)
        self.get_logger().info(f'_execute_joints: duration={dur}s, joints={[round(j,3) for j in joints]}')
        point.time_from_start = Duration(sec=dur)
        traj.points = [point]

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj

        self.get_logger().info('_execute_joints: calling send_goal_async...')
        goal_handle = _wait_for_future(self._traj_client.send_goal_async(goal))
        self.get_logger().info(f'_execute_joints: goal_handle={goal_handle}')
        if goal_handle is None:
            self.get_logger().error('send_goal_async timed out (15 s) — no response from action server')
            return False
        if not goal_handle.accepted:
            self.get_logger().error('Trajectory goal rejected')
            return False

        self.get_logger().info('_execute_joints: goal accepted, waiting for result...')
        result = _wait_for_future(goal_handle.get_result_async())
        if result is None:
            self.get_logger().error('get_result_async timed out (15 s)')
            return False
        ok = result.result.error_code == 0
        if not ok:
            self.get_logger().error(f'Trajectory failed: error_code={result.result.error_code}')
        return ok

    def _execute_pose(self, x: float, y: float, z: float) -> bool:
        if not self._ik_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('IK service unavailable')
            return False

        req = GetPositionIK.Request()
        req.ik_request.group_name = self._group_name
        req.ik_request.avoid_collisions = True
        req.ik_request.timeout.sec = 5

        pose = PoseStamped()
        pose.header.frame_id = self._base_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        q = self._approach_quat
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        req.ik_request.pose_stamped = pose

        with self._joints_lock:
            if self._current_joints is not None:
                req.ik_request.robot_state.joint_state = self._current_joints

        ik_result = _wait_for_future(self._ik_client.call_async(req))
        if ik_result.error_code.val != 1:
            self.get_logger().error(f'IK failed: error_code={ik_result.error_code.val}')
            return False

        joint_map = dict(zip(
            ik_result.solution.joint_state.name,
            ik_result.solution.joint_state.position,
        ))
        joints = [joint_map.get(n, 0.0) for n in JOINT_NAMES]
        self.get_logger().info(f'IK → {[round(j, 3) for j in joints]}')
        return self._execute_joints(joints)

    def _approach_cube(self, color: str, res: Trigger.Response) -> Trigger.Response:
        pos = self._cube_pos[color]
        if pos is None:
            res.success = False
            res.message = f'No 3D position for {color}'
            self.get_logger().warn(res.message)
            return res
        x, y, z = pos.point.x, pos.point.y, pos.point.z + self._approach_height
        self.get_logger().info(f'Approaching {color} at ({x:.3f}, {y:.3f}, {z:.3f})')
        ok = self._execute_pose(x, y, z)
        res.success = ok
        res.message = f'Approached {color}' if ok else f'Failed to approach {color}'
        return res

    # ── service callbacks ─────────────────────────────────────────────────────

    def _svc_home(self, req, res):
        self.get_logger().info('Moving to home')
        ok = self._execute_joints(self._home_joints)
        res.success = ok
        res.message = 'At home' if ok else 'Home failed'
        return res

    def _svc_overview(self, req, res):
        self.get_logger().info('Moving to overview')
        ok = self._execute_joints(self._overview_joints)
        res.success = ok
        res.message = 'At overview' if ok else 'Overview failed'
        return res

    def _svc_red(self, req, res):
        return self._approach_cube('red', res)

    def _svc_green(self, req, res):
        return self._approach_cube('green', res)

    def _svc_blue(self, req, res):
        return self._approach_cube('blue', res)

    def _svc_search(self, req, res, idx: int):
        self.get_logger().info(f'Moving to search position {idx}')
        ok = self._execute_joints(self._search_joints[idx])
        res.success = ok
        res.message = f'At search {idx}' if ok else f'Search {idx} failed'
        return res


def main(args=None):
    rclpy.init(args=args)
    node = MotionNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
