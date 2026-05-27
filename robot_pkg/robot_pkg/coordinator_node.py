import json
import time
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import String
from std_srvs.srv import Trigger
from geometry_msgs.msg import PointStamped


class State(Enum):
    IDLE               = auto()
    MOVING_HOME        = auto()
    MOVING_OVERVIEW    = auto()
    WAITING_RED        = auto()
    MOVING_RED         = auto()
    HOMING_AFTER_RED   = auto()
    WAITING_GREEN      = auto()
    MOVING_GREEN       = auto()
    HOMING_AFTER_GREEN = auto()
    WAITING_BLUE       = auto()
    MOVING_BLUE        = auto()
    HOMING_AFTER_BLUE  = auto()
    SEARCHING          = auto()
    ALERT              = auto()
    DONE               = auto()


CUBE_COLORS = ('red', 'green', 'blue')

# States where the robot is stationary and detections should be accepted.
_WAITING_STATES = {State.WAITING_RED, State.WAITING_GREEN, State.WAITING_BLUE}


class CoordinatorNode(Node):
    """
    State machine that orchestrates the full cube-pointing task.

    The sequence starts by calling the /robot/start service (std_srvs/Trigger).

    If a cube is not detected within detection_timeout seconds, the robot tries
    each search position in turn.  After all search positions are exhausted it
    enters ALERT and stops.

    3D position updates are only accepted while the robot is stationary
    (WAITING_* states or after arriving at a search position).
    """

    def __init__(self):
        super().__init__('coordinator_node')

        self.declare_parameter('detection_timeout', 5.0)
        self.declare_parameter('search_timeout',    5.0)
        self.declare_parameter('search_count',      3)

        self._detection_timeout = self.get_parameter('detection_timeout').value
        self._search_timeout    = self.get_parameter('search_timeout').value
        self._search_count      = self.get_parameter('search_count').value

        self._state               = State.IDLE
        self._pending             = None        # current async service future
        self._detections          = {}          # latest vision/detections payload
        self._has_3d              = {c: False for c in CUBE_COLORS}
        self._wait_start          = 0.0
        self._search_idx          = 0
        self._missing_color       = ''
        self._accepting_detections = False      # only True while robot is stationary
        self._search_arrived      = False       # True once trajectory to search pos finished

        cb = ReentrantCallbackGroup()

        self._sub_detections = self.create_subscription(
            String, 'vision/detections', self._detections_cb, 10)

        for color in CUBE_COLORS:
            self.create_subscription(
                PointStamped,
                f'vision/{color}_position_3d',
                lambda msg, c=color: self._on_3d_pos(msg, c),
                10,
                callback_group=cb,
            )

        # Service clients for motion_node
        self._cli = {
            'home':     self._make_client('robot/move_home',     cb),
            'overview': self._make_client('robot/move_overview',  cb),
            'red':      self._make_client('robot/move_to_red',    cb),
            'green':    self._make_client('robot/move_to_green',  cb),
            'blue':     self._make_client('robot/move_to_blue',   cb),
        }
        for i in range(self._search_count):
            self._cli[f'search_{i}'] = self._make_client(
                f'robot/move_to_search_{i}', cb)

        # Start trigger
        self.create_service(
            Trigger, 'robot/start', self._start_cb, callback_group=cb)

        # State machine timer (ticks at 10 Hz)
        self._timer = self.create_timer(0.1, self._tick, callback_group=cb)

        self.get_logger().info('Coordinator ready — call /robot/start to begin')

    # helpers

    def _make_client(self, name: str, cb_group):
        client = self.create_client(Trigger, name, callback_group=cb_group)
        return client

    def _call(self, key: str):
        """Fire an async service call and store the future in self._pending."""
        client = self._cli[key]
        if not client.service_is_ready():
            self.get_logger().warn(f'Service {key} not available yet')
        self._pending = client.call_async(Trigger.Request())

    def _pending_done(self) -> bool:
        return self._pending is not None and self._pending.done()

    def _pending_ok(self) -> bool:
        return self._pending_done() and self._pending.result().success

    def _start_timer(self):
        self._wait_start = time.monotonic()

    def _elapsed(self) -> float:
        return time.monotonic() - self._wait_start

    # callbacks

    def _on_3d_pos(self, msg: PointStamped, color: str):
        # Ignore detections while the robot is moving to avoid false positives
        # from camera shake or nearby objects picked up mid-trajectory.
        if not self._accepting_detections:
            return
        if not self._has_3d[color]:
            self.get_logger().info(f'3D position received for {color}')
            self._has_3d[color] = True

    def _detections_cb(self, msg: String):
        try:
            payload = json.loads(msg.data)
            self._detections = payload.get('detections', {})
        except json.JSONDecodeError:
            pass

    def _start_cb(self, req, res):
        if self._state != State.IDLE:
            res.success = False
            res.message = f'Already running (state: {self._state.name})'
            return res
        self.get_logger().info('Starting task sequence')
        self._transition(State.MOVING_HOME)
        self._call('home')
        res.success = True
        res.message = 'Started'
        return res

    # state machine

    def _transition(self, new_state: State):
        self.get_logger().info(f'{self._state.name} → {new_state.name}')
        self._state = new_state
        self._pending = None
        self._search_arrived = False
        # Accept detections only in stationary waiting states;
        # SEARCHING manages its own flag once the robot arrives.
        self._accepting_detections = new_state in _WAITING_STATES

    def _tick(self):
        s = self._state

        if s == State.IDLE:
            pass  # waiting for /robot/start

        elif s == State.MOVING_HOME:
            if self._pending_done():
                self._transition(State.MOVING_OVERVIEW)
                self._call('overview')

        elif s == State.MOVING_OVERVIEW:
            if self._pending_done():
                self._has_3d['red'] = False
                self._transition(State.WAITING_RED)
                self._start_timer()

        elif s == State.WAITING_RED:
            if self._has_3d['red']:
                self._transition(State.MOVING_RED)
                self._call('red')
            elif self._elapsed() > self._detection_timeout:
                self._missing_color = 'red'
                self.get_logger().warning('Timeout — no 3D position for red — starting search')
                self._search_idx = 0
                self._transition(State.SEARCHING)
                self._call(f'search_{self._search_idx}')

        elif s == State.MOVING_RED:
            if self._pending_done():
                if self._pending_ok():
                    self._transition(State.HOMING_AFTER_RED)
                    self._call('home')
                else:
                    self._missing_color = 'red'
                    self._search_idx = 0
                    self._transition(State.SEARCHING)
                    self._call(f'search_{self._search_idx}')

        elif s == State.HOMING_AFTER_RED:
            if self._pending_done():
                self._has_3d['green'] = False
                self._transition(State.WAITING_GREEN)
                self._start_timer()

        elif s == State.WAITING_GREEN:
            if self._has_3d['green']:
                self._transition(State.MOVING_GREEN)
                self._call('green')
            elif self._elapsed() > self._detection_timeout:
                self._missing_color = 'green'
                self.get_logger().warning('Timeout — no 3D position for green — starting search')
                self._search_idx = 0
                self._transition(State.SEARCHING)
                self._call(f'search_{self._search_idx}')

        elif s == State.MOVING_GREEN:
            if self._pending_done():
                if self._pending_ok():
                    self._transition(State.HOMING_AFTER_GREEN)
                    self._call('home')
                else:
                    self._missing_color = 'green'
                    self._search_idx = 0
                    self._transition(State.SEARCHING)
                    self._call(f'search_{self._search_idx}')

        elif s == State.HOMING_AFTER_GREEN:
            if self._pending_done():
                self._has_3d['blue'] = False
                self._transition(State.WAITING_BLUE)
                self._start_timer()

        elif s == State.WAITING_BLUE:
            if self._has_3d['blue']:
                self._transition(State.MOVING_BLUE)
                self._call('blue')
            elif self._elapsed() > self._detection_timeout:
                self._missing_color = 'blue'
                self.get_logger().warning('Timeout — no 3D position for blue — starting search')
                self._search_idx = 0
                self._transition(State.SEARCHING)
                self._call(f'search_{self._search_idx}')

        elif s == State.MOVING_BLUE:
            if self._pending_done():
                if self._pending_ok():
                    self._transition(State.HOMING_AFTER_BLUE)
                    self._call('home')
                else:
                    self._missing_color = 'blue'
                    self._search_idx = 0
                    self._transition(State.SEARCHING)
                    self._call(f'search_{self._search_idx}')

        elif s == State.HOMING_AFTER_BLUE:
            if self._pending_done():
                self._transition(State.DONE)

        elif s == State.SEARCHING:
            if not self._search_arrived:
                # Phase 1: waiting for the trajectory to the search position to finish
                if self._pending_done():
                    self._search_arrived = True
                    self._accepting_detections = True   # robot is now stationary
                    self._start_timer()
                    self.get_logger().info(
                        f'Arrived at search position {self._search_idx} — '
                        f'dwelling {self._search_timeout:.0f} s')
            else:
                # Phase 2: stationary dwell — wait for detection or timeout
                if self._has_3d[self._missing_color]:
                    self.get_logger().info(
                        f'Got 3D position for {self._missing_color} at search '
                        f'position {self._search_idx}')
                    color_states = {
                        'red':   State.MOVING_RED,
                        'green': State.MOVING_GREEN,
                        'blue':  State.MOVING_BLUE,
                    }
                    self._transition(color_states[self._missing_color])
                    self._call(self._missing_color)
                elif self._elapsed() > self._search_timeout:
                    self._search_idx += 1
                    if self._search_idx >= self._search_count:
                        self._transition(State.ALERT)
                    else:
                        self.get_logger().info(
                            f'Moving to search position {self._search_idx}')
                        self._accepting_detections = False
                        self._search_arrived = False
                        self._call(f'search_{self._search_idx}')

        elif s == State.ALERT:
            self.get_logger().error(
                f'Could not find {self._missing_color} cube after all search '
                f'positions. Stopping.',
                throttle_duration_sec=10.0,
            )

        elif s == State.DONE:
            self.get_logger().info(
                'Task complete — pointed at all three cubes.',
                throttle_duration_sec=30.0,
            )


def main(args=None):
    rclpy.init(args=args)
    node = CoordinatorNode()
    executor = MultiThreadedExecutor()
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
