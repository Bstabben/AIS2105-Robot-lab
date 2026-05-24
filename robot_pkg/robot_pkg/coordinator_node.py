import json
import time
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import String
from std_srvs.srv import Trigger


class State(Enum):
    IDLE          = auto()
    MOVING_HOME   = auto()
    MOVING_OVERVIEW = auto()
    WAITING_DETECT  = auto()
    MOVING_RED   = auto()
    MOVING_GREEN = auto()
    MOVING_BLUE  = auto()
    SEARCHING     = auto()
    ALERT         = auto()
    DONE          = auto()


CUBE_COLORS = ('red', 'green', 'blue')


class CoordinatorNode(Node):
    """
    State machine that orchestrates the full cube-pointing task.

    The sequence starts by calling the /robot/start service (std_srvs/Trigger).

    If a cube is not detected within detection_timeout seconds, the robot tries
    each search position in turn.  After all search positions are exhausted it
    enters ALERT and stops.
    """

    def __init__(self):
        super().__init__('coordinator_node')

        self.declare_parameter('detection_timeout', 5.0)
        self.declare_parameter('search_timeout',    4.0)
        self.declare_parameter('search_count',      3)

        self._detection_timeout = self.get_parameter('detection_timeout').value
        self._search_timeout    = self.get_parameter('search_timeout').value
        self._search_count      = self.get_parameter('search_count').value

        self._state         = State.IDLE
        self._pending       = None        # current async service future
        self._detections    = {}          # latest vision/detections payload
        self._wait_start    = 0.0         # time we entered current wait state
        self._search_idx    = 0           # which search position we're trying
        self._missing_color = ''          # cube that triggered search

        cb = ReentrantCallbackGroup()

        self._sub_detections = self.create_subscription(
            String, 'vision/detections', self._detections_cb, 10)

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

    def _detected(self, color: str) -> bool:
        return color in self._detections

    def _all_detected(self) -> bool:
        return all(self._detected(c) for c in CUBE_COLORS)

    def _start_timer(self):
        self._wait_start = time.monotonic()

    def _elapsed(self) -> float:
        return time.monotonic() - self._wait_start

    # callbacks

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
                self._transition(State.WAITING_DETECT)
                self._start_timer()

        elif s == State.WAITING_DETECT:
            if self._all_detected():
                self._transition(State.MOVING_RED)
                self._call('red')
            elif self._elapsed() > self._detection_timeout:
                missing = [c for c in CUBE_COLORS if not self._detected(c)]
                self._missing_color = missing[0]
                self.get_logger().warning(
                    f'Timeout waiting for {", ".join(missing)} — starting search')
                self._search_idx = 0
                self._transition(State.SEARCHING)
                self._call(f'search_{self._search_idx}')

        elif s == State.MOVING_RED:
            if self._pending_done():
                if self._pending_ok():
                    self._transition(State.MOVING_GREEN)
                    self._call('green')
                else:
                    self._missing_color = 'red'
                    self._search_idx = 0
                    self._transition(State.SEARCHING)
                    self._call(f'search_{self._search_idx}')

        elif s == State.MOVING_GREEN:
            if self._pending_done():
                if self._pending_ok():
                    self._transition(State.MOVING_BLUE)
                    self._call('blue')
                else:
                    self._missing_color = 'green'
                    self._search_idx = 0
                    self._transition(State.SEARCHING)
                    self._call(f'search_{self._search_idx}')

        elif s == State.MOVING_BLUE:
            if self._pending_done():
                if self._pending_ok():
                    self._transition(State.DONE)
                else:
                    self._missing_color = 'blue'
                    self._search_idx = 0
                    self._transition(State.SEARCHING)
                    self._call(f'search_{self._search_idx}')

        elif s == State.SEARCHING:
            if self._pending_done():
                # Arrived at search position, wait for detection
                if self._detected(self._missing_color):
                    self.get_logger().info(
                        f'Found {self._missing_color} at search position '
                        f'{self._search_idx}')
                    _resume = {
                        'red':   (State.MOVING_RED,   'red'),
                        'green': (State.MOVING_GREEN, 'green'),
                        'blue':  (State.MOVING_BLUE,  'blue'),
                    }
                    next_state, color_key = _resume[self._missing_color]
                    self._transition(next_state)
                    self._call(color_key)
                elif self._elapsed() > self._search_timeout:
                    self._search_idx += 1
                    if self._search_idx >= self._search_count:
                        self._transition(State.ALERT)
                    else:
                        self.get_logger().info(
                            f'Trying search position {self._search_idx}')
                        self._start_timer()
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
