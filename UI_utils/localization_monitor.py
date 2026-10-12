#!/usr/bin/env python3
"""
RTAB-Map localization health monitor for the control UI (replaces watching RViz).

Runs its own node on a background executor thread, so the 10-20 Hz odometry
and localization topics never compete with the UI's one-callback-per-tick
spin. The Qt side only calls snapshot() from a timer.

Signals used (checked against rtabmap_ros CoreWrapper / OdometryROS):
- /rtabmap/localization_pose is published on every map update while someone
  subscribes. Its covariance is 9999 until the first match, is reset to the
  match covariance on each loop closure / proximity detection, and grows with
  odometry covariance in between. So sigma is the confidence and a sigma that
  does not grow marks a correction.
- /rtabmap/odom carries pose covariance 9999 when ICP odometry is lost.
- Mem/IncrementalMemory on /rtabmap/rtabmap tells mapping from localization
  (in mapping mode localization_pose is published too, but means nothing).
"""

import math
import threading
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters

RTABMAP_NAMESPACE = '/rtabmap'
RTABMAP_NODE = 'rtabmap'
LOCALIZATION_POSE_TOPIC = '/rtabmap/localization_pose'
ODOM_TOPIC = '/rtabmap/odom'

BAD_COVARIANCE = 9999.0     # rtabmap's "unknown / lost" covariance marker
SIGMA_OK = 0.25             # (m) above this the pose is shown as uncertain
DRIFT_DISTANCE = 2.0        # (m) driven on odometry without a correction before warning
ODOM_TIMEOUT = 2.0          # (s) no odometry for this long -> odometry missing
MAX_ODOM_STEP = 1.0         # (m) larger odometry jumps are resets, not motion
GRAPH_POLL_PERIOD = 1.0     # (s) rtabmap node presence check
MODE_POLL_PERIOD = 5.0      # (s) mapping/localization mode can change at runtime

# States returned by snapshot(), worst first after 'offline'.
OFFLINE = 'offline'
MAPPING = 'mapping'
NO_ODOM = 'no_odom'
ODOM_LOST = 'odom_lost'
NOT_LOCALIZED = 'not_localized'
UNCERTAIN = 'uncertain'
LOCALIZED = 'localized'


class LocalizationMonitor:
    """Tracks RTAB-Map localization health; thread-safe snapshot() for the UI."""

    def __init__(self, node_name='robot_control_ui_localization'):
        self._lock = threading.Lock()
        self._reset_rtabmap_state()
        self._rtabmap_present = False
        self._last_odom_time = None
        self._odom_lost = False
        self._last_odom_xy = None
        self._odom_distance = 0.0   # cumulative distance driven on odometry
        self._last_mode_poll = 0.0
        self._mode_future = None

        self.node = rclpy.create_node(node_name)
        self.node.create_subscription(
            PoseWithCovarianceStamped, LOCALIZATION_POSE_TOPIC,
            self._on_localization_pose,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        self.node.create_subscription(
            Odometry, ODOM_TOPIC, self._on_odom,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self._param_client = self.node.create_client(
            GetParameters, f'{RTABMAP_NAMESPACE}/{RTABMAP_NODE}/get_parameters')
        self.node.create_timer(GRAPH_POLL_PERIOD, self._poll_rtabmap)

        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self.node)
        self._thread = threading.Thread(
            target=self._spin, name='localization_monitor', daemon=True)
        self._thread.start()

    def _spin(self):
        try:
            self._executor.spin()
        except Exception:
            pass  # context shut down while the UI closes

    def stop(self):
        self._executor.shutdown(timeout_sec=1.0)
        self._thread.join(timeout=1.0)
        self.node.destroy_node()

    def _reset_rtabmap_state(self):
        """Forget everything learned from an rtabmap instance (it restarted or left)."""
        self._mode = None               # 'mapping' / 'localization' / None (unknown)
        self._sigma = None              # None until the first match
        self._last_correction_time = None
        self._correction_distance = 0.0

    # ── callbacks (executor thread) ──

    def _on_odom(self, msg):
        cov = msg.pose.covariance
        with self._lock:
            self._last_odom_time = time.monotonic()
            if cov[0] >= BAD_COVARIANCE:
                self._odom_lost = True
                self._last_odom_xy = None
                return
            self._odom_lost = False
            xy = (msg.pose.pose.position.x, msg.pose.pose.position.y)
            if self._last_odom_xy is not None:
                step = math.hypot(xy[0] - self._last_odom_xy[0], xy[1] - self._last_odom_xy[1])
                if step < MAX_ODOM_STEP:
                    self._odom_distance += step
            self._last_odom_xy = xy

    def _on_localization_pose(self, msg):
        cov = msg.pose.covariance
        with self._lock:
            if cov[0] >= BAD_COVARIANCE:
                self._sigma = None
                return
            sigma = math.sqrt(max(cov[0], cov[7], 0.0))
            # Between matches rtabmap only adds odometry covariance, so a sigma
            # that did not grow is a match (or the robot stood still, where
            # resetting the drift distance changes nothing).
            if self._sigma is None or sigma <= self._sigma:
                self._last_correction_time = time.monotonic()
                self._correction_distance = self._odom_distance
            self._sigma = sigma

    def _poll_rtabmap(self):
        present = any(
            name == RTABMAP_NODE and ns == RTABMAP_NAMESPACE
            for name, ns in self.node.get_node_names_and_namespaces())
        with self._lock:
            if not present and self._rtabmap_present:
                self._reset_rtabmap_state()
            self._rtabmap_present = present
        if not present or self._mode_future is not None:
            return
        now = time.monotonic()
        if now - self._last_mode_poll < MODE_POLL_PERIOD or not self._param_client.service_is_ready():
            return
        self._last_mode_poll = now
        self._mode_future = self._param_client.call_async(
            GetParameters.Request(names=['Mem/IncrementalMemory']))
        self._mode_future.add_done_callback(self._on_mode)

    def _on_mode(self, future):
        self._mode_future = None
        try:
            value = future.result().values[0]
        except Exception:
            return
        if value.type == ParameterType.PARAMETER_BOOL:
            incremental = value.bool_value
        elif value.type == ParameterType.PARAMETER_STRING:
            incremental = value.string_value.strip().lower() == 'true'
        else:
            return
        with self._lock:
            self._mode = 'mapping' if incremental else 'localization'

    # ── UI thread ──

    def snapshot(self):
        """Return (state, detail) where state is one of the module constants."""
        now = time.monotonic()
        with self._lock:
            if not self._rtabmap_present:
                return OFFLINE, 'RTAB-Map not running'
            if self._mode == 'mapping':
                return MAPPING, 'building the map (SLAM)'
            if self._last_odom_time is None or now - self._last_odom_time > ODOM_TIMEOUT:
                return NO_ODOM, f'no messages on {ODOM_TOPIC}'
            if self._odom_lost:
                return ODOM_LOST, 'ICP odometry lost, drive back to a feature-rich area'
            if self._sigma is None:
                return NOT_LOCALIZED, 'waiting for the first match against the map'
            driven = self._odom_distance - self._correction_distance
            age = now - self._last_correction_time
            detail = f'σ {self._sigma:.2f} m · last correction {age:.0f} s / {driven:.1f} m ago'
            if self._sigma > SIGMA_OK or driven > DRIFT_DISTANCE:
                return UNCERTAIN, detail
            return LOCALIZED, detail
