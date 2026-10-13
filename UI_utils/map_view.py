#!/usr/bin/env python3
"""
2D map view for the control UI: RTAB-Map's occupancy grid with the robot,
Nav2's path/goal and the detected walls drawn on top (an RViz stand-in that
costs a QGraphicsScene instead of an OpenGL point-cloud renderer).

The robot pose and localization state come from LocalizationMonitor. The
other topics are subscribed only while the view is visible: rtabmap only
assembles /map while it has a subscriber, and in mapping mode it republishes
it on every update. Subscriptions are created and destroyed on the monitor's
executor thread (through a guard condition) so they never race its spin.

In goal mode a press-drag-release on the map picks a pose like RViz's
"2D Goal Pose" tool (press = position, drag = heading) and emits
goal_selected; the owner publishes it.
"""

import math
import threading
import time

import numpy as np
from PyQt5.QtCore import Qt, QPointF, QRectF, pyqtSignal
from PyQt5.QtGui import (
    QBrush, QColor, QFont, QImage, QPainter, QPainterPath, QPen, QPixmap, QPolygonF,
)
from PyQt5.QtWidgets import QGraphicsItem, QGraphicsScene, QGraphicsView
from rclpy.exceptions import InvalidHandle
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from action_msgs.msg import GoalStatus, GoalStatusArray
from nav_msgs.msg import OccupancyGrid, Path

try:
    # navi_wall depends on arm_control, so this cannot be a package dependency;
    # both are built in the robot workspace, elsewhere walls are just not drawn.
    from navi_wall.msg import WallArray
except ImportError:
    WallArray = None

MAP_TOPIC = '/map'
PLAN_TOPIC = '/plan'
NAV_STATUS_TOPIC = '/navigate_to_pose/_action/status'
WALLS_TOPIC = '/wall_detection_node/walls'

MAP_REDRAW_PERIOD = 2.0     # (s) at most one grid -> image conversion per period
ROBOT_LENGTH = 0.7          # (m) drawn robot arrow
SCENE_MARGIN = 20.0         # (m) pannable space around the map
GOAL_MIN_DRAG = 0.05        # (m) shorter drags keep the heading at 0 (+x), as in RViz

_LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL)
_ACTIVE_GOAL = (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING,
                GoalStatus.STATUS_CANCELING)


def _scene(x, y):
    """Map frame (y up) -> scene (y down)."""
    return QPointF(x, -y)


def _grid_lut():
    """ARGB per occupancy value; index = value as uint8 (-1 -> 255 = unknown)."""
    lut = np.empty(256, dtype=np.uint32)
    occupancy = np.arange(101)
    shade = (240 - occupancy * 215 // 100).astype(np.uint32)   # free 240 -> occupied 25
    lut[:101] = 0xFF000000 | (shade << 16) | (shade << 8) | shade
    lut[101:] = 0xFF8A9099                                      # unknown
    return lut


_LUT = _grid_lut()
UNKNOWN_COLOR = QColor('#8a9099')


class MapOverlays:
    """Latest /map, /plan, Nav2 goal status and walls, filled on the monitor thread."""

    def __init__(self, node):
        self._node = node
        self._lock = threading.Lock()
        self._active = False
        self._subscriptions = []
        self.grid = None            # latest OccupancyGrid
        self.grid_version = 0
        self.path = []              # [(x, y)] in the map frame
        self.goal = None            # (x, y, yaw) last pose of the path
        self.nav_active = False
        self.walls = []             # [(sx, sy, ex, ey, refined, two_sided)]
        self.walls_version = 0
        self._guard = node.create_guard_condition(self._sync_subscriptions)

    def set_active(self, active):
        """Subscribe (True) or unsubscribe (False); safe from the UI thread."""
        self._active = active
        try:
            self._guard.trigger()
        except InvalidHandle:
            pass  # monitor node already destroyed: the window is closing

    def _sync_subscriptions(self):
        if self._active and not self._subscriptions:
            self._subscriptions = [
                self._node.create_subscription(OccupancyGrid, MAP_TOPIC, self._on_map, _LATCHED),
                self._node.create_subscription(Path, PLAN_TOPIC, self._on_plan, QoSProfile(depth=1)),
                self._node.create_subscription(
                    GoalStatusArray, NAV_STATUS_TOPIC, self._on_nav_status, _LATCHED),
            ]
            if WallArray is not None:
                self._subscriptions.append(self._node.create_subscription(
                    WallArray, WALLS_TOPIC, self._on_walls, _LATCHED))
        elif not self._active and self._subscriptions:
            for subscription in self._subscriptions:
                self._node.destroy_subscription(subscription)
            self._subscriptions = []

    def _on_map(self, msg):
        with self._lock:
            self.grid = msg
            self.grid_version += 1

    def _on_plan(self, msg):
        path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        goal = None
        if msg.poses:
            pose = msg.poses[-1].pose
            q = pose.orientation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            goal = (pose.position.x, pose.position.y, yaw)
        with self._lock:
            self.path = path
            self.goal = goal

    def _on_nav_status(self, msg):
        with self._lock:
            self.nav_active = any(s.status in _ACTIVE_GOAL for s in msg.status_list)

    def _on_walls(self, msg):
        walls = [(w.start.x, w.start.y, w.end.x, w.end.y, w.refined, w.two_sided)
                 for w in msg.walls]
        with self._lock:
            self.walls = walls
            self.walls_version += 1

    def snapshot(self):
        with self._lock:
            return {
                'grid': self.grid, 'grid_version': self.grid_version,
                'path': self.path if self.nav_active else [],
                'goal': self.goal if self.nav_active else None,
                'walls': self.walls, 'walls_version': self.walls_version,
            }


class MapView(QGraphicsView):
    """Zoomable map; refresh() is driven by the owner's timer while visible."""

    goal_selected = pyqtSignal(float, float, float)     # x, y, yaw in the map frame

    def __init__(self, monitor, state_colors, parent=None):
        self._scene = QGraphicsScene()
        super().__init__(self._scene, parent)
        self.setRenderHint(QPainter.Antialiasing, True)
        self.setDragMode(QGraphicsView.ScrollHandDrag)
        self.setTransformationAnchor(QGraphicsView.AnchorUnderMouse)
        self.setResizeAnchor(QGraphicsView.AnchorViewCenter)
        self.setBackgroundBrush(QBrush(UNKNOWN_COLOR))
        self._monitor = monitor
        self._state_colors = state_colors
        self.overlays = MapOverlays(monitor.node)
        self.follow_robot = True

        self._grid_version = 0
        self._grid_drawn_at = 0.0
        self._walls_version = 0
        self._fitted = False

        self._map_item = self._scene.addPixmap(QPixmap())
        self._map_item.setTransformationMode(Qt.FastTransformation)  # crisp cells
        self._map_item.setZValue(0)
        self._wall_items = []

        self._path_item = self._scene.addPath(self._polyline([]), self._pen('#1f6feb', 2))
        self._path_item.setZValue(2)
        self._goal_item = self._scene.addPolygon(
            self._arrow(), self._pen('#1f6feb', 2), QBrush(Qt.NoBrush))
        self._goal_item.setZValue(2)
        self._goal_item.hide()

        self._sigma_item = self._scene.addEllipse(QRectF(), self._pen('#8b949e', 1))
        self._sigma_item.setZValue(3)
        self._robot_item = self._scene.addPolygon(
            self._arrow(), self._pen('#22272e', 1), QBrush(QColor('#8b949e')))
        self._robot_item.setZValue(4)
        self._robot_item.hide()
        self._sigma_item.hide()

        self._goal_mode = False
        self._goal_press = None     # (x, y) in the map frame while placing a goal
        goal_pen = self._pen('#e8590c', 2)
        goal_pen.setStyle(Qt.DashLine)
        self._goal_preview_item = self._scene.addPolygon(
            self._arrow(), goal_pen, QBrush(QColor(232, 89, 12, 60)))
        self._goal_preview_item.setZValue(5)
        self._goal_preview_item.hide()

    @staticmethod
    def _pen(color, width):
        pen = QPen(QColor(color), width)
        pen.setCosmetic(True)   # constant on-screen width at any zoom
        return pen

    @staticmethod
    def _arrow():
        """Robot-sized arrow pointing along +x of its own (scene) frame."""
        l = ROBOT_LENGTH
        return QPolygonF([QPointF(l * 0.6, 0.0), QPointF(-l * 0.4, l * 0.35),
                          QPointF(-l * 0.2, 0.0), QPointF(-l * 0.4, -l * 0.35)])

    @staticmethod
    def _polyline(points):
        path = QPainterPath()
        for i, (x, y) in enumerate(points):
            if i == 0:
                path.moveTo(_scene(x, y))
            else:
                path.lineTo(_scene(x, y))
        return path

    # ── visibility gates the subscriptions ──

    def showEvent(self, event):
        self.overlays.set_active(True)
        super().showEvent(event)

    def hideEvent(self, event):
        self.overlays.set_active(False)
        super().hideEvent(event)

    # ── 2D goal pose (RViz-style press, drag for heading, release) ──

    def set_goal_mode(self, enabled):
        self._goal_mode = enabled
        self._goal_press = None
        self._goal_preview_item.hide()
        self.setDragMode(QGraphicsView.NoDrag if enabled else QGraphicsView.ScrollHandDrag)
        if enabled:
            self.viewport().setCursor(Qt.CrossCursor)
        else:
            self.viewport().unsetCursor()

    def _map_point(self, event):
        p = self.mapToScene(event.pos())
        return p.x(), -p.y()

    def _goal_yaw(self, x, y):
        px, py = self._goal_press
        if math.hypot(x - px, y - py) < GOAL_MIN_DRAG:
            return 0.0
        return math.atan2(y - py, x - px)

    def mousePressEvent(self, event):
        if self._goal_mode and event.button() == Qt.LeftButton:
            self._goal_press = self._map_point(event)
            self._goal_preview_item.setPos(_scene(*self._goal_press))
            self._goal_preview_item.setRotation(0.0)
            self._goal_preview_item.show()
            event.accept()
            return
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event):
        if self._goal_press is not None:
            yaw = self._goal_yaw(*self._map_point(event))
            self._goal_preview_item.setRotation(-math.degrees(yaw))
            event.accept()
            return
        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event):
        if self._goal_press is not None and event.button() == Qt.LeftButton:
            x, y = self._goal_press
            yaw = self._goal_yaw(*self._map_point(event))
            self._goal_press = None
            self._goal_preview_item.hide()
            event.accept()
            self.goal_selected.emit(x, y, yaw)
            return
        super().mouseReleaseEvent(event)

    def keyPressEvent(self, event):
        if self._goal_mode and event.key() == Qt.Key_Escape:
            self._goal_press = None
            self._goal_preview_item.hide()
            event.accept()
            return
        super().keyPressEvent(event)

    # ── zoom (same feel as the FSM graph) ──

    def wheelEvent(self, event):
        delta = event.angleDelta().y()
        if delta == 0:
            super().wheelEvent(event)
            return
        self.zoom(1.15 if delta > 0 else 1.0 / 1.15)
        event.accept()

    def zoom(self, factor):
        self.scale(factor, factor)

    def fit(self):
        rect = self._map_item.sceneBoundingRect()
        if rect.isEmpty():
            rect = self._scene.itemsBoundingRect()
        if not rect.isEmpty():
            self.fitInView(rect, Qt.KeepAspectRatio)

    # ── drawing ──

    def refresh(self, state):
        """Redraw from the latest data; returns a one-line summary for a status label."""
        data = self.overlays.snapshot()
        self._update_grid(data['grid'], data['grid_version'])
        self._update_walls(data['walls'], data['walls_version'])
        self._path_item.setPath(self._polyline(data['path']))
        if data['goal'] is not None:
            gx, gy, gyaw = data['goal']
            self._goal_item.setPos(_scene(gx, gy))
            self._goal_item.setRotation(-math.degrees(gyaw))
            self._goal_item.show()
        else:
            self._goal_item.hide()
        self._update_robot(state)

        grid = data['grid']
        parts = []
        if grid is None:
            parts.append(f'waiting for {MAP_TOPIC}')
        else:
            info = grid.info
            parts.append(f'map {info.width * info.resolution:.0f}×'
                         f'{info.height * info.resolution:.0f} m @ {info.resolution:.2f} m')
        if WallArray is None:
            parts.append('walls: navi_wall not built')
        else:
            parts.append(f'{len(data["walls"])} walls')
        if data['goal'] is not None:
            parts.append(f'navigating, path {len(data["path"])} poses')
        return ' · '.join(parts)

    def _update_grid(self, grid, version):
        now = time.monotonic()
        if grid is None or version == self._grid_version or now - self._grid_drawn_at < MAP_REDRAW_PERIOD:
            return
        self._grid_version = version
        self._grid_drawn_at = now
        info = grid.info
        w, h = info.width, info.height
        if w == 0 or h == 0:
            return
        cells = np.asarray(grid.data, dtype=np.int8)[:w * h].view(np.uint8).reshape(h, w)
        argb = np.ascontiguousarray(_LUT[cells][::-1])     # row 0 = top = max y
        image = QImage(argb.data, w, h, 4 * w, QImage.Format_ARGB32).copy()
        self._map_item.setPixmap(QPixmap.fromImage(image))
        res = info.resolution
        ox, oy = info.origin.position.x, info.origin.position.y
        self._map_item.setScale(res)
        self._map_item.setPos(ox, -(oy + h * res))
        bounds = QRectF(ox, -(oy + h * res), w * res, h * res)
        self._scene.setSceneRect(bounds.adjusted(-SCENE_MARGIN, -SCENE_MARGIN, SCENE_MARGIN, SCENE_MARGIN))
        if not self._fitted:
            self._fitted = True
            self.fit()

    def _update_walls(self, walls, version):
        if version == self._walls_version:
            return
        self._walls_version = version
        for item in self._wall_items:
            self._scene.removeItem(item)
        self._wall_items = []
        label_font = QFont('Sans Serif', 9, QFont.Bold)
        for i, (sx, sy, ex, ey, refined, two_sided) in enumerate(walls):
            # Same colours as the detector's RViz markers.
            color = '#d63ad6' if two_sided else ('#2ea043' if refined else '#d29922')
            line = self._scene.addLine(sx, -sy, ex, -ey, self._pen(color, 4))
            line.setZValue(1)
            label = self._scene.addSimpleText(f'W{i + 1}', label_font)
            label.setBrush(QBrush(QColor('#b8860b')))
            label.setFlag(QGraphicsItem.ItemIgnoresTransformations, True)
            label.setPos(_scene((sx + ex) / 2.0, (sy + ey) / 2.0))
            label.setZValue(1)
            self._wall_items += [line, label]

    def _update_robot(self, state):
        located = self._monitor.robot_pose()
        if located is None:
            self._robot_item.hide()
            self._sigma_item.hide()
            return
        (x, y, yaw), sigma = located
        color = QColor(self._state_colors.get(state, '#8b949e'))
        self._robot_item.setBrush(QBrush(color))
        self._robot_item.setPos(_scene(x, y))
        self._robot_item.setRotation(-math.degrees(yaw))
        self._robot_item.show()
        if sigma is not None and sigma > 0.0:
            fill = QColor(color)
            fill.setAlpha(50)
            self._sigma_item.setRect(QRectF(-sigma, -sigma, 2 * sigma, 2 * sigma))
            self._sigma_item.setPen(self._pen(color.name(), 1))
            self._sigma_item.setBrush(QBrush(fill))
            self._sigma_item.setPos(_scene(x, y))
            self._sigma_item.show()
        else:
            self._sigma_item.hide()
        if self.follow_robot and self._goal_press is None:   # keep the map still while aiming
            self.centerOn(_scene(x, y))
