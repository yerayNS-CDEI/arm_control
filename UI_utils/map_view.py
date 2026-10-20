#!/usr/bin/env python3
"""
2D map view for the control UI: RTAB-Map's occupancy grid with the robot,
its Nav2 footprint, Nav2's path/goal, its local costmap and the detected
walls drawn on top (an
RViz stand-in that costs a QGraphicsScene instead of an OpenGL point-cloud
renderer).

The robot pose and localization state come from LocalizationMonitor. The
other topics are subscribed only while the view is visible: rtabmap only
assembles /map while it has a subscriber, and in mapping mode it republishes
it on every update. Subscriptions are created and destroyed on the monitor's
executor thread (through a guard condition) so they never race its spin.
The grid is dropped when rtabmap stops: its latched /map would otherwise stay
drawn with nothing left to replace it.

In goal mode a press-drag-release on the map picks a pose like RViz's
"2D Goal Pose" tool (press = position, drag = heading) and emits
goal_selected; the owner publishes it. Shift + left-drag rotates the view
about its centre.

The local costmap is a small rolling window (~120x120 cells at 2 Hz) in the
odom frame; it is placed with LocalizationMonitor's map->odom, like the robot.
So is the footprint: the local costmap publishes it (with the arm-aware
dynamic footprint when that runs) already at the robot's odom pose, at its
update rate, so it can trail the arrow by a fraction of a second.
"""

import math
import threading
import time

import numpy as np
from scipy import ndimage
from PyQt5.QtCore import Qt, QPointF, QRectF, pyqtSignal
from PyQt5.QtGui import (
    QBrush, QColor, QFont, QImage, QPainter, QPainterPath, QPen, QPixmap, QPolygonF,
    QTransform,
)
from PyQt5.QtWidgets import QGraphicsItem, QGraphicsScene, QGraphicsView
from rclpy.exceptions import InvalidHandle
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import PolygonStamped
from nav_msgs.msg import OccupancyGrid, Path

from .localization_monitor import OFFLINE

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
LOCAL_COSTMAP_TOPIC = '/local_costmap/costmap'
FOOTPRINT_TOPIC = '/local_costmap/published_footprint'
FOOTPRINT_TIMEOUT = 2.0     # (s) hide the footprint when Nav2 stops publishing it

MAP_REDRAW_PERIOD = 2.0     # (s) at most one grid -> image conversion per period
ROBOT_LENGTH = 0.7          # (m) drawn robot arrow
SCENE_MARGIN = 20.0         # (m) pannable space around the map
GOAL_MIN_DRAG = 0.05        # (m) shorter drags keep the heading at 0 (+x), as in RViz
ROTATE_DEAD_ZONE = 10       # (px) around the view centre, where the drag angle is unstable
FIT_BLOCK = 1.0             # (m) fit() looks for the mapped area in blocks of this size,
FIT_MIN_KNOWN = 0.5         #     those at least this fraction known, largest connected group
FIT_MARGIN = 8              # (px) left around the fitted area

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


def _costmap_lut():
    """ARGB per costmap value as Nav2 publishes it (0 free, 1-98 cost, 99 inscribed,
    100 lethal, -1 unknown); free and unknown are transparent."""
    lut = np.zeros(256, dtype=np.uint32)
    t = (np.arange(1, 99) - 1) / 97.0                       # low cost blue -> high cost red
    r = (255 * t).astype(np.uint32)
    g = (200 * (1.0 - np.abs(2.0 * t - 1.0))).astype(np.uint32)
    b = (255 * (1.0 - t)).astype(np.uint32)
    a = (60 + 120 * t).astype(np.uint32)
    lut[1:99] = (a << 24) | (r << 16) | (g << 8) | b
    lut[99] = 0xC000BCD4                                    # inscribed: cyan
    lut[100] = 0xE06F42C1                                   # lethal: purple
    return lut


_LUT = _grid_lut()
_COSTMAP_LUT = _costmap_lut()
UNKNOWN_COLOR = QColor('#8a9099')


class MapOverlays:
    """Latest /map, /plan, Nav2 goal status, footprint and walls, filled on the monitor thread."""

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
        self.costmap = None         # latest local costmap OccupancyGrid
        self.costmap_version = 0
        self.footprint = None       # (frame_id, [(x, y)], monotonic receive time)
        self._map_enabled = True
        self._map_subscription = None
        self._costmap_enabled = True
        self._costmap_subscription = None
        self._guard = node.create_guard_condition(self._sync_subscriptions)

    def set_active(self, active):
        """Subscribe (True) or unsubscribe (False); safe from the UI thread."""
        self._active = active
        self._trigger_sync()

    def set_map_enabled(self, enabled):
        """Subscribe to /map only while it is shown; safe from the UI thread."""
        self._map_enabled = enabled
        if enabled:
            self.clear_map()    # the latched map comes again with the new subscription
        self._trigger_sync()

    def clear_map(self):
        """Forget the grid (rtabmap left, or the map is shown again); safe from any thread."""
        with self._lock:
            self.grid = None
            self.grid_version += 1

    def set_costmap_enabled(self, enabled):
        """Subscribe to the local costmap only while it is shown; safe from the UI thread."""
        self._costmap_enabled = enabled
        if enabled:
            with self._lock:    # never draw a costmap left over from before
                self.costmap = None
                self.costmap_version += 1
        self._trigger_sync()

    def _trigger_sync(self):
        try:
            self._guard.trigger()
        except InvalidHandle:
            pass  # monitor node already destroyed: the window is closing

    def _sync_subscriptions(self):
        if self._active and not self._subscriptions:
            self._subscriptions = [
                self._node.create_subscription(Path, PLAN_TOPIC, self._on_plan, QoSProfile(depth=1)),
                self._node.create_subscription(
                    GoalStatusArray, NAV_STATUS_TOPIC, self._on_nav_status, _LATCHED),
                self._node.create_subscription(
                    PolygonStamped, FOOTPRINT_TOPIC, self._on_footprint, QoSProfile(depth=1)),
            ]
            if WallArray is not None:
                self._subscriptions.append(self._node.create_subscription(
                    WallArray, WALLS_TOPIC, self._on_walls, _LATCHED))
        elif not self._active and self._subscriptions:
            for subscription in self._subscriptions:
                self._node.destroy_subscription(subscription)
            self._subscriptions = []
        want_map = self._active and self._map_enabled
        if want_map and self._map_subscription is None:
            self._map_subscription = self._node.create_subscription(
                OccupancyGrid, MAP_TOPIC, self._on_map, _LATCHED)
        elif not want_map and self._map_subscription is not None:
            self._node.destroy_subscription(self._map_subscription)
            self._map_subscription = None
        want_costmap = self._active and self._costmap_enabled
        if want_costmap and self._costmap_subscription is None:
            self._costmap_subscription = self._node.create_subscription(
                OccupancyGrid, LOCAL_COSTMAP_TOPIC, self._on_costmap, _LATCHED)
        elif not want_costmap and self._costmap_subscription is not None:
            self._node.destroy_subscription(self._costmap_subscription)
            self._costmap_subscription = None

    def _on_map(self, msg):
        with self._lock:
            self.grid = msg
            self.grid_version += 1

    def _on_costmap(self, msg):
        with self._lock:
            self.costmap = msg
            self.costmap_version += 1

    def _on_footprint(self, msg):
        points = [(p.x, p.y) for p in msg.polygon.points]
        with self._lock:
            self.footprint = (msg.header.frame_id.lstrip('/'), points, time.monotonic())

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
                'costmap': self.costmap, 'costmap_version': self.costmap_version,
                'footprint': self.footprint,
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
        # Panning is by drag; scroll bars that come and go as the view rotates or
        # zooms resize the viewport, shifting the map and spoiling fit().
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.setBackgroundBrush(QBrush(UNKNOWN_COLOR))
        self._monitor = monitor
        self._state_colors = state_colors
        self.overlays = MapOverlays(monitor.node)
        self.follow_robot = True

        self._grid_version = 0
        self._grid_drawn_at = 0.0
        self._walls_version = 0
        self._fitted = False
        self._fit_grid = None       # (cells, resolution, origin x, origin y) for fit()
        self._last_state = None     # localization state at the previous refresh()

        self.show_map = True
        self._map_item = self._scene.addPixmap(QPixmap())
        self._map_item.setTransformationMode(Qt.FastTransformation)  # crisp cells
        self._map_item.setZValue(0)
        self.show_walls = True
        self._wall_items = []

        self.show_costmap = True
        self._costmap_version = 0
        self._costmap_geometry = None   # (frame_id, resolution, origin x, top y) of the drawn costmap
        self._costmap_item = self._scene.addPixmap(QPixmap())
        self._costmap_item.setTransformationMode(Qt.FastTransformation)
        self._costmap_item.setZValue(0.5)
        self._costmap_item.hide()

        self._path_item = self._scene.addPath(self._polyline([]), self._pen('#1f6feb', 2))
        self._path_item.setZValue(2)
        self._goal_item = self._scene.addPolygon(
            self._arrow(), self._pen('#1f6feb', 2), QBrush(Qt.NoBrush))
        self._goal_item.setZValue(2)
        self._goal_item.hide()

        self.show_footprint = True
        self._footprint_item = self._scene.addPolygon(
            QPolygonF(), self._pen('#22272e', 2), QBrush(QColor(34, 39, 46, 40)))
        self._footprint_item.setZValue(2.5)
        self._footprint_item.hide()

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

        self._rotating = False
        self._rotate_from = None    # cursor angle (deg) about the view centre while rotating
        self._rotate_cursor = None  # viewport cursor to restore afterwards

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

    # ── Shift + left-drag rotates about the view centre ──

    def _cursor_angle(self, event):
        """Cursor angle (deg, clockwise on screen) about the view centre, None near it."""
        centre = self.viewport().rect().center()
        dx, dy = event.pos().x() - centre.x(), event.pos().y() - centre.y()
        if math.hypot(dx, dy) < ROTATE_DEAD_ZONE:
            return None
        return math.degrees(math.atan2(dy, dx))

    def _rotate_view(self, degrees):
        anchor = self.transformationAnchor()
        self.setTransformationAnchor(QGraphicsView.AnchorViewCenter)
        self.rotate(degrees)
        self.setTransformationAnchor(anchor)

    def mousePressEvent(self, event):
        if (event.button() == Qt.LeftButton and event.modifiers() & Qt.ShiftModifier
                and self._goal_press is None):
            self._rotating = True
            self._rotate_from = self._cursor_angle(event)
            self._rotate_cursor = self.viewport().cursor()
            self.viewport().setCursor(Qt.SizeAllCursor)
            event.accept()
            return
        if self._goal_mode and event.button() == Qt.LeftButton:
            self._goal_press = self._map_point(event)
            self._goal_preview_item.setPos(_scene(*self._goal_press))
            self._goal_preview_item.setRotation(0.0)
            self._goal_preview_item.show()
            event.accept()
            return
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event):
        if self._rotating:
            angle = self._cursor_angle(event)
            if angle is not None and self._rotate_from is not None:
                self._rotate_view((angle - self._rotate_from + 180.0) % 360.0 - 180.0)
            if angle is not None:
                self._rotate_from = angle
            event.accept()
            return
        if self._goal_press is not None:
            yaw = self._goal_yaw(*self._map_point(event))
            self._goal_preview_item.setRotation(-math.degrees(yaw))
            event.accept()
            return
        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event):
        if self._rotating and event.button() == Qt.LeftButton:
            self._rotating = False
            self.viewport().setCursor(self._rotate_cursor)
            event.accept()
            return
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
        """Fit the mapped area to the view, keeping the rotation.

        RTAB-Map's grid also spans sparse speckle (seen through windows and
        doors), so the whole grid can be twice the size of what was mapped.
        """
        points = None if self._fit_grid is None else self._mapped_area(*self._fit_grid)
        if points is None:
            rect = self._scene.itemsBoundingRect()
            if rect.isEmpty():
                return
            points = np.array([[rect.left(), rect.top()], [rect.right(), rect.bottom()],
                               [rect.left(), rect.bottom()], [rect.right(), rect.top()]])
        t = self.transform()
        scale = math.hypot(t.m11(), t.m12())
        if scale == 0.0:
            return
        c, s = t.m11() / scale, t.m12() / scale
        # Extent in the view's orientation, so a rotated map is fitted tightly.
        u = c * points[:, 0] - s * points[:, 1]
        v = s * points[:, 0] + c * points[:, 1]
        du, dv = u.max() - u.min(), v.max() - v.min()
        viewport = self.viewport().rect()
        width, height = viewport.width() - 2 * FIT_MARGIN, viewport.height() - 2 * FIT_MARGIN
        if width <= 0 or height <= 0 or (du == 0.0 and dv == 0.0):
            return
        scale = min(width / du if du > 0.0 else math.inf, height / dv if dv > 0.0 else math.inf)
        self.setTransform(QTransform(c * scale, s * scale, -s * scale, c * scale, 0.0, 0.0))
        uc, vc = (u.max() + u.min()) / 2.0, (v.max() + v.min()) / 2.0
        self.centerOn(c * uc + s * vc, -s * uc + c * vc)

    @staticmethod
    def _mapped_area(cells, res, ox, oy):
        """Scene corners of the largest well-known area of the grid, or None if there is none."""
        b = max(1, int(round(FIT_BLOCK / res)))
        h, w = cells.shape[0] // b * b, cells.shape[1] // b * b
        if h == 0 or w == 0:
            return None
        known = (cells[:h, :w] != 255).reshape(h // b, b, w // b, b).mean(axis=(1, 3))
        labels, count = ndimage.label(known >= FIT_MIN_KNOWN, structure=np.ones((3, 3)))
        if count == 0:
            return None
        largest = np.argmax(np.bincount(labels.ravel())[1:]) + 1
        rows, cols = np.nonzero(labels == largest)
        size = b * res
        x0, y0 = ox + cols * size, oy + rows * size
        xs = np.concatenate([x0, x0 + size, x0, x0 + size])
        ys = np.concatenate([y0, y0, y0 + size, y0 + size])
        return np.column_stack([xs, -ys])      # map frame -> scene (y down)

    # ── occupancy grid and walls ──

    def set_map_visible(self, visible):
        self.show_map = visible
        self.overlays.set_map_enabled(visible)
        self._map_item.setVisible(visible)

    def set_walls_visible(self, visible):
        self.show_walls = visible
        for item in self._wall_items:
            item.setVisible(visible)

    def _clear_map(self):
        """Drop the drawn grid: its rtabmap is gone and no update will replace it."""
        self.overlays.clear_map()
        self._map_item.setPixmap(QPixmap())
        self._fit_grid = None
        self._fitted = False    # fit the next session's map when it arrives

    # ── local costmap ──

    def set_costmap_visible(self, visible):
        self.show_costmap = visible
        self.overlays.set_costmap_enabled(visible)
        if not visible:
            self._costmap_item.hide()

    # ── footprint ──

    def set_footprint_visible(self, visible):
        self.show_footprint = visible
        if not visible:
            self._footprint_item.hide()

    # ── drawing ──

    def refresh(self, state):
        """Redraw from the latest data; returns a one-line summary for a status label."""
        # Only on the change to offline: rtabmap can publish its latched /map a
        # moment before the monitor sees it running, and that map must stay.
        if state == OFFLINE and self._last_state not in (None, OFFLINE):
            self._clear_map()
        self._last_state = state
        data = self.overlays.snapshot()
        self._update_grid(data['grid'], data['grid_version'])
        self._update_walls(data['walls'], data['walls_version'])
        self._update_costmap(data['costmap'], data['costmap_version'])
        self._path_item.setPath(self._polyline(data['path']))
        if data['goal'] is not None:
            gx, gy, gyaw = data['goal']
            self._goal_item.setPos(_scene(gx, gy))
            self._goal_item.setRotation(-math.degrees(gyaw))
            self._goal_item.show()
        else:
            self._goal_item.hide()
        self._update_footprint(data['footprint'])
        self._update_robot(state)

        grid = data['grid']
        parts = []
        if not self.show_map:
            parts.append('map hidden')
        elif grid is None:
            parts.append(f'waiting for {MAP_TOPIC}')
        else:
            info = grid.info
            parts.append(f'map {info.width * info.resolution:.0f}×'
                         f'{info.height * info.resolution:.0f} m @ {info.resolution:.2f} m')
        costmap = data['costmap']
        if self.show_costmap:
            if costmap is None:
                parts.append(f'waiting for {LOCAL_COSTMAP_TOPIC}')
            else:
                info = costmap.info
                parts.append(f'costmap {info.width * info.resolution:.0f}×'
                             f'{info.height * info.resolution:.0f} m')
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
        self._fit_grid = (cells, res, ox, oy)
        bounds = QRectF(ox, -(oy + h * res), w * res, h * res)
        self._scene.setSceneRect(bounds.adjusted(-SCENE_MARGIN, -SCENE_MARGIN, SCENE_MARGIN, SCENE_MARGIN))
        if not self._fitted:
            self._fitted = True
            self.fit()

    def _update_costmap(self, costmap, version):
        if not self.show_costmap or costmap is None:
            self._costmap_item.hide()
            return
        if version != self._costmap_version:
            self._costmap_version = version
            info = costmap.info
            w, h, res = info.width, info.height, info.resolution
            if w == 0 or h == 0:
                self._costmap_geometry = None
            else:
                cells = np.asarray(costmap.data, dtype=np.int8)[:w * h].view(np.uint8).reshape(h, w)
                argb = np.ascontiguousarray(_COSTMAP_LUT[cells][::-1])   # row 0 = top = max y
                image = QImage(argb.data, w, h, 4 * w, QImage.Format_ARGB32).copy()
                self._costmap_item.setPixmap(QPixmap.fromImage(image))
                self._costmap_geometry = (costmap.header.frame_id.lstrip('/'), res,
                                          info.origin.position.x, info.origin.position.y + h * res)
        if self._costmap_geometry is None:
            self._costmap_item.hide()
            return
        # Re-placed every refresh: map->odom moves with each localization match.
        frame, res, ox, top = self._costmap_geometry
        tx, ty, th = (0.0, 0.0, 0.0) if frame == 'map' else self._monitor.map_to_odom()
        c, s = math.cos(th), math.sin(th)
        # Image pixel (u, v) -> frame (ox + u res, top - v res) -> map -> scene (y down).
        self._costmap_item.setTransform(QTransform(
            c * res, -s * res, s * res, c * res,
            tx + c * ox - s * top, -(ty + s * ox + c * top)))
        self._costmap_item.show()

    def _update_footprint(self, footprint):
        if not self.show_footprint or footprint is None or time.monotonic() - footprint[2] > FOOTPRINT_TIMEOUT \
                or len(footprint[1]) < 3:
            self._footprint_item.hide()
            return
        # Re-placed every refresh, like the costmap: map->odom moves with each match.
        frame, points, _ = footprint
        tx, ty, th = (0.0, 0.0, 0.0) if frame == 'map' else self._monitor.map_to_odom()
        c, s = math.cos(th), math.sin(th)
        self._footprint_item.setPolygon(QPolygonF(
            [_scene(tx + c * x - s * y, ty + s * x + c * y) for x, y in points]))
        self._footprint_item.show()

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
            for item in (line, label):
                item.setVisible(self.show_walls)
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
