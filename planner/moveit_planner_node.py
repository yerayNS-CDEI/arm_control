#!/usr/bin/env python3

import math
import time
from collections import deque
from typing import List, Optional, Sequence, Tuple

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from builtin_interfaces.msg import Duration as DurationMsg
from geometry_msgs.msg import Point, Pose, PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    CollisionObject,
    Constraints,
    JointConstraint,
    OrientationConstraint,
    PlanningScene,
    PositionConstraint,
    RobotState,
)
from moveit_msgs.srv import ApplyPlanningScene, GetPositionIK
from sensor_msgs.msg import JointState
from shape_msgs.msg import Mesh, MeshTriangle, SolidPrimitive
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray


class MoveItPlannerNode(Node):
    WALL_OBJECT_PREFIX = 'scan_wall_panel_'
    BASE_COLUMN_OBJECT_ID = 'mobile_base_column'
    BASE_FOOTPRINT_OBJECT_ID = 'mobile_base_footprint'
    PILZ_PIPELINE_ID = 'pilz_industrial_motion_planner'
    OMPL_PIPELINE_ID = 'move_group'
    # APS runs 2 parallel RRTConnect planners and continuously shortcutting the best
    # path within the planning_time budget, producing far shorter trajectories than
    # a single RRTConnect run while keeping the same collision-aware guarantees.
    OMPL_POSE_PLANNER_ID = 'AnytimePathShorteningkConfigDefault'

    def __init__(self):
        super().__init__('moveit_planner_node')

        self.declare_parameter('group_name', 'arm_manipulator')
        self.declare_parameter('end_effector_link', 'arm_tool0')
        self.declare_parameter('planning_frame', 'arm_base')
        self.declare_parameter('mode', 'arm')
        self.declare_parameter('planning_pipeline', 'pilz_industrial_motion_planner')
        self.declare_parameter('pose_planner_id', 'PTP')
        self.declare_parameter('joint_planner_id', 'PTP')
        self.declare_parameter(
            'arm_joint_names',
            [
                'arm_shoulder_pan_joint',
                'arm_shoulder_lift_joint',
                'arm_elbow_joint',
                'arm_wrist_1_joint',
                'arm_wrist_2_joint',
                'arm_wrist_3_joint',
            ],
        )
        self.declare_parameter('planning_time', 5.0)
        self.declare_parameter('planning_attempts', 5)
        self.declare_parameter('position_tolerance', 0.002)
        self.declare_parameter('orientation_tolerance', 0.02)
        self.declare_parameter('max_velocity_scaling', 0.1)
        self.declare_parameter('max_acceleration_scaling', 0.1)
        # z_min = -1.5 so OMPL can plan paths that pass below the arm mount.
        # A UR10e mounted on a mobile base routinely sweeps through negative-z
        # arm_base territory; clamping to 0.0 prevented the OMPL fallback planner
        # from finding valid joint-space paths for many scan configurations.
        self.declare_parameter('workspace_min_corner', [-1.6, -1.6, -1.5])
        self.declare_parameter('workspace_max_corner', [1.6, 1.6, 1.8])
        self.declare_parameter('enable_planning_scene_obstacles', True)
        self.declare_parameter('enable_wall_scene_sync', True)
        self.declare_parameter('wall_marker_topic', '/wall_discretization/markers')
        self.declare_parameter('wall_panel_thickness', 0.04)
        self.declare_parameter('enable_base_collision', True)
        self.declare_parameter('enable_base_column_collision', True)
        self.declare_parameter('base_column_radius', 0.3)
        self.declare_parameter('base_column_z_limits', [-1.112, -0.02])
        self.declare_parameter('enable_base_footprint_collision', True)
        self.declare_parameter(
            'base_footprint_xy',
            [-0.2747, 0.7544, 0.3478, 0.1323, -0.1328, -0.3487, -0.7553, 0.2733],
        )
        self.declare_parameter('base_footprint_z_limits', [-1.112, -0.02])

        self.group_name = self.get_parameter('group_name').value
        self.end_effector_link = self.get_parameter('end_effector_link').value
        self.planning_frame = self.get_parameter('planning_frame').value
        self.mode = str(self.get_parameter('mode').value).strip().lower()
        self.planning_pipeline = str(self.get_parameter('planning_pipeline').value).strip()
        self.pose_planner_id = str(self.get_parameter('pose_planner_id').value).strip()
        self.joint_planner_id = str(self.get_parameter('joint_planner_id').value).strip()
        self.arm_joint_names = list(self.get_parameter('arm_joint_names').value)
        self.planning_time = float(self.get_parameter('planning_time').value)
        self.planning_attempts = int(self.get_parameter('planning_attempts').value)
        self.position_tolerance = float(self.get_parameter('position_tolerance').value)
        self.orientation_tolerance = float(self.get_parameter('orientation_tolerance').value)
        self.max_velocity_scaling = float(self.get_parameter('max_velocity_scaling').value)
        self.max_acceleration_scaling = float(self.get_parameter('max_acceleration_scaling').value)
        self.workspace_min_corner = list(self.get_parameter('workspace_min_corner').value)
        self.workspace_max_corner = list(self.get_parameter('workspace_max_corner').value)
        self.enable_planning_scene_obstacles = self._get_bool_parameter(
            'enable_planning_scene_obstacles'
        )
        self.enable_wall_scene_sync = self._get_bool_parameter('enable_wall_scene_sync')
        self.wall_marker_topic = str(self.get_parameter('wall_marker_topic').value)
        self.wall_panel_thickness = float(self.get_parameter('wall_panel_thickness').value)
        self.enable_base_collision = self._get_bool_parameter('enable_base_collision')
        self.enable_base_column_collision = self._get_bool_parameter(
            'enable_base_column_collision'
        )
        self.base_column_radius = float(self.get_parameter('base_column_radius').value)
        self.base_column_z_limits = list(self.get_parameter('base_column_z_limits').value)
        self.enable_base_footprint_collision = self._get_bool_parameter(
            'enable_base_footprint_collision'
        )
        self.base_footprint_xy = self._pairs_from_flat_list(
            list(self.get_parameter('base_footprint_xy').value)
        )
        self.base_footprint_z_limits = list(
            self.get_parameter('base_footprint_z_limits').value
        )

        self.get_logger().info(
            f"MoveIt planner configuration: mode='{self.mode}', "
            f"pipeline='{self.planning_pipeline or 'default'}', "
            f"pose_planner_id='{self.pose_planner_id or 'default'}', "
            f"joint_planner_id='{self.joint_planner_id or 'default'}', "
            f"enable_base_collision={self.enable_base_collision}"
        )

        if self.mode == 'full':
            if self.enable_base_collision:
                self.get_logger().info(
                    'Full mode detected with enable_base_collision=True; '
                    'disabling legacy synthetic mobile-base collision objects '
                    'because the full robot URDF already contains the mounted base geometry.'
                )
                self.enable_base_collision = False
                self.enable_base_column_collision = False
                self.enable_base_footprint_collision = False
            else:
                self.get_logger().info(
                    'Full mode detected with enable_base_collision already False; '
                    'using URDF geometry for collision checking (no synthetic objects).'
                )
        else:
            self.get_logger().info(
                f"Arm-only mode detected; enable_base_collision={self.enable_base_collision}"
            )

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.goal_queue = deque()
        self.current_goal_handle = None
        self.execution_complete = True
        # `execution_succeeded` drives the /execution_status topic: it is True only
        # when the most recent MoveIt goal actually finished with SUCCESS.  This
        # distinguishes "execution is finished" (which `execution_complete` tracks
        # internally) from "execution succeeded" (what downstream consumers want).
        # A failure leaves `execution_succeeded` False so /execution_status never
        # transitions True for that goal; downstream listens to
        # /planner/goal_failed for the failure signal.
        self.execution_succeeded = False
        self.prev_status = False
        self.emergency_stop = False
        self._wall_panels = []
        self._active_wall_object_ids = set()
        self._scene_dirty = self.enable_planning_scene_obstacles
        self._scene_request_in_flight = False
        self._scene_request_started_at = None
        self._last_logged_wall_panel_count = None

        self._action_client = ActionClient(self, MoveGroup, 'move_action')
        self._planning_scene_client = self.create_client(ApplyPlanningScene, 'apply_planning_scene')
        self._compute_ik_client = self.create_client(GetPositionIK, 'compute_ik')
        self._current_joint_state: Optional[JointState] = None

        self.create_subscription(PoseStamped, '/arm/goal_pose', self.goal_callback, 10)
        self.create_subscription(PoseStamped, '/arm/goal_pose_ptp', self.goal_ptp_callback, 10)
        self.create_subscription(PoseStamped, '/arm/goal_pose_lin', self.goal_lin_callback, 10)
        self.create_subscription(PoseStamped, '/arm/goal_pose_ompl', self.goal_ompl_callback, 10)
        self.create_subscription(JointState, '/arm/joint_goal', self.joint_goal_callback, 10)
        self.create_subscription(JointState, '/arm/joint_goal_ptp', self.joint_goal_ptp_callback, 10)
        self.create_subscription(JointState, '/arm/joint_goal_aps', self.joint_goal_aps_callback, 10)
        self.create_subscription(PoseStamped, '/arm/goal_pose_aps', self.goal_aps_callback, 10)
        self.create_subscription(JointState, 'joint_states', self.current_joint_state_callback, 10)
        self.create_subscription(JointState, '/joint_states', self.current_joint_state_callback, 10)
        self.create_subscription(JointState, '/arm/joint_states', self.current_joint_state_callback, 10)
        self.create_subscription(JointState, '/moveit_joint_states', self.current_joint_state_callback, 10)
        self.create_subscription(Bool, 'emergency_stop', self.emergency_callback, qos)
        self.create_subscription(Bool, '/arm/emergency_stop', self.emergency_callback, qos)
        if self.enable_planning_scene_obstacles and self.enable_wall_scene_sync:
            wall_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            self.create_subscription(
                MarkerArray,
                self.wall_marker_topic,
                self.wall_markers_callback,
                wall_qos,
            )

        self.status_pub = self.create_publisher(Bool, 'execution_status', 10)
        self.goal_failed_pub = self.create_publisher(Bool, '/planner/goal_failed', 10)
        self.create_service(Trigger, 'emergency_stop', self.handle_emergency_stop)

        self.create_timer(0.1, self.timer_callback)
        self.get_logger().info(
            f"MoveIt planner ready for group '{self.group_name}' using end effector "
            f"'{self.end_effector_link}'."
        )

    def goal_callback(self, msg: PoseStamped):
        self.enqueue_pose_goal(msg)

    def goal_ptp_callback(self, msg: PoseStamped):
        self.enqueue_pose_goal(
            msg,
            pipeline_id=self.PILZ_PIPELINE_ID,
            planner_id='PTP',
            source_label='PTP pose',
        )

    def goal_lin_callback(self, msg: PoseStamped):
        self.enqueue_pose_goal(
            msg,
            pipeline_id=self.PILZ_PIPELINE_ID,
            planner_id='LIN',
            source_label='LIN pose',
        )

    def goal_ompl_callback(self, msg: PoseStamped):
        self.enqueue_pose_goal(
            msg,
            pipeline_id=self.OMPL_PIPELINE_ID,
            planner_id=self.OMPL_POSE_PLANNER_ID,
            source_label='OMPL pose',
        )

    def joint_goal_callback(self, msg: JointState):
        self.enqueue_joint_goal(msg)

    def joint_goal_ptp_callback(self, msg: JointState):
        self.enqueue_joint_goal(
            msg,
            pipeline_id=self.PILZ_PIPELINE_ID,
            planner_id='PTP',
            source_label='PTP joint',
        )

    def joint_goal_aps_callback(self, msg: JointState):
        self.enqueue_joint_goal(
            msg,
            pipeline_id=self.OMPL_PIPELINE_ID,
            planner_id='AnytimePathShorteningkConfigDefault',
            source_label='APS joint',
        )

    def goal_aps_callback(self, msg: PoseStamped):
        self.enqueue_pose_goal(
            msg,
            pipeline_id=self.OMPL_PIPELINE_ID,
            planner_id='AnytimePathShorteningkConfigDefault',
            source_label='APS pose',
        )

    def enqueue_pose_goal(
        self,
        msg: PoseStamped,
        pipeline_id: Optional[str] = None,
        planner_id: Optional[str] = None,
        source_label: str = 'default pose',
    ):
        self.goal_queue.append(('pose', msg, pipeline_id, planner_id, source_label))
        frame_id = msg.header.frame_id or self.planning_frame
        self.get_logger().info(
            f"Queued {source_label} MoveIt goal in frame '{frame_id}' at "
            f"({msg.pose.position.x:.3f}, {msg.pose.position.y:.3f}, {msg.pose.position.z:.3f})."
        )

    def enqueue_joint_goal(
        self,
        msg: JointState,
        pipeline_id: Optional[str] = None,
        planner_id: Optional[str] = None,
        source_label: str = 'default joint',
    ):
        if len(msg.name) != len(msg.position):
            self.get_logger().error(
                'Ignoring joint goal: joint name and position arrays have different lengths.'
            )
            return

        if not msg.name:
            self.get_logger().error('Ignoring joint goal: no joint names were provided.')
            return

        self.goal_queue.append(('joint', msg, pipeline_id, planner_id, source_label))
        joint_summary = ', '.join(
            f"{joint_name}={position:.3f}"
            for joint_name, position in zip(msg.name, msg.position)
        )
        self.get_logger().info(f"Queued {source_label} MoveIt joint goal: {joint_summary}.")

    def emergency_callback(self, msg: Bool):
        self.emergency_stop = bool(msg.data)
        if self.emergency_stop:
            self.cancel_active_goal('Emergency stop topic received.')

    def current_joint_state_callback(self, msg: JointState):
        if not msg.name or len(msg.name) != len(msg.position):
            return
        self._current_joint_state = msg

    def handle_emergency_stop(self, request, response):
        del request
        if self.current_goal_handle is None:
            response.success = False
            response.message = 'No active MoveIt goal to cancel.'
            return response

        self.cancel_active_goal('Emergency stop service called.')
        response.success = True
        response.message = 'Active MoveIt goal cancel requested.'
        return response

    def timer_callback(self):
        status_msg = Bool()
        status_msg.data = self.execution_succeeded
        if status_msg.data != self.prev_status:
            self.status_pub.publish(status_msg)
            self.prev_status = status_msg.data

        self.maybe_apply_planning_scene()

        if self.execution_complete and self.goal_queue and not self.emergency_stop:
            if not self._action_client.server_is_ready():
                self.get_logger().warn('Waiting for MoveIt move_action server...')
                return
            goal_type, goal_msg, pipeline_id, planner_id, source_label = self.goal_queue.popleft()
            if goal_type == 'joint':
                self.send_move_group_joint_goal(
                    goal_msg,
                    pipeline_id=pipeline_id,
                    planner_id=planner_id,
                    source_label=source_label,
                )
            else:
                self.send_move_group_goal(
                    goal_msg,
                    pipeline_id=pipeline_id,
                    planner_id=planner_id,
                    source_label=source_label,
                )

    def send_move_group_goal(
        self,
        goal_pose: PoseStamped,
        pipeline_id: Optional[str] = None,
        planner_id: Optional[str] = None,
        source_label: str = 'default pose',
        use_seeded_ik: bool = True,
    ):
        planner_name = planner_id if planner_id is not None else self.pose_planner_id
        pipeline_name = pipeline_id if pipeline_id is not None else self.planning_pipeline
        if use_seeded_ik and self.should_seed_pose_goal_with_ik(planner_name, pipeline_name):
            if self.try_seed_pose_goal_with_ik(
                goal_pose,
                pipeline_id=pipeline_id,
                planner_id=planner_id,
                source_label=source_label,
            ):
                return

        self.dispatch_pose_goal(
            goal_pose,
            pipeline_id=pipeline_id,
            planner_id=planner_id,
            source_label=source_label,
        )

    def dispatch_pose_goal(
        self,
        goal_pose: PoseStamped,
        pipeline_id: Optional[str] = None,
        planner_id: Optional[str] = None,
        source_label: str = 'default pose',
    ):

        self.execution_complete = False
        self.execution_succeeded = False

        goal_msg = MoveGroup.Goal()
        self.configure_request_planner(
            goal_msg.request,
            goal_type='pose',
            pipeline_id=pipeline_id,
            planner_id=planner_id,
        )
        goal_msg.request.group_name = self.group_name
        # Pilz (LIN/PTP) is deterministic: retrying planning produces the same result.
        # Only OMPL benefits from multiple attempts (different random seeds each time).
        is_pilz = (goal_msg.request.pipeline_id == self.PILZ_PIPELINE_ID)
        goal_msg.request.num_planning_attempts = 1 if is_pilz else self.planning_attempts
        goal_msg.request.allowed_planning_time = self.planning_time
        goal_msg.request.max_velocity_scaling_factor = self.max_velocity_scaling
        goal_msg.request.max_acceleration_scaling_factor = self.max_acceleration_scaling

        frame_id = goal_pose.header.frame_id or self.planning_frame
        goal_msg.request.workspace_parameters.header.frame_id = frame_id
        goal_msg.request.workspace_parameters.min_corner.x = float(self.workspace_min_corner[0])
        goal_msg.request.workspace_parameters.min_corner.y = float(self.workspace_min_corner[1])
        goal_msg.request.workspace_parameters.min_corner.z = float(self.workspace_min_corner[2])
        goal_msg.request.workspace_parameters.max_corner.x = float(self.workspace_max_corner[0])
        goal_msg.request.workspace_parameters.max_corner.y = float(self.workspace_max_corner[1])
        goal_msg.request.workspace_parameters.max_corner.z = float(self.workspace_max_corner[2])

        goal_msg.request.goal_constraints = [self.build_goal_constraints(goal_pose.pose, frame_id)]

        goal_msg.planning_options.plan_only = False
        goal_msg.planning_options.look_around = False
        # Pilz failures at planning time are deterministic — replanning won't change the
        # outcome and only adds latency. OMPL benefits from replanning on execution failure.
        goal_msg.planning_options.replan = not is_pilz
        goal_msg.planning_options.replan_attempts = 0 if is_pilz else 3
        goal_msg.planning_options.replan_delay = 0.2

        self.goal_failed_pub.publish(Bool(data=False))
        self.get_logger().info(
            f"Sending {source_label} MoveIt planning+execution goal "
            f"(pipeline='{goal_msg.request.pipeline_id or 'default'}', "
            f"planner='{goal_msg.request.planner_id or 'default'}')."
        )
        future = self._action_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    def should_seed_pose_goal_with_ik(
        self,
        planner_id: Optional[str],
        pipeline_id: Optional[str],
    ) -> bool:
        planner_name = (planner_id or '').strip().upper()
        pipeline_name = (pipeline_id or '').strip()
        # PTP needs a joint goal — resolve pose -> IK before sending to Pilz.
        if planner_name == 'PTP' and pipeline_name == self.PILZ_PIPELINE_ID:
            return True
        # APS over OMPL: when the goal pose has tight orientation constraints,
        # random IK sampling at the goal can find only colliding solutions and the
        # planner fails with "Unable to sample valid states for goal tree".
        # Seeded IK (started from the current joint state) picks a known-valid
        # solution; planning then runs collision-aware in joint space.
        if planner_name == 'ANYTIMEPATHSHORTENINGKCONFIGDEFAULT' \
                and pipeline_name == self.OMPL_PIPELINE_ID:
            return True
        return False

    def try_seed_pose_goal_with_ik(
        self,
        goal_pose: PoseStamped,
        pipeline_id: Optional[str],
        planner_id: Optional[str],
        source_label: str,
    ) -> bool:
        if self._current_joint_state is None:
            self.get_logger().warn(
                f'No current joint state available to seed IK for {source_label}; '
                'falling back to direct pose planning.'
            )
            return False
        if not self._compute_ik_client.service_is_ready():
            self.get_logger().warn(
                f'compute_ik service is not ready for {source_label}; '
                'falling back to direct pose planning.'
            )
            return False

        request = GetPositionIK.Request()
        request.ik_request.group_name = self.group_name
        request.ik_request.ik_link_name = self.end_effector_link
        request.ik_request.pose_stamped = goal_pose
        request.ik_request.avoid_collisions = True
        request.ik_request.timeout = DurationMsg(sec=0, nanosec=300_000_000)

        robot_state = RobotState()
        robot_state.joint_state = self._current_joint_state
        robot_state.is_diff = False
        request.ik_request.robot_state = robot_state

        self.execution_complete = False
        self.execution_succeeded = False
        self.goal_failed_pub.publish(Bool(data=False))
        self.get_logger().info(
            f"Resolving {source_label} pose goal to a seeded IK solution "
            f"before joint-space planning."
        )
        future = self._compute_ik_client.call_async(request)
        future.add_done_callback(
            lambda fut: self.ik_response_callback(
                fut,
                goal_pose=goal_pose,
                pipeline_id=pipeline_id,
                planner_id=planner_id,
                source_label=source_label,
            )
        )
        return True

    def ik_response_callback(
        self,
        future,
        *,
        goal_pose: PoseStamped,
        pipeline_id: Optional[str],
        planner_id: Optional[str],
        source_label: str,
    ):
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().warn(
                f'Seeded IK request for {source_label} raised an exception: {exc}. '
                'Falling back to direct pose planning.'
            )
            self.send_move_group_goal_without_seed(
                goal_pose,
                pipeline_id=pipeline_id,
                planner_id=planner_id,
                source_label=source_label,
            )
            return

        if response is None or response.error_code.val != response.error_code.SUCCESS:
            error_code = response.error_code.val if response is not None else 'unknown'
            self.get_logger().warn(
                f'Seeded IK request for {source_label} failed with error code {error_code}; '
                'falling back to direct pose planning.'
            )
            self.send_move_group_goal_without_seed(
                goal_pose,
                pipeline_id=pipeline_id,
                planner_id=planner_id,
                source_label=source_label,
            )
            return

        joint_goal = self.joint_goal_from_robot_state(response.solution)
        if joint_goal is None:
            self.get_logger().warn(
                f'Seeded IK solution for {source_label} did not contain the expected arm joints; '
                'falling back to direct pose planning.'
            )
            self.send_move_group_goal_without_seed(
                goal_pose,
                pipeline_id=pipeline_id,
                planner_id=planner_id,
                source_label=source_label,
            )
            return

        self.get_logger().info(
            f'Using seeded IK solution for {source_label}; '
            f'forwarding as joint-space goal to the selected planner.'
        )
        self.send_move_group_joint_goal(
            joint_goal,
            pipeline_id=pipeline_id,
            planner_id=planner_id,
            source_label=f'{source_label} (seeded IK)',
        )

    def send_move_group_goal_without_seed(
        self,
        goal_pose: PoseStamped,
        pipeline_id: Optional[str],
        planner_id: Optional[str],
        source_label: str,
    ):
        self.send_move_group_goal(
            goal_pose,
            pipeline_id=pipeline_id,
            planner_id=planner_id,
            source_label=source_label,
            use_seeded_ik=False,
        )

    def joint_goal_from_robot_state(self, robot_state: RobotState) -> Optional[JointState]:
        joint_state = robot_state.joint_state
        if not joint_state.name or len(joint_state.name) != len(joint_state.position):
            return None

        positions_by_name = {
            name: float(position)
            for name, position in zip(joint_state.name, joint_state.position)
        }
        if any(name not in positions_by_name for name in self.arm_joint_names):
            return None

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.arm_joint_names)
        msg.position = [positions_by_name[name] for name in self.arm_joint_names]
        return msg

    def send_move_group_joint_goal(
        self,
        joint_goal: JointState,
        pipeline_id: Optional[str] = None,
        planner_id: Optional[str] = None,
        source_label: str = 'default joint',
    ):
        self.execution_complete = False
        self.execution_succeeded = False

        goal_msg = MoveGroup.Goal()
        self.configure_request_planner(
            goal_msg.request,
            goal_type='joint',
            pipeline_id=pipeline_id,
            planner_id=planner_id,
        )
        goal_msg.request.group_name = self.group_name
        is_pilz = (goal_msg.request.pipeline_id == self.PILZ_PIPELINE_ID)
        goal_msg.request.num_planning_attempts = 1 if is_pilz else self.planning_attempts
        goal_msg.request.allowed_planning_time = self.planning_time
        goal_msg.request.max_velocity_scaling_factor = self.max_velocity_scaling
        goal_msg.request.max_acceleration_scaling_factor = self.max_acceleration_scaling
        goal_msg.request.goal_constraints = [self.build_joint_goal_constraints(joint_goal)]

        goal_msg.planning_options.plan_only = False
        goal_msg.planning_options.look_around = False
        goal_msg.planning_options.replan = not is_pilz
        goal_msg.planning_options.replan_attempts = 0 if is_pilz else 3
        goal_msg.planning_options.replan_delay = 0.2

        self.goal_failed_pub.publish(Bool(data=False))
        self.get_logger().info(
            f"Sending {source_label} MoveIt joint-space planning+execution goal "
            f"(pipeline='{goal_msg.request.pipeline_id or 'default'}', "
            f"planner='{goal_msg.request.planner_id or 'default'}')."
        )
        future = self._action_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    def configure_request_planner(
        self,
        request,
        goal_type: str,
        pipeline_id: Optional[str] = None,
        planner_id: Optional[str] = None,
    ):
        selected_pipeline_id = pipeline_id if pipeline_id is not None else self.planning_pipeline
        if selected_pipeline_id:
            request.pipeline_id = selected_pipeline_id

        if planner_id is None:
            planner_id = self.pose_planner_id if goal_type == 'pose' else self.joint_planner_id
        if planner_id:
            request.planner_id = planner_id

    def build_goal_constraints(self, pose: Pose, frame_id: str) -> Constraints:
        constraints = Constraints()

        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = frame_id
        position_constraint.link_name = self.end_effector_link
        position_constraint.weight = 1.0

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.SPHERE
        primitive.dimensions = [self.position_tolerance]
        position_constraint.constraint_region.primitives = [primitive]

        primitive_pose = Pose()
        primitive_pose.position = pose.position
        primitive_pose.orientation.w = 1.0
        position_constraint.constraint_region.primitive_poses = [primitive_pose]

        orientation_constraint = OrientationConstraint()
        orientation_constraint.header.frame_id = frame_id
        orientation_constraint.link_name = self.end_effector_link
        orientation_constraint.orientation = pose.orientation
        orientation_constraint.absolute_x_axis_tolerance = self.orientation_tolerance
        orientation_constraint.absolute_y_axis_tolerance = self.orientation_tolerance
        orientation_constraint.absolute_z_axis_tolerance = self.orientation_tolerance
        orientation_constraint.weight = 1.0

        constraints.position_constraints = [position_constraint]
        constraints.orientation_constraints = [orientation_constraint]
        return constraints

    def build_joint_goal_constraints(self, joint_goal: JointState) -> Constraints:
        constraints = Constraints()
        constraints.joint_constraints = []

        for joint_name, position in zip(joint_goal.name, joint_goal.position):
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = joint_name
            joint_constraint.position = float(position)
            joint_constraint.tolerance_above = 0.001
            joint_constraint.tolerance_below = 0.001
            joint_constraint.weight = 1.0
            constraints.joint_constraints.append(joint_constraint)

        return constraints

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error('MoveIt goal was rejected.')
            self.current_goal_handle = None
            self.execution_complete = True
            self.execution_succeeded = False
            self.goal_failed_pub.publish(Bool(data=True))
            return

        self.current_goal_handle = goal_handle
        self.get_logger().info('MoveIt goal accepted.')
        goal_handle.get_result_async().add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result()
        if result is None:
            self.get_logger().error('MoveIt result future returned no result.')
            self.goal_failed_pub.publish(Bool(data=True))
            self.execution_complete = True
            self.execution_succeeded = False
            self.current_goal_handle = None
            return

        moveit_result = result.result
        error_code = moveit_result.error_code.val

        if error_code == moveit_result.error_code.SUCCESS:
            self.get_logger().info(
                f'MoveIt execution succeeded in {moveit_result.planning_time:.3f}s planning time.'
            )
            self.goal_failed_pub.publish(Bool(data=False))
            self.execution_succeeded = True
        else:
            # Dump the current joint state on failure so we can tell whether the
            # planner failed because the START state is invalid (planning_time
            # increase won't help) or because the PATH is just hard to find
            # (more time would help).
            current_js_str = "unavailable"
            if self._current_joint_state is not None:
                pairs = []
                for n, p in zip(
                    self._current_joint_state.name,
                    self._current_joint_state.position,
                ):
                    if n in self.arm_joint_names:
                        pairs.append(f"{n}={p:.3f}")
                current_js_str = ", ".join(pairs) if pairs else "no arm joints"
            self.get_logger().error(
                f'MoveIt execution failed with error code {error_code}. '
                f'Current joint state at failure: [{current_js_str}].'
            )
            self.goal_failed_pub.publish(Bool(data=True))
            self.execution_succeeded = False

        self.execution_complete = True
        self.current_goal_handle = None

    def cancel_active_goal(self, reason: str):
        if self.current_goal_handle is None:
            self.get_logger().warn(f'{reason} No active MoveIt goal to cancel.')
            return

        self.get_logger().warn(f'{reason} Requesting cancel on active MoveIt goal...')
        cancel_future = self.current_goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(self.cancel_done_callback)

    def cancel_done_callback(self, future):
        del future
        self.execution_complete = True
        self.execution_succeeded = False
        self.current_goal_handle = None
        self.goal_failed_pub.publish(Bool(data=True))
        self.get_logger().warn('MoveIt goal cancel request sent.')

    def wall_markers_callback(self, msg: MarkerArray):
        panels = []
        for marker in msg.markers:
            if marker.action == Marker.DELETEALL:
                continue
            if marker.ns != 'panel_outline' or marker.type != Marker.LINE_STRIP:
                continue

            panel = self._panel_from_outline_marker(marker)
            if panel is not None:
                panels.append(panel)

        self._wall_panels = panels
        self._scene_dirty = True

        panel_count = len(panels)
        if panel_count != self._last_logged_wall_panel_count:
            self.get_logger().info(
                f"Captured {panel_count} discretized wall panel(s) from '{self.wall_marker_topic}' "
                'for MoveIt collision checking.'
            )
            self._last_logged_wall_panel_count = panel_count

    def maybe_apply_planning_scene(self):
        if not self.enable_planning_scene_obstacles:
            return
        if not self._scene_dirty:
            return
        if self._scene_request_in_flight:
            if (
                self._scene_request_started_at is not None
                and (time.monotonic() - self._scene_request_started_at) > 2.0
            ):
                self._scene_request_in_flight = False
                self._scene_request_started_at = None
                self.get_logger().warn(
                    'Timed out waiting for ApplyPlanningScene response; retrying scene sync.'
                )
            return
        if not self._action_client.server_is_ready():
            return
        if not self._planning_scene_client.service_is_ready():
            return

        request = ApplyPlanningScene.Request()
        request.scene = self.build_planning_scene_diff()

        self._scene_request_in_flight = True
        self._scene_request_started_at = time.monotonic()
        future = self._planning_scene_client.call_async(request)
        future.add_done_callback(self.apply_planning_scene_done_callback)

    def apply_planning_scene_done_callback(self, future):
        self._scene_request_in_flight = False
        self._scene_request_started_at = None

        try:
            response = future.result()
        except Exception as exc:
            self._scene_dirty = True
            self.get_logger().error(f'Failed to apply MoveIt PlanningScene diff: {exc}')
            return

        if response is None or not response.success:
            self._scene_dirty = True
            self.get_logger().error('MoveIt rejected the PlanningScene diff.')
            return

        self._scene_dirty = False
        if self.enable_base_collision:
            obstacle_summary = 'and legacy mobile-base obstacles'
        else:
            obstacle_summary = 'and no synthetic mobile-base obstacles'
        self.get_logger().info(
            f'PlanningScene updated with {len(self._wall_panels)} wall panel(s) '
            f'{obstacle_summary}.'
        )

    def build_planning_scene_diff(self) -> PlanningScene:
        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True

        collision_objects = []

        for object_id in sorted(self._active_wall_object_ids):
            remove_object = CollisionObject()
            remove_object.id = object_id
            remove_object.header.frame_id = self.planning_frame
            remove_object.operation = CollisionObject.REMOVE
            collision_objects.append(remove_object)

        new_wall_ids = set()
        if self.enable_wall_scene_sync:
            for idx, panel in enumerate(self._wall_panels):
                object_id = f'{self.WALL_OBJECT_PREFIX}{idx}'
                collision_objects.append(self.make_wall_panel_collision_object(object_id, panel))
                new_wall_ids.add(object_id)
        self._active_wall_object_ids = new_wall_ids

        if self.enable_base_collision:
            if self.enable_base_column_collision:
                collision_objects.append(self.make_base_column_collision_object())
            if self.enable_base_footprint_collision:
                footprint_object = self.make_base_footprint_collision_object()
                if footprint_object is not None:
                    collision_objects.append(footprint_object)

        scene.world.collision_objects = collision_objects
        return scene

    def make_wall_panel_collision_object(self, object_id: str, panel: dict) -> CollisionObject:
        collision_object = CollisionObject()
        collision_object.id = object_id
        collision_object.header.frame_id = panel['frame_id']
        collision_object.operation = CollisionObject.ADD

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [
            float(panel['length']),
            float(max(self.wall_panel_thickness, 0.001)),
            float(panel['height']),
        ]

        pose = Pose()
        pose.position.x = float(panel['center'][0])
        pose.position.y = float(panel['center'][1])
        pose.position.z = float(panel['center'][2])
        pose.orientation.z = float(math.sin(panel['yaw'] / 2.0))
        pose.orientation.w = float(math.cos(panel['yaw'] / 2.0))

        collision_object.primitives = [primitive]
        collision_object.primitive_poses = [pose]
        return collision_object

    def make_base_column_collision_object(self) -> CollisionObject:
        z_min, z_max = self._extract_z_limits(self.base_column_z_limits, default=(-1.112, -0.02))

        collision_object = CollisionObject()
        collision_object.id = self.BASE_COLUMN_OBJECT_ID
        collision_object.header.frame_id = self.planning_frame
        collision_object.operation = CollisionObject.ADD

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.CYLINDER
        primitive.dimensions = [float(z_max - z_min), float(self.base_column_radius)]

        pose = Pose()
        pose.position.z = float((z_min + z_max) / 2.0)
        pose.orientation.w = 1.0

        collision_object.primitives = [primitive]
        collision_object.primitive_poses = [pose]
        return collision_object

    def make_base_footprint_collision_object(self) -> Optional[CollisionObject]:
        if len(self.base_footprint_xy) < 3:
            self.get_logger().warn('Skipping base footprint collision object: not enough polygon vertices.')
            return None

        z_min, z_max = self._extract_z_limits(self.base_footprint_z_limits, default=(-1.112, -0.02))
        mesh = self.build_extruded_polygon_mesh(self.base_footprint_xy, z_min, z_max)

        collision_object = CollisionObject()
        collision_object.id = self.BASE_FOOTPRINT_OBJECT_ID
        collision_object.header.frame_id = self.planning_frame
        collision_object.operation = CollisionObject.ADD
        collision_object.meshes = [mesh]

        pose = Pose()
        pose.orientation.w = 1.0
        collision_object.mesh_poses = [pose]
        return collision_object

    def build_extruded_polygon_mesh(
        self,
        polygon_xy: Sequence[Tuple[float, float]],
        z_min: float,
        z_max: float,
    ) -> Mesh:
        polygon = list(polygon_xy)
        if self._signed_area(polygon) < 0.0:
            polygon.reverse()

        mesh = Mesh()
        vertex_count = len(polygon)

        for x, y in polygon:
            mesh.vertices.append(Point(x=float(x), y=float(y), z=float(z_min)))
        for x, y in polygon:
            mesh.vertices.append(Point(x=float(x), y=float(y), z=float(z_max)))

        for idx in range(1, vertex_count - 1):
            mesh.triangles.append(self.make_triangle(0, idx + 1, idx))
            mesh.triangles.append(
                self.make_triangle(vertex_count, vertex_count + idx, vertex_count + idx + 1)
            )

        for idx in range(vertex_count):
            next_idx = (idx + 1) % vertex_count
            mesh.triangles.append(self.make_triangle(idx, next_idx, vertex_count + next_idx))
            mesh.triangles.append(self.make_triangle(idx, vertex_count + next_idx, vertex_count + idx))

        return mesh

    def make_triangle(self, a: int, b: int, c: int) -> MeshTriangle:
        triangle = MeshTriangle()
        triangle.vertex_indices = [int(a), int(b), int(c)]
        return triangle


    def _get_bool_parameter(self, name: str) -> bool:
        value = self.get_parameter(name).value
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.strip().lower() in ('1', 'true', 'yes', 'on')
        return bool(value)

    def _panel_from_outline_marker(self, marker: Marker) -> Optional[dict]:
        if len(marker.points) < 4:
            return None

        vertices = marker.points[:4]
        width = self._distance(vertices[0], vertices[1])
        height = 0.5 * (
            self._distance(vertices[0], vertices[3]) + self._distance(vertices[1], vertices[2])
        )
        if width <= 1e-6 or height <= 1e-6:
            return None

        center = (
            sum(point.x for point in vertices) / 4.0,
            sum(point.y for point in vertices) / 4.0,
            sum(point.z for point in vertices) / 4.0,
        )
        yaw = math.atan2(
            vertices[1].y - vertices[0].y,
            vertices[1].x - vertices[0].x,
        )

        return {
            'frame_id': marker.header.frame_id or 'map',
            'center': center,
            'length': width,
            'height': height,
            'yaw': yaw,
        }

    def _pairs_from_flat_list(self, values: Sequence[float]) -> List[Tuple[float, float]]:
        if len(values) % 2 != 0:
            self.get_logger().warn(
                'base_footprint_xy must contain an even number of values. Ignoring the last entry.'
            )
            values = values[:-1]
        return [
            (float(values[idx]), float(values[idx + 1]))
            for idx in range(0, len(values), 2)
        ]

    def _extract_z_limits(
        self,
        values: Sequence[float],
        default: Tuple[float, float],
    ) -> Tuple[float, float]:
        if len(values) >= 2:
            return float(values[0]), float(values[1])
        return float(default[0]), float(default[1])

    def _signed_area(self, polygon_xy: Sequence[Tuple[float, float]]) -> float:
        area = 0.0
        vertex_count = len(polygon_xy)
        for idx in range(vertex_count):
            x1, y1 = polygon_xy[idx]
            x2, y2 = polygon_xy[(idx + 1) % vertex_count]
            area += (x1 * y2) - (x2 * y1)
        return 0.5 * area

    def _distance(self, p1: Point, p2: Point) -> float:
        return math.sqrt(
            (p1.x - p2.x) ** 2 +
            (p1.y - p2.y) ** 2 +
            (p1.z - p2.z) ** 2
        )


def main(args=None):
    rclpy.init(args=args)
    node = MoveItPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
