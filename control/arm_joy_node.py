#!/usr/bin/env python3
"""Jog the UR10e from the gamepad, the way the teach pendant's Move tab does.

The mode is whichever button is held -- Y joint, A cartesian in the tool frame,
B cartesian in the base frame -- and releasing it stops the arm. That is the same
contract the base already has through ``teleop_twist_joy``'s ``enable_button``,
and it means there is no latched mode to disagree with what the operator believes
is selected. Holding two mode buttons stops the arm rather than picking a winner.

Jogging needs the arm's command interfaces, so it cannot coexist with the
trajectory controller the planner and the FSM execute through. Rather than paper
over that, this node switches ``passthrough_trajectory_controller`` out and
``forward_position_controller`` in when armed, and back when disarmed, letting
ros2_control enforce that a jog and a planned trajectory can never run at once.
Arming is deliberate: both stick clicks held, or the ``~/set_armed`` service for
the UI.

Positions are streamed rather than velocities. Behind a wireless gamepad a lost
publisher must mean "hold", not "keep going at the last commanded speed".

The kinematics live in ``planner_lib.jog`` so they can be tested without a robot;
this file is the ROS shell around them.
"""

import numpy as np
import rclpy
import rclpy.time
import tf2_ros
from controller_manager_msgs.srv import SwitchController
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import JointState, Joy
from std_msgs.msg import Bool, Float64MultiArray, String
from std_srvs.srv import SetBool

from planner.planner_lib.jog import (
    MODE_CARTESIAN_BASE,
    MODE_CARTESIAN_TCP,
    MODE_JOINT,
    MODE_NONE,
    active_mode,
    block_at_joint_limits,
    cartesian_jog_velocity,
    joint_jog_velocity,
    parse_mapping,
    ramp,
    read_command,
    rising_edge,
)
from planner.planner_lib.ur10e_kinematics import calibrate_tool0_to_dh_end, fk, pose_matrix


class ArmJoyNode(Node):

    def __init__(self):
        super().__init__('arm_joy_node')

        # --- Gamepad ---------------------------------------------------------
        self.declare_parameter('joy_topic', 'joy')
        self.declare_parameter('deadzone', 0.3)
        self.declare_parameter('proportional_axes', False)
        self.declare_parameter('mode_button_joint', 3)
        self.declare_parameter('mode_button_tcp', 1)
        self.declare_parameter('mode_button_base', 2)
        self.declare_parameter('blocking_buttons', [0])
        self.declare_parameter('speed_down_button', 8)
        self.declare_parameter('speed_up_button', 9)
        self.declare_parameter('arm_chord', [10, 11])
        self.declare_parameter('arm_chord_hold', 0.5)
        self.declare_parameter('joint_mode', [
            'btn:6,7', 'btn:4,5', 'axis:0', 'axis:1', 'axis:4', 'axis:5',
        ])
        self.declare_parameter('cartesian_mode', [
            'axis:4', 'axis:5', 'btn:4,5', 'axis:1', 'axis:0', 'btn:6,7',
        ])

        # --- Speeds ----------------------------------------------------------
        self.declare_parameter('joint_speed', 0.20)
        self.declare_parameter('linear_speed', 0.05)
        self.declare_parameter('angular_speed', 0.20)
        self.declare_parameter('speed_scale', 0.5)
        self.declare_parameter('speed_scale_min', 0.1)
        self.declare_parameter('speed_scale_max', 1.0)
        self.declare_parameter('speed_scale_step', 0.1)
        self.declare_parameter('max_velocity_fraction', 0.25)
        self.declare_parameter('acceleration', 1.0)
        self.declare_parameter('joint_limit_margin', 0.10)
        self.declare_parameter('singularity_slow_below', 0.05)
        self.declare_parameter('singularity_stop_below', 0.01)

        # --- Rates and watchdogs ---------------------------------------------
        self.declare_parameter('publish_rate', 100.0)
        self.declare_parameter('joint_state_timeout', 0.5)
        self.declare_parameter('joy_timeout', 0.3)
        self.declare_parameter('max_continuous_jog', 15.0)
        self.declare_parameter('disarm_timeout', 60.0)
        self.declare_parameter('max_tracking_error', 0.20)

        # --- Robot -----------------------------------------------------------
        self.declare_parameter('joints', [
            'arm_shoulder_pan_joint', 'arm_shoulder_lift_joint', 'arm_elbow_joint',
            'arm_wrist_1_joint', 'arm_wrist_2_joint', 'arm_wrist_3_joint',
        ])
        self.declare_parameter('base_frame', 'arm_base')
        self.declare_parameter('tool0_frame', 'arm_tool0')
        self.declare_parameter('wrist_frame', 'arm_wrist_3_link')
        self.declare_parameter('preempt_on_arm', True)
        self.declare_parameter('preempt_topic', 'emergency_stop')
        self.declare_parameter('preempt_delay', 0.25)
        self.declare_parameter('controller_manager', 'controller_manager')
        self.declare_parameter('jog_controller', 'forward_position_controller')
        self.declare_parameter('trajectory_controller', 'passthrough_trajectory_controller')

        p = self.get_parameter
        self.deadzone = p('deadzone').value
        self.proportional = p('proportional_axes').value
        self.mode_buttons = {
            MODE_JOINT: p('mode_button_joint').value,
            MODE_CARTESIAN_TCP: p('mode_button_tcp').value,
            MODE_CARTESIAN_BASE: p('mode_button_base').value,
        }
        self.blocking_buttons = list(p('blocking_buttons').value)
        self.speed_down_button = p('speed_down_button').value
        self.speed_up_button = p('speed_up_button').value
        self.arm_chord = list(p('arm_chord').value)
        self.arm_chord_hold = p('arm_chord_hold').value

        try:
            self.joint_map = parse_mapping(list(p('joint_mode').value))
            self.cartesian_map = parse_mapping(list(p('cartesian_mode').value))
        except ValueError as e:
            # A broken mapping means the operator's buttons do something other
            # than what the config says; refuse rather than jog by surprise.
            self.get_logger().fatal(f'Bad jog mapping: {e}')
            raise

        self.joint_speed = p('joint_speed').value
        self.linear_speed = p('linear_speed').value
        self.angular_speed = p('angular_speed').value
        self.speed_scale = p('speed_scale').value
        self.speed_scale_min = p('speed_scale_min').value
        self.speed_scale_max = p('speed_scale_max').value
        self.speed_scale_step = p('speed_scale_step').value
        self.max_velocity_fraction = p('max_velocity_fraction').value
        self.acceleration = p('acceleration').value
        self.joint_limit_margin = p('joint_limit_margin').value
        self.singularity_slow_below = p('singularity_slow_below').value
        self.singularity_stop_below = p('singularity_stop_below').value

        self.joint_state_timeout = p('joint_state_timeout').value
        self.joy_timeout = p('joy_timeout').value
        self.max_continuous_jog = p('max_continuous_jog').value
        self.disarm_timeout = p('disarm_timeout').value
        self.max_tracking_error = p('max_tracking_error').value

        self.joint_names = list(p('joints').value)
        self.base_frame = p('base_frame').value
        self.tool0_frame = p('tool0_frame').value
        self.wrist_frame = p('wrist_frame').value
        self.preempt_on_arm = p('preempt_on_arm').value
        self.preempt_delay = p('preempt_delay').value
        self.jog_controller = p('jog_controller').value
        self.trajectory_controller = p('trajectory_controller').value

        # --- State -----------------------------------------------------------
        self.armed = False
        self.switch_pending = False
        self.joy = None
        self.joy_stamp = 0.0
        self.prev_buttons = []
        self.chord_since = None
        self.chord_fired = False
        self.q = None                 # measured, ordered like self.joint_names
        self.q_stamp = 0.0
        self.q_cmd = None             # streamed command, integrated from qdot
        self.qdot = np.zeros(6)
        self.mode = MODE_NONE
        self.jog_since = None         # start of the current uninterrupted jog
        self.idle_since = None
        self.jog_capped = False
        self.T_tool0_dhend = None
        self.preempt_timer = None
        self.preempting = False

        # --- ROS -------------------------------------------------------------
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_subscription(Joy, p('joy_topic').value, self.on_joy, 1)
        self.create_subscription(JointState, 'joint_states', self.on_joint_state, 10)
        self.command_pub = self.create_publisher(
            Float64MultiArray, f'{self.jog_controller}/commands', 1)
        self.status_pub = self.create_publisher(String, '~/status', 1)
        # Same QoS the planners subscribe with, so a late-joining node still sees
        # that the operator has taken the arm.
        self.preempt_pub = self.create_publisher(
            Bool, p('preempt_topic').value,
            QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                       reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_service(SetBool, '~/set_armed', self.on_set_armed)
        self.switch_client = self.create_client(
            SwitchController, f"{p('controller_manager').value}/switch_controller")

        rate = float(p('publish_rate').value)
        self.dt = 1.0 / rate
        self.create_timer(self.dt, self.on_timer)

        self.get_logger().info(
            f'Arm jog ready, disarmed. Hold buttons {self.arm_chord} together to arm; '
            f'then hold {self.mode_buttons[MODE_JOINT]} for joint, '
            f'{self.mode_buttons[MODE_CARTESIAN_TCP]} for cartesian tool, '
            f'{self.mode_buttons[MODE_CARTESIAN_BASE]} for cartesian base.'
        )

    # ------------------------------------------------------------------
    # Inputs
    # ------------------------------------------------------------------
    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_joy(self, msg):
        self.joy = msg
        self.joy_stamp = self.now()

        buttons = list(msg.buttons)
        # Speed steps and the arm chord act on the press, not while held: joy_linux
        # autorepeats at 20 Hz, so a held button would otherwise run the speed
        # scale end to end in half a second.
        chord_held = bool(self.arm_chord) and all(
            b < len(buttons) and buttons[b] for b in self.arm_chord)
        if chord_held:
            if self.chord_since is None:
                self.chord_since = self.now()
            elif (not self.chord_fired
                    and self.now() - self.chord_since >= self.arm_chord_hold
                    and not self.switch_pending):
                # Latched until release: the chord is held past the trigger every
                # time, and without this it toggles again half a second later --
                # arming and instantly disarming, which reads as "it will not stay
                # armed" rather than as a bug.
                self.chord_fired = True
                self.set_armed(not self.armed, 'stick-click chord')
        else:
            self.chord_since = None
            self.chord_fired = False
            if rising_edge(buttons, self.prev_buttons, self.speed_up_button):
                self.step_speed(+1)
            if rising_edge(buttons, self.prev_buttons, self.speed_down_button):
                self.step_speed(-1)

        self.prev_buttons = buttons

    def step_speed(self, direction):
        self.speed_scale = float(np.clip(
            self.speed_scale + direction * self.speed_scale_step,
            self.speed_scale_min, self.speed_scale_max))
        self.get_logger().info(f'Jog speed scale {self.speed_scale:.2f}')

    def on_joint_state(self, msg):
        try:
            self.q = np.array([msg.position[msg.name.index(n)] for n in self.joint_names])
        except (ValueError, IndexError):
            # joint_states is merged from base and arm, so a message that does not
            # carry all six arm joints is normal rather than an error.
            return
        self.q_stamp = self.now()

    def on_set_armed(self, request, response):
        response.success = self.set_armed(request.data, 'service')
        response.message = 'armed' if self.armed else 'disarmed'
        return response

    # ------------------------------------------------------------------
    # Arming: swap the trajectory controller for the jog controller
    # ------------------------------------------------------------------
    def set_armed(self, armed, source):
        if armed == self.armed or self.switch_pending:
            return True
        if armed and self.q is None:
            self.get_logger().warn('Refusing to arm the jog: no joint_states yet')
            return False
        if not self.switch_client.service_is_ready():
            self.get_logger().warn(
                f'Refusing to {"arm" if armed else "disarm"} the jog: '
                f'{self.switch_client.srv_name} is not available')
            return False

        self.switch_pending = True
        if armed and self.preempt_on_arm:
            # The operator outranks the FSM. Tell the planners to drop what they
            # are running *before* taking the controller away, so a running
            # trajectory is cancelled through the path they already listen on
            # and reported as such -- rather than dying when the controller it
            # was executing through disappears underneath it.
            self.publish_preempt(True)
            self.preempt_timer = self.create_timer(
                self.preempt_delay, lambda: self.fire_switch(armed, source))
        else:
            self.request_switch(armed, source)
        return True

    def publish_preempt(self, stop):
        """Raise or clear the stack's emergency_stop.

        This is the channel the planners and the trajectory bridge already watch:
        a True cancels the active goal and makes them refuse new ones, a False
        hands the arm back. Reused rather than reinvented so that taking the arm
        on the gamepad looks the same to the rest of the stack as any other stop.
        """
        self.preempting = stop
        self.preempt_pub.publish(Bool(data=bool(stop)))

    def fire_switch(self, armed, source):
        if self.preempt_timer is not None:
            self.preempt_timer.cancel()
            self.preempt_timer = None
        self.request_switch(armed, source)

    def request_switch(self, armed, source):
        request = SwitchController.Request()
        if armed:
            request.activate_controllers = [self.jog_controller]
            request.deactivate_controllers = [self.trajectory_controller]
        else:
            request.activate_controllers = [self.trajectory_controller]
            request.deactivate_controllers = [self.jog_controller]
        # STRICT: a half-completed switch would leave two controllers claiming the
        # same command interfaces, which is worse than not switching at all.
        request.strictness = SwitchController.Request.STRICT
        request.timeout = Duration(seconds=2.0).to_msg()

        future = self.switch_client.call_async(request)
        future.add_done_callback(lambda f: self.on_switch_done(f, armed, source))

    def on_switch_done(self, future, armed, source):
        self.switch_pending = False
        try:
            ok = future.result().ok
        except Exception as e:
            self.get_logger().error(f'Controller switch failed: {e}')
            self.release_failed_preempt()
            return
        if not ok:
            self.get_logger().error(
                f'Controller manager refused to switch to '
                f'{self.jog_controller if armed else self.trajectory_controller}')
            self.release_failed_preempt()
            return

        self.armed = armed
        self.reset_jog()
        if armed:
            # Seed the stream with where the arm already is, immediately and before
            # anything else can command it, rather than trusting the controller's
            # activation behaviour to hold position for us.
            self.q_cmd = self.q.copy()
            self.publish_command()
            self.idle_since = self.now()
        else:
            # Arm handed back: clear the stop last, so nothing can dispatch a new
            # trajectory into the window before the controller is ready for it.
            self.publish_preempt(False)
        self.get_logger().warn(
            f'Arm jog {"ARMED" if armed else "disarmed"} ({source}); '
            f'{self.trajectory_controller} is now '
            f'{"inactive - planned trajectories will not run" if armed else "active"}')

    def release_failed_preempt(self):
        """An arm that never happened must not leave the planners blocked."""
        if self.preempting:
            self.publish_preempt(False)

    def reset_jog(self):
        self.qdot = np.zeros(6)
        self.mode = MODE_NONE
        self.jog_since = None
        self.jog_capped = False

    # ------------------------------------------------------------------
    # Control loop
    # ------------------------------------------------------------------
    def on_timer(self):
        # Status first and unconditionally. It used to ride along with the joint
        # command, which only publishes while armed, so the topic went silent in
        # exactly the state someone would be checking it for -- a UI or an
        # operator asking "is the jog armed?" got no answer at all.
        self.publish_status()
        if not self.armed or self.q_cmd is None:
            return

        now = self.now()
        if self.q is None or now - self.q_stamp > self.joint_state_timeout:
            self.halt('joint_states went stale')
            return
        if np.max(np.abs(self.q_cmd - self.q)) > self.max_tracking_error:
            # The arm is not where we told it to be: the controller is not tracking,
            # or something else is commanding it. Hand the arm back rather than
            # keep integrating away from reality.
            self.halt('command and measurement diverged')
            self.set_armed(False, 'tracking error')
            return

        mode = MODE_NONE
        if self.joy is not None and now - self.joy_stamp <= self.joy_timeout:
            mode = active_mode(
                list(self.joy.buttons), self.mode_buttons, self.blocking_buttons)

        if mode != self.mode:
            self.mode = mode
            self.jog_since = None
            self.jog_capped = False

        target = np.zeros(6)
        if mode != MODE_NONE:
            target = self.jog_target(mode)

        # The cap is on time spent actually moving, not on time holding the mode
        # button: standing on the enable with the sticks centred is how an
        # operator waits, and cutting that off would train them to fight it.
        if np.any(target):
            if self.jog_since is None:
                self.jog_since = now
            elif now - self.jog_since > self.max_continuous_jog:
                if not self.jog_capped:
                    self.get_logger().warn(
                        f'Jogged for over {self.max_continuous_jog:.0f} s without '
                        f'letting go; stopping. Release the control to continue.')
                    self.jog_capped = True
                target = np.zeros(6)
        else:
            self.jog_since = None
            self.jog_capped = False

        if np.any(target) or np.any(self.qdot):
            self.idle_since = now
        elif self.idle_since is not None and now - self.idle_since > self.disarm_timeout:
            self.set_armed(False, f'idle for {self.disarm_timeout:.0f} s')
            return

        self.qdot = ramp(self.qdot, target, self.acceleration * self.dt)
        self.qdot = block_at_joint_limits(
            self.q_cmd, self.qdot, self.dt, self.joint_limit_margin)

        if not np.any(self.qdot):
            # Idle: follow the measurement so the command never drifts away from
            # the arm, and so an external nudge does not become a step on resume.
            self.q_cmd = self.q.copy()
        else:
            self.q_cmd = self.q_cmd + self.qdot * self.dt

        self.publish_command()

    def jog_target(self, mode):
        """Target joint velocities for the held mode, before ramping."""
        axes, buttons = list(self.joy.axes), list(self.joy.buttons)
        scale = self.speed_scale

        if mode == MODE_JOINT:
            command = read_command(
                self.joint_map, axes, buttons, self.deadzone, self.proportional)
            return joint_jog_velocity(
                command, self.joint_speed * scale, self.max_velocity_fraction)

        frames = self.cartesian_frames()
        if frames is None:
            self.get_logger().warn(
                f'Cartesian jog needs {self.base_frame}->{self.tool0_frame}; '
                f'joint mode still works', throttle_duration_sec=5.0)
            return np.zeros(6)
        rotation, p_tcp, p_dh_end = frames

        command = read_command(
            self.cartesian_map, axes, buttons, self.deadzone, self.proportional)
        qdot, sigma, singular = cartesian_jog_velocity(
            self.q_cmd, command,
            rotation if mode == MODE_CARTESIAN_TCP else np.eye(3),
            p_tcp, p_dh_end,
            self.linear_speed * scale, self.angular_speed * scale,
            self.max_velocity_fraction,
            self.singularity_slow_below, self.singularity_stop_below,
        )
        if np.any(command) and singular < 1.0:
            self.get_logger().warn(
                f'Near a singularity (sigma={sigma:.4f}): jog at {singular * 100:.0f}%',
                throttle_duration_sec=2.0)
        return qdot

    def cartesian_frames(self):
        """``(R_base_tool0, p_tool0, p_dh_end)`` in ``arm_base``, or None.

        The analytic IK's DH end frame is not ``arm_tool0``, so the operator's
        "tool frame" and the frame the Jacobian describes are two different
        things. Both come out of one FK call once the constant offset between
        them has been measured against TF.
        """
        if self.T_tool0_dhend is None:
            self.T_tool0_dhend = self.calibrate()
            if self.T_tool0_dhend is None:
                return None
        T_base_dhend = fk(self.q_cmd)
        T_base_tool0 = T_base_dhend @ np.linalg.inv(self.T_tool0_dhend)
        return T_base_tool0[:3, :3], T_base_tool0[:3, 3], T_base_dhend[:3, 3]

    def calibrate(self):
        """Measure the constant ``tool0 -> DH end`` offset against live TF.

        The same trick ``wall_sweep_executor`` uses: derive it from FK(q) against
        the real transform rather than from flange conventions, because a
        convention slip here jogs the tool along an axis the operator did not pick.
        """
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame, self.tool0_frame, rclpy.time.Time(),
                Duration(seconds=0.1))
        except Exception as e:
            self.get_logger().warn(
                f'{self.base_frame}->{self.tool0_frame} lookup failed: {e}',
                throttle_duration_sec=5.0)
            return None
        t, r = tf.transform.translation, tf.transform.rotation
        T_base_tool0 = pose_matrix(
            R.from_quat([r.x, r.y, r.z, r.w]).as_matrix(), [t.x, t.y, t.z])
        offset = calibrate_tool0_to_dh_end(T_base_tool0, self.q)
        self.get_logger().info(
            f'Calibrated {self.tool0_frame}->DH-end '
            f'(trans={np.round(offset[:3, 3], 4).tolist()})')
        self.report_tcp()
        return offset

    def report_tcp(self):
        """Say where the jog believes the TCP is, so it can be checked in seconds.

        Cartesian rotations happen about ``tool0_frame``, and on this robot that
        sits 0.30 m out along the tool axis (a 0.15 m adapter cylinder plus a
        0.15 m ``sensors_offset``). With a lever that long, a TCP that disagrees
        with the pendant's by a couple of centimetres is plainly visible as the
        tool swinging instead of spinning -- so print the number rather than
        leave it to be discovered on the robot.

        If it disagrees with the pendant, fix ``sensors_offset`` in the URDF
        rather than adding an offset here: the planner and the collision scene
        read the TCP from the same TF, and a jog-local override would quietly put
        them in different places.
        """
        try:
            tf = self.tf_buffer.lookup_transform(
                self.wrist_frame, self.tool0_frame, rclpy.time.Time(),
                Duration(seconds=0.1))
        except Exception:
            return
        t = tf.transform.translation
        self.get_logger().warn(
            f'Jog rotates about {self.tool0_frame}, '
            f'{np.linalg.norm([t.x, t.y, t.z]):.3f} m from {self.wrist_frame} '
            f'(xyz={np.round([t.x, t.y, t.z], 4).tolist()}). '
            f'Check this against the TCP set on the teach pendant.')

    def halt(self, reason):
        if np.any(self.qdot):
            self.get_logger().warn(f'Jog halted: {reason}')
        self.reset_jog()
        if self.q is not None:
            self.q_cmd = self.q.copy()
            self.publish_command()

    def publish_command(self):
        msg = Float64MultiArray()
        msg.data = [float(v) for v in self.q_cmd]
        self.command_pub.publish(msg)

    def publish_status(self):
        joy_age = self.now() - self.joy_stamp if self.joy is not None else float('inf')
        self.status_pub.publish(String(
            data=f'{"armed" if self.armed else "disarmed"} {self.mode} '
                 f'scale={self.speed_scale:.2f} '
                 f'joy={"ok" if joy_age <= self.joy_timeout else "stale"}'))


def main(args=None):
    rclpy.init(args=args)
    node = ArmJoyNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        # SIGINT/SIGTERM already shuts the context down via rclpy's signal
        # handler; a second shutdown() raises RCLError -> exit 1.
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
