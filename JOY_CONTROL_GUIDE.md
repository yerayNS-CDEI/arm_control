# Gamepad Control Reference

Logitech F710, **mode switch at D** (DirectInput). Check with `ros2 topic echo /joy`:
6 axes means D and this map is correct, 8 means the switch is at X and every
index below is wrong.

Raw indices, for reading the configs:

```
axes     0/1 left stick X/Y    2/3 right stick X/Y    4/5 dpad X/Y
buttons  0 X   1 A   2 B   3 Y   4 LB  5 RB  6 LT  7 RT
         8 BACK  9 START  10 left stick click  11 right stick click
```

## The rule

**The mode is whichever button you hold.** Release it and everything stops.
Holding two mode buttons stops the arm rather than guessing which you meant.

| Hold | What moves |
|------|-----------|
| **X** | Mobile base (and turret) |
| **Y** | Arm, joint by joint |
| **A** | Arm, Cartesian in the **tool** frame |
| **B** | Arm, Cartesian in the **base** frame |

The right stick is unused on purpose — the right thumb's only job is holding the
mode button.

## Base — hold X

`teleop_twist_joy` + `turret_joy`, configured in navi_wall's
`config/general_params.yaml` (diff) and `general_params_omni.yaml` (omni).
**RB = turbo** while held.

| Control | Omni | Differential |
|---------|------|--------------|
| Left stick down/up | X, 0.3 → 0.8 m/s | X, 0.15 → 0.8 m/s |
| Left stick left/right | Y strafe, 0.3 → 0.8 m/s | yaw, 0.4 → 0.5 rad/s |
| Dpad left/right | yaw, 0.8 → 0.5 rad/s **+ turret** | turret |
| Dpad down/up | turret speed step | turret speed step |

In omni, dpad left/right drives base yaw *and* the turret. That is pre-existing
behaviour, not something the arm jog introduced.

The turret only responds while X is held — that is what keeps the dpad free for
the arm modes.

## Arm — hold Y, A or B

Nothing moves until the jog is **armed** (below).

| Control | **Y** joint mode | **A** / **B** Cartesian |
|---------|------------------|-------------------------|
| LT / RT | J1 shoulder_pan −/+ | RZ −/+ |
| LB / RB | J2 shoulder_lift −/+ | Z −/+ |
| Left stick left/right | J3 elbow | RY |
| Left stick down/up | J4 wrist_1 | RX |
| Dpad left/right | J5 wrist_2 | X |
| Dpad down/up | J6 wrist_3 | Y |
| BACK / START | speed scale −/+ | speed scale −/+ |

A and B share one control table. The only difference is which frame the axes
refer to — exactly what the pendant's Base/Tool toggle changes. Rotations are
about the TCP in both, which on this robot is `arm_tool0`, 0.300 m out along the
tool axis.

Jogging is fixed-rate: past the deadzone a control means full speed in that
direction, like the pendant's arrows. Set `proportional_axes: true` in
`config/joy_arm.yaml` if you want stick deflection to scale the speed instead.

## Arming and disarming

**Press both stick clicks together and hold for half a second.** It toggles once
per press — holding longer will not toggle it back; release and press again.

Or from the UI / a terminal:

```bash
ros2 service call /arm_joy_node/set_armed std_srvs/srv/SetBool "{data: true}"
ros2 service call /arm_joy_node/set_armed std_srvs/srv/SetBool "{data: false}"
```

**Arming takes the arm away from the FSM.** It raises the stack's
`emergency_stop`, which cancels any running trajectory and makes the planners
refuse new ones, then swaps `passthrough_trajectory_controller` (real) or
`joint_trajectory_controller` (Gazebo) out for `forward_position_controller`.
Planned motions cannot run while the jog is armed. Disarming reverses both.

It also disarms itself after `disarm_timeout` (60 s) with no jogging.

## Checking what it is doing

```bash
ros2 topic echo /arm_joy_node/status
```

Publishes continuously whether armed or not:

```
disarmed none scale=0.50 joy=ok
armed cartesian_tcp scale=0.50 joy=ok
```

Watch this while pressing buttons — the mode changes here before anything moves,
which makes it a safe way to learn the layout.

Controller states:

```bash
ros2 control list_controllers | grep -E "forward_position|trajectory"
```

`forward_position_controller inactive` is the correct resting state. It goes
`active` only while armed.

## If nothing moves

| Symptom | Check |
|---------|-------|
| No `/arm_joy_node` in `ros2 node list` | launched with `joy_arm:=false`? It defaults to true |
| `status` says `joy=stale` | joystick unplugged, or `/joy` not reaching the node |
| Armed, mode shows, still nothing | `forward_position_controller` loaded? Under Gazebo it needs the spawner that `joy_arm` gates |
| Arms then instantly disarms | released the chord too slowly — it is latched, not repeating |
| Cartesian does nothing, joint works | TF `arm_base`→`arm_tool0` missing; the log says so |
| Jog goes heavy near a pose | singularity slowdown, reported in the log with sigma |
| Stops after ~15 s of holding | `max_continuous_jog`; release the control and press again |

## Limits worth knowing

- Speeds, ramps and watchdogs all live in `config/joy_arm.yaml` — no rebuild
  needed, the config installs as a symlink.
- A joint on its limit stops going outward but can still be jogged back.
- Positions are streamed, not velocities, so a lost publisher holds the arm
  rather than letting it run on.
