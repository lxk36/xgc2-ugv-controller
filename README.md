# XGC2 UGV Controller

ROS1 unicycle UGV controller product repository for XGC2 robots.

Packages:

- `unicycle_reference_trajectory`: planar reference trajectory messages,
  generation runtime, and ROS publishers.
- `unicycle_ugv_controller`: unicycle chassis controller, unique `cmd_vel`
  publisher. CONTROL states: SelfCheck / Ready / Reset / Custom1. Reset executes leased commands from `ugv_reset_safety`. Custom1 selects
  `nmpc` or `flatness` by rosparam. Remote I/O is canonical `{ns}/pose` only.
- `mecanum_ugv_controller`: reusable holonomic chassis modules. Health /
  SelfCheck, `Reset` to an Experiment `initialPose`, and first-order Custom1
  (world ENU velocity to body FLU, heading P to east). Algorithms publish
  `{ns}/alg/reference/twist` only; they do not publish `cmd_vel`.
- `ugv_reset_safety`: shared fleet Reset coordinator, geometric guidance,
  full-footprint obstacle/inter-vehicle CBF QP, command limits and slew limits.
  Both chassis owners use this implementation; the old Reset laws are removed.

Reset requires an explicit target, a complete UGV roster, fresh canonical poses
and controller states, and an authoritative static obstacle snapshot. Missing
inputs refuse motion. See [Reset design and limits](ugv_reset_safety/README.md).

## Install

```bash
sudo apt update
sudo apt install ros-noetic-xgc2-ugv-controller
```

## Smoke Test

```bash
source /opt/ros/noetic/setup.bash
roslaunch --files unicycle_reference_trajectory ugv_unicycle_reference_trajectory.launch
roslaunch --files unicycle_ugv_controller ugv_unicycle_nmpc_controller.launch
roslaunch --files mecanum_ugv_controller ugv_mecanum_reset_controller.launch
```

## Shuttle rail (no U-turn)

`unicycle_target_replanner` can hold a fixed `X` and shuttle along `Y`.
Rail heading is the nearer of `+Y` / `-Y` to the current body yaw. Body
speed is `vy * sin(yaw)`, so a nose already on `-Y` goes forward toward
`-Y` instead of spinning 180° onto `+Y`. Off-rail poses within
`shuttle_capture_radius` (default 30 m of the finite rail segment) get a
geometric SE2 plan onto a rail entry pose if MINCO fails. Reverse Y-legs
are published only after the robot is on the rail (X and a rail-axis yaw).
Poses farther than the capture radius are refused.

```bash
roslaunch unicycle_reference_trajectory ugv_unicycle_target_replanner.launch \
  ns:=ugv1 random_targets:=false shuttle_mode:=true \
  shuttle_x:=0.0 shuttle_y_min:=-2.0 shuttle_y_max:=2.0 shuttle_speed:=0.5
```

The runtime model keeps yaw rate as a state and commands angular acceleration:

```text
x = [x, y, yaw, speed, omega]
u = [linear_accel, angular_accel]
```

This makes `omega` continuous between shooting stages. Both its magnitude and
its rate of change are part of the optimization; `max_angular_acceleration`
also prevents a one-stage full-lock sign flip.

NMPC stage cost is nonlinear LS. All eleven weights are ROS params and
`roslaunch` args (`nmpc_weight_omega:=6` and so on). They are read
once when the node starts. Tune with launch args or `rosparam`, restart
the controller, then freeze the keepers into yaml and the launch defaults.

```bash
# ns:=ugv1  — set, restart NMPC, then dump
rosparam set /ugv1/unicycle_ugv_controller/nmpc/weights/omega 6.0
# restart unicycle_ugv_controller
rosparam get /ugv1/unicycle_ugv_controller/nmpc/weights
```

| param | launch arg | default |
| --- | --- | ---: |
| `nmpc/weights/position_x` | `nmpc_weight_position_x` | 20 |
| `nmpc/weights/position_y` | `nmpc_weight_position_y` | 20 |
| `nmpc/weights/yaw` | `nmpc_weight_yaw` | 8 |
| `nmpc/weights/speed` | `nmpc_weight_speed` | 4 |
| `nmpc/weights/accel` | `nmpc_weight_accel` | 0.4 |
| `nmpc/weights/omega` | `nmpc_weight_omega` | 10 |
| `nmpc/weights/angular_accel` | `nmpc_weight_angular_accel` | 1 |
| `nmpc/weights/terminal_position_x` | `nmpc_weight_terminal_position_x` | 60 |
| `nmpc/weights/terminal_position_y` | `nmpc_weight_terminal_position_y` | 60 |
| `nmpc/weights/terminal_yaw` | `nmpc_weight_terminal_yaw` | 20 |
| `nmpc/weights/terminal_speed` | `nmpc_weight_terminal_speed` | 10 |

Default `ω` is `10.0` (was `0.08`, then `4.0`). The old ratio made a saturated
yaw-rate cheaper than a few centimeters of cross-track.
Default `max_angular_acceleration` is `3.0 rad/s²`; it is an engineering value
selected from the 2026-08-21 Scout field bag, whose realized 0.1 s yaw-rate
finite-difference p95 was about `2.94 rad/s²`.

## Control-state modes

The same controller executable supports both state providers:

- `state_source:=state_estimator` consumes `RigidStateEstimate` and projects
  to SE2 (`x, y, yaw, speed, yaw_rate`) at the control boundary.
  Healthy means `estimator_state == STATE_RUNNING` (**3**, not the planar 2)
  and no `FLAG_FAULT`.
- `state_source:=vrpn_direct` consumes trusted VRPN pose and twist directly,
  without launching an estimator. The product NMPC remains the sole
  `cmd_vel` publisher for the nonholonomic vehicle.

Example:

```bash
roslaunch unicycle_ugv_controller ugv_unicycle_nmpc_controller.launch \
  ns:=ugv1 state_source:=vrpn_direct
```
