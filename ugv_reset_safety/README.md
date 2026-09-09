# Coordinated UGV Reset

This package coordinates Reset in both chassis controllers.
It does not control UAVs or change the Custom1 tracking laws. Start exactly one
coordinator for the complete ground-vehicle roster; the existing chassis nodes
remain the sole publishers of native `cmd_vel`.

## Data and execution

Provide each owner an explicit world-frame `geometry_msgs/Pose2D` on
`/<namespace>/reset_pose`, or all three `reset_initial_x/y/yaw` parameters.
Missing targets do not default to the shared world origin. Updating a target
caches it; only the existing `reset` command enters Reset. Each session freezes
its target. Completion requires XY error at most 0.05 m, measured low speed,
and an exact zero command checked by the safety QP;
there is no added terminal yaw gate. Timeout or Stop also returns Ready, so
Ready alone is not evidence of arrival.

```bash
roslaunch ugv_reset_safety ugv_reset_coordinator.launch \
  fleet_config:=/absolute/path/to/fleet.yaml
```

The installed `four_scout.yaml`, `two_mecanum.yaml`, and `mixed_pair.yaml` are
simulation envelopes, not measured physical-robot certifications. They specify
footprints, body-origin offsets, linear/yaw velocity limits, acceleration limits,
uncertainty bounds, and an explicit world-frame fence. Their Reset limits are
0.35 m/s per available translational channel and 0.5 rad/s, with 0.35 m/s² and
0.6 rad/s² slew limits. The coordinator rejects a nonstationary start. Stop the
vehicles before starting a new Reset session; the default session deadline is
600 seconds, including waiting for other Reset groups.

Obstacle input is `xgc2_geometry_msgs/GeometryLibrary` plus `ConvexBodyArray`.
Default topics are `/xgc2_geometry/geometry_library` and
`/xgc2_geometry/static_body_instances`. In Director-managed simulation, explicitly
select `/xgc2/simulation/obstacles/geometry_library` and
`/xgc2/simulation/obstacles/instances`. Do not subscribe competing maps. All data
must already use the canonical pose frame (`world` by default). The scene must
cover every external obstacle; the Director snapshot covers its managed prefix,
not arbitrary unregistered Gazebo props. Static snapshots are latched; identical
retransmissions preserve sessions, changed geometry cancels them.

`cube`, `sphere`, `cylinder`, and polytope/mesh support-point templates are
projected conservatively to convex 2D polygons. Primitive curved shapes use
transformed bounding boxes. Received unsupported, malformed, or moving geometry rejects
Reset. An absent scene is not treated as an empty scene. A valid explicitly
empty snapshot is supported.

## Guidance and control

Each session obtains a visibility-graph/Dijkstra route around inflated static
polygons, then follows a finite lookahead point. This small geometric planner
provides a way around obstacles that a purely local CBF may stop in front of;
it is not a time-parameterized dynamic feasibility certificate. The route is
cached, not repeatedly optimized. A frozen goal-occupancy graph orders the fleet
by strongly connected components:
a vehicle clears another's target first, with deterministic ordering of independent
groups. One vehicle or a mutually dependent pair runs at a time; remaining robots
are held at zero and included in route geometry. Pairs use persistent right-hand
passage targets to break symmetric encounters. Target-overlap conflicts, a target
occupied by a nonparticipating robot, and dependency cycles of more than two robots
are explicitly rejected before motion. This bounded scheduler does not implement
staging-place search for larger cycles. A short 150 ms admission interval collects
the Reset command fan-out while holding all vehicles stationary; joins during
motion or unfinished batch membership changes reject the batch. Collision constraints remain
in the joint QP, so the approach is centralized for the small fleet.

Mecanum computes nominal world XY velocity and rotates it into body FLU using
the current canonical pose yaw. It simultaneously applies wrapped shortest-angle
feedback to the requested yaw. Body `vx`, `vy`, and yaw rate are independent QP
variables; a nonzero heading does not corrupt world-frame translation.

Scout commands only body `vx` and yaw rate (`vy` is always zero). Its nominal
polar law follows a fixed reverse direction during Reset. With reverse-bearing
error `alpha`, distance `rho` to the current lookahead point, and uncertainty
offset bound `ell`, it uses:

```
k_r = min(position_gain, 0.5 * max_omega)
lambda = rho / (rho + ell)
v_nom = -lambda * min(max_vx, k_r * rho)
omega_nom = lambda * clamp(heading_gain * alpha, -max_omega, max_omega)
```

The gain cap preserves steering authority under yaw saturation; common scaling
reduces feedback bandwidth near the target without changing the commanded
curvature ratio. A geometrically feasible reverse tangent can bias the final
approach toward the requested target yaw. If it would require a cusp or blocked
approach, final heading is unconstrained. No terminal in-place rotation is
required. This is a position-reset controller with a soft heading objective,
not an arbitrary terminal-pose convergence guarantee.

The reverse law was selected against the observed model
`v_y = -ell_actual * omega`, with `ell_actual` in `[0, 0.25]` m, and synthetic
linear/angular actuator lag. That is an empirical nominal-convergence envelope;
positive coupling has not been established as convergent. No lateral actuator
or differentiated lateral-speed feedback is introduced.

## Safety filter and failure behavior

Each rectangle is covered by multiple overlapping disks. Their body offsets
rotate with pose, so yaw motion of the corners participates in the obstacle,
fence, and every inter-vehicle disk-pair constraint. For disk center `q`,

```
q_dot = A(pose, disk_offset) * u + delta * omega * body_left + disturbance
|delta| <= lateral_velocity_per_yaw_bound
||disturbance|| <= velocity_uncertainty
h_obstacle = distance(q, convex_obstacle)^2 - inflated_disk_radius^2
h_dot + barrier_gain * h >= 0
```

The QP enumerates both coupling signs (four combinations for vehicle pairs),
adds worst-case bounded translational disturbance, and enforces velocity and
command-slew boxes as hard constraints. Only the nominal tracking objective is
soft. OSQP results must pass finite-value, solver-status, residual, and timing
checks. Inactive channels and Scout lateral commands are exact zeros; residuals
are checked after canonicalizing those fixed channels. An active stop request
requires zero within the same slew and disturbance constraints before reporting
arrival. There is no safety-constraint slack or old-controller fallback.

Requests carry explicit process/session generation, exact request timestamp,
pose timestamp, and the last command actually submitted by the native ROS
publisher. ROS rewrites `Header.seq`, so it is not used as a session identifier.
The coordinator bases slew bounds on that acknowledgement, not on an unreceived
proposal. This acknowledges ROS publish submission, not hardware delivery.
Owners reject unmatched/replayed responses; command leases expire after 150 ms
in both wall and ROS time. Wall loops retain the stop watchdog during a paused
simulation clock. Slew integration follows elapsed ROS/plant time.

Invalid scene, stale fleet pose/state, uncontrolled moving peer, changed target,
clock/deadline failure, or infeasible QP rejects active sessions and requests zero
output. Emergency zero is a stop request, not a proof of instantaneous physical
braking or continued CBF feasibility. The continuous-time barrier assumptions do
not alone prove sampled-data safety with arbitrary actuator delay, tracking
error, packet loss, or an initial collision. Margins must bound the actual plant.
No controller can reach a target inside an obstacle or satisfy overlapping fleet
goals. No-route and rejection are distinct from successful completion.

## Verification boundary

Tests separately exercise geometry/QP constraints, guidance convergence,
closed-loop rectangle collision clearance (including plant substeps), command
slew, disturbed motion, and real ROS owner/coordinator session transport.
Deterministic seeds preserve discovered near-goal/orbit counterexamples.
Simulation and physical-robot acceptance remain separate from these offline
models and ROS kinematic tests; results belong to their exact source revision.

The combined scenario test contains 164 deterministic trials: 60 varied single
and paired starts, 100 seeded sparse polygon layouts, and four four-Scout
opposite-corner exchanges. Its tested outcome is 160 arrivals and four
conservative no-route rejections before motion, with no detected rectangle
collision. The sparse and four-Scout cases use 0.62 m by 0.52 m bodies, the
0.35 m/s and 0.5 rad/s profile above, a 0.25 m coupling bound, 0.10 m/s residual
velocity bound, and 0.13 m geometric margin. Their perturbed plant uses
`v_y = -0.229 * omega`, with 0.12 s linear and 0.16 s angular first-order lag.
Commands update at 20 ms; physical rectangle clearance is checked at 2 ms plant
substeps. The four-Scout exchanges take about four minutes (236–245 s), including
waiting for the other pair. These finite fixtures do not establish convergence
for arbitrary initial layouts, larger occupancy cycles, unmodelled obstacles,
or physical robots. Terminal yaw is reported but is not an arrival criterion.

The visibility graph follows [LaValle, Planning Algorithms §6.2.4](https://msl.cs.illinois.edu/~lavalle/planning/node271.html).
The CBF-QP construction follows the framework in
[Ames et al., Control Barrier Functions: Theory and Applications](https://arxiv.org/abs/1903.11199).
These sources do not certify this package's discrete implementation or hardware.
