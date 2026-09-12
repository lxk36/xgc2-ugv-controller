# Coordinated UGV Reset

This package coordinates Reset for Scout and Mecanum. It does not change UAV
control or Custom1 tracking. Run one coordinator for the ground-vehicle roster;
each chassis controller remains the sole publisher of native `cmd_vel`.

## Targets and execution

Supply a world-frame `geometry_msgs/Pose2D` on `/<namespace>/reset_pose`, or all
three `reset_initial_x/y/yaw` parameters. The product launcher maps each
Experiment slot's `initialPose` to these parameters. Missing targets never
default to the origin. Updating a target only caches it; `reset` enters Reset,
and the session freezes that target and its generation.

Arrival requires XY error at most 0.05 m, shortest yaw error at most 10 degrees,
measured low speed, and an exact zero command for both chassis. Stop and timeout
also return Ready: Ready alone is not arrival. The chassis Reset timeout is 90 s.

```bash
roslaunch ugv_reset_safety ugv_reset_coordinator.launch \
  fleet_config:=/absolute/path/to/fleet.yaml
```

The installed `four_scout.yaml`, `two_mecanum.yaml`, and `mixed_pair.yaml` provide
simulation profiles: 0.35 m/s on available translation axes, 0.5 rad/s yaw,
0.35 m/s² linear acceleration, and 0.6 rad/s² angular acceleration. These are
configuration limits, not physical-robot certifications.

A 150 ms admission interval collects the requesting roster. The cohort starts
together once all members enter Reset. Overlapping targets and a target occupied
by a nonparticipant reject admission. Nonparticipants and completed vehicles
remain stationary collision obstacles.

## Path planning and DWA

`ResetPath` owns only geometry, the frozen original target, and arrival tests.
Visibility-graph Dijkstra plans around inflated convex polygons. Inflation covers
the circumscribed robot footprint, path clearance, and half the path lookahead
for preferred corner tracking room. If that extra lookahead reserve prevents a
route, planning retries with the full footprint and path clearance; the hard
DWA margins stay unchanged. A path lookahead point is a geometric scoring reference;
it produces no nominal velocity or extra control phase.

`ResetDwa` samples body velocities from the intersection of chassis speed limits
and the previous command's acceleration window. Scout samples both positive and
negative `vx`, with `vy = 0`; Mecanum samples `vx`, `vy`, and yaw independently.
The candidate trajectories are scored directly against the path, heading, and
stopping objective. When the path lookahead reaches the original target, Scout
uses Reeds-Shepp distance to that complete pose as its terminal score. Both the
candidate and target are expressed at the estimated rotation center. The metric
accounts for forward/reverse nonholonomic maneuvers; DWA still selects each body
velocity and checks its footprint rollout. No Reeds-Shepp segments are executed.
The distance formulas are adapted from OMPL 1.4.2; see `THIRD_PARTY_NOTICES`.

Scout trajectory scoring uses successive existing poses to estimate the bounded
signed lateral/yaw coupling, without a new twist subscription or lateral
actuator. Ill-conditioned small turns are excluded. This calibrates the scoring
prediction; collision checks include zero and both signed coupling endpoints
for each Scout. Candidates are ranked by trajectory score before the full
collision rollout; the selected trajectory retains every safety check.

Native Reset has a fixed 90 s timeout, including the production Scout launch.
The per-cycle solve deadline is separate. The 45 s mathematical regression
scenarios remain a stricter convergence check, not the runtime timeout.

`planPeerPaths` recomputes complete routes around current fleet footprints
to the unchanged original targets. Transient target occupancy and close poses
that cannot fit the conservative planning inflation remain DWA constraints.
If no extra peer detour fits, the valid scene route remains the geometric
reference; DWA must still find an admissible command. Equal-length detours
prefer the robot's right. Relative-velocity heading scoring in DWA supports
crossing traffic without storing passage targets. Arrival only stops that owner:
the frozen cohort, other routes, and learned motion stay intact. Arrived peers
keep the same body collision checks at zero velocity. There is no polar velocity
law, common time scaling, CBF-QP, or Reset OSQP dependency.

## Scene and admissibility

The coordinator consumes `xgc2_geometry_msgs/SceneSnapshot` and `SceneState`
under `scene_namespace` (default `/xgc/scene`). An explicit empty snapshot is
valid; absent or stale scene data is not an empty scene. Epoch/revision changes
reject an active Reset. Live geometry motion invalidates cached paths.

Each compound part projects independently into 32 supporting XY halfspaces.
Curves and oriented meshes use conservative planar outer approximations;
compound openings are not filled by a shared hull. With no height envelope,
overhead geometry also occupies XY. This does not certify 3D passage.

Live scene occupancy includes the configured 2 s translational/rotational motion
envelope. Pause uses the live pose with zero twist. Unsupported motion or
geometry reports a capability gap. Stale, unordered, or mismatched state is
rejected. Consumer status distinguishes applied documents from operational
capability; `success` is an alias of `applied`.

Every candidate undergoes a 1 s chassis rollout in 0.1 s increments followed by
an acceleration-limited braking tail. Scene/fence checks use covering disks;
peer checks use predicted full rectangles. This is a discrete kinematic check,
not a hardware braking or continuous-time safety certificate. Future moving peer
predictions retain a 2 cm contact reserve. Actual rectangle penetration remains inadmissible, including for stationary vehicles.

No admissible sample means exact zero and `local_plan_feasible = false`; an
unrolled brake or shorter horizon cannot be substituted as a safe candidate.
Invalid inputs also fail closed. The owners enforce request generations, response
leases, measured arrival, and timeouts.

## Verification

The mathematical tests cover path validity, speed and acceleration limits,
forward/reverse choice, bounded-time straight return, blocked motion, obstacle
layouts, complete target poses, and Scout lateral coupling with actuator lag. Scenario
collision audits independently check physical rectangle footprints every 2 ms.
The four-Scout crossing/return regression uses a stricter 45 s convergence bound.
Production close-goal sweeps and the ROS transport initial pose use the same
45 s gate. Seeded-layout tests retain their arrival and collision assertions.
These finite tests do not establish arbitrary-fleet convergence or hardware
safety. ROS tests additionally exercise coordinator requests, leases, scene
updates, and arrival responses.
