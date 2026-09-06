# Flatness reference replay

Build `flatness_reference_replay` with this package's catkin test targets. It
links the production `computeFlatnessCommand` and pose velocity filter. Source
the matching catkin workspace so an older installed runtime library cannot
shadow the build. This program starts no ROS node or robot.

Input is headerless CSV: receipt time, world x/y, vx/vy, ax/ay. Use a single
robot's recorded `alg/reference/pva`, in increasing receipt-time order. It
starts at the first reference position with zero speed and yaw zero; therefore
use recordings with that initial condition. Columns and units are documented
in the source. Example arguments after the CSV are `kp kv delay_s tau_s
plant_yaw_limit`:

```
flatness_reference_replay pva.csv 6 4 0 0 1.05 > ideal.csv
flatness_reference_replay pva.csv 6 4 0.15 0.15 0.5235 > delayed.csv
```

The diagnostic plant is an ideal unicycle with optional pure command delay,
first-order velocity response and yaw-command limit. Controller and pose
updates use 4 ms steps. This intentionally simplified comparison omits skid,
wheel PI, contact loads, network jitter and the real publish schedule. It can
expose sensitivity to lag and saturation; it cannot identify a physical plant,
prove a Gazebo root cause, or establish closed-loop safety. The recorded PVA is
held fixed, so this is a tracking-layer replay, not a new DMPC closed loop.
