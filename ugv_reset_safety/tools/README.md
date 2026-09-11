# Reset Monte Carlo validation

`reset_monte_carlo` reuses the production `ResetGuidance`, centralized `FleetGuidance`, `FleetSchedule`, and joint safety filter through the same closed-loop plant used by `reset_scenarios_test.cpp`. The plant integrates every 2 ms inside the 20 ms command period and audits full rectangular footprints independently of the controller's covering-disk approximation.

Build the workspace with testing enabled, then run a large sharded campaign:

```bash
python3 ugv_reset_safety/tools/run_reset_monte_carlo.py \
  --cases-per-mode 10000 \
  --shards 8 \
  --robots-min 1 --robots-max 6 \
  --obstacles-min 0 --obstacles-max 5 \
  --max-time 240 \
  --output-prefix /tmp/reset_mc
```

By default the driver runs Scout-only, Mecanum-only, and mixed fleets. Static convex obstacles, start/goal permutations, headings, Scout lateral coupling, and first-order linear/angular actuator lag are randomized from deterministic seed ranges. Each failed or non-completed case is emitted with its exact seed so it can be replayed with a one-case invocation.

The aggregate JSON separates `completed`, `no_route`, `timeout`, `filter_failure`, `collision`, and `contract_failure`. A physical rectangle collision or controller/plant contract violation makes the shard process fail. `no_route`, timeout, and QP infeasibility remain measured outcomes rather than being silently relabeled as successful reset.

This is an offline algorithm/plant stress test. It does not replace ROS transport tests, Gazebo timing, virtual-real integration, or physical-robot acceptance.
