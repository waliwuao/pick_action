# AGENTS.md — pick_action

ROS 2 Jazzy monorepo for a LiDAR-based autonomous pick sequence (scan → recognize → align → grasp → lift → retreat → done).

## Setup & Build

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install       # builds all 4 packages; order auto-resolved
source install/setup.bash
```

**Dependencies** (one-time):
```bash
rosdep install --from-paths src --ignore-src -r -y
```

## Package Map (4 packages under src/)

| Package | Build Type | Role |
|---|---|---|
| `pick_action_interfaces` | ament_cmake | IDL: `action/PickSequence.action` |
| `ares_tool_interfaces` | ament_cmake | IDL: `srv/ToolAction.srv` (connector/arm commands) |
| `ldlidar_stl_ros2` | ament_cmake (C++) | STL-27L/LD06/LD19 LiDAR driver — publishes `sensor_msgs/LaserScan` |
| `pick_action` | ament_python (Python) | Main app: recognition node + action server + synthetic scan + trigger CLI |

`pick_action` depends on both interface packages — they must build first (colcon handles this).

## Run

```bash
# Real LiDAR hardware
ros2 launch pick_action pick_action.launch.py port_name:=/dev/ttyUSB0

# No hardware — synthetic scan for development
ros2 launch pick_action pick_action.launch.py use_synthetic:=true

# Trigger manually
ros2 action send_goal /pick_action pick_action_interfaces/action/PickSequence "{expected_count: 3}" --feedback
```

## Key Gotchas

- **`ares_tool_control` C++ node is NOT in this repo.** It lives in a separate workspace (`ares_ws`) and exposes the `/ares_tool_node/tool_action` service. This repo provides only the `ares_tool_interfaces` IDL package so `pick_action` can compile against it. At runtime the service is called over network.
- **No real tests exist.** `python3-pytest` is declared as a test dependency but there are zero test files. `test_lift.py` is a manual hardware-test utility, not an automated test.
- **`setup.cfg` installs console scripts to `$base/lib/pick_action`** instead of the typical `bin/` prefix. The `--symlink-install` flag keeps them linked to source.
- **Configuration is in `src/pick_action/config/*.yaml`** — parameters are loaded via ROS parameter YAML, not code defaults.
- **Ubuntu 24.04 / ROS 2 Jazzy only** — no other distro/ROS version is supported.
- **No CI, no linting/formatting config, no pre-commit hooks** in this repo.

## Development Without Hardware

Use `use_synthetic:=true` in the launch file. The `synthetic_scan_node` publishes a reproducible fake `/scan` topic with configurable targets. Recognition and action server exercise the full pipeline against synthetic data.

## Action Server States

8-state sequence: `VALIDATING → ALIGN_X → FORWARD → GRASP → LIFT → RETREAT → LOWER → DONE`. Any step failure aborts.
