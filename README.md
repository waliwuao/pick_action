# pick_action — LiDAR-based Autonomous Pick Sequence

Self-contained ROS 2 Action that orchestrates a complete pick operation:
LiDAR recognition → X-alignment → forward approach → grasp → lift → retreat → lower.

Includes its own recognition pipeline, tool service interface, and synthetic scan
for development. Only requires the STL-27L vendor driver (`ldlidar_stl_ros2`) at
runtime — no other external packages needed.

## Structure

```
pick_action/
├── action/PickSequence.action      # Action interface (Goal/Result/Feedback)
├── srv/ToolAction.svr             # Tool control service interface (prepare/grasp)
├── pick_action/
│   ├── __init__.py
│   ├── core.py                    # Scan filtering, clustering, calibration
│   ├── temporal_recognition.py    # Multi-frame voting, target scoring
│   ├── recognition_node.py        # ROS 2 recognition node
│   ├── synthetic_scan_node.py     # Fake scan for no-hardware dev
│   └── pick_action_server.py      # Action server (8-state machine)
├── config/
│   ├── pick_action.yaml           # Pick sequence parameters
│   └── recognition.yaml           # Recognition parameters
├── launch/
│   └── pick_action.launch.py      # Launch: driver + recognition + action server
├── scripts/
│   ├── pick_action_server_node    # Server entry point
│   ├── recognition_node           # Recognition entry point
│   ├── synthetic_scan_node        # Synthetic scan entry point
│   └── trigger_pick.py            # Action client (blocks until done)
├── CMakeLists.txt
├── package.xml
└── README.md
```

## Requirements

- Ubuntu 24.04 / ROS 2 Jazzy
- `ldlidar_stl_ros2` package (STL-27L vendor driver)
- STL-27L LiDAR + ARES R2 Tool MCU (for hardware operation)

## Build

```bash
# Standalone build (alongside ldlidar_stl_ros2 in a colcon workspace)
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to pick_action
source install/setup.bash
```

## Usage

```bash
# 1a. Launch with real LiDAR
ros2 launch pick_action pick_action.launch.py port_name:=/dev/ttyUSB0

# 1b. Launch with synthetic scan (no hardware needed)
ros2 launch pick_action pick_action.launch.py use_synthetic:=true

# 2. Trigger the sequence
python3 scripts/trigger_pick.py

# Or via CLI:
ros2 action send_goal /pick_action pick_action/action/PickSequence \
  "{expected_count: 3}" --feedback
```

## Sequence (8 states)

| Step | Feedback State | Action |
|---|---|---|
| 1 | `VALIDATING` | Wait for LiDAR recognition to report 3 stable targets |
| 2 | `ALIGN_X` | Align connector X via `{prepare}` service |
| 3 | `FORWARD` | Chassis forward 0.2 m/s x 4s on `/t0x0101_` |
| 4 | `GRASP` | Close gripper via `{grasp}` service |
| 5 | `LIFT` | Publish `[0.10,0.10,0.10,0.10]` on `/t0x0102_` |
| 6 | `RETREAT` | Chassis reverse 0.2 m/s x 2s on `/t0x0101_` |
| 7 | `LOWER` | Publish `[-0.10,-0.10,-0.10,-0.10]` on `/t0x0102_` |
| 8 | `DONE` | Goal succeeded |

Any step fails → goal aborted.

## Topics / Services

| Interface | Type | Direction |
|---|---|---|
| `/pick_action` | `PickSequence.action` | Action Server |
| `/scan` | `sensor_msgs/LaserScan` | Subscribed (recognition) |
| `/spear_recognition/result` | `std_msgs/String` (JSON) | Published (recognition) / Subscribed (server) |
| `/ares_tool_node/tool_action` | `ToolAction.srv` | Service Client |
| `/t0x0101_` | `std_msgs/Float32MultiArray` `[fwd, lat, 0]` | Published |
| `/t0x0102_` | `std_msgs/Float32MultiArray` `[leg1..leg4]` | Published |

## Configuration

- `config/recognition.yaml` — ROI, calibration, grid parameters
- `config/pick_action.yaml` — approach speeds, durations, direction signs, offsets
