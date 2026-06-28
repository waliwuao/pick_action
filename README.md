# pick_action — LiDAR-based Autonomous Pick Sequence

Self-contained monorepo. Everything needed to run the complete pick operation:
LiDAR driver + recognition + action server. Clone. Build. Run.

## Structure

```
pick_action/
├── src/
│   ├── pick_action/              # Python: recognition + action server
│   │   ├── action/               #  PickSequence.action
│   │   ├── srv/                  #  ToolAction.srv
│   │   ├── pick_action/          #  Python modules
│   │   ├── config/               #  YAML parameters
│   │   ├── launch/               #  Launch files
│   │   └── scripts/              #  Trigger / wrapper scripts
│   └── ldlidar_stl_ros2/         # C++: STL-27L vendor driver
├── .gitignore
└── README.md
```

## Requirements

- Ubuntu 24.04 / ROS 2 Jazzy
- STL-27L LiDAR + ARES R2 Tool MCU (hardware)

## Build

```bash
cd pick_action
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Usage

```bash
# Real LiDAR
ros2 launch pick_action pick_action.launch.py port_name:=/dev/ttyUSB0

# No hardware (synthetic scan)
ros2 launch pick_action pick_action.launch.py use_synthetic:=true

# Trigger
ros2 action send_goal /pick_action pick_action_interfaces/action/PickSequence \
  "{expected_count: 3}" --feedback
```

## Sequence (8 states)

| State | Action |
|---|---|
| `VALIDATING` | Wait for 3 stable targets |
| `ALIGN_X` | Align connector X via prepare service |
| `FORWARD` | Chassis forward 0.2 m/s x 4s |
| `GRASP` | Close gripper |
| `LIFT` | Raise chassis (publish lift_height_mm to /t0x0112_) |
| `RETREAT` | Chassis reverse 0.2 m/s x 2s |
| `LOWER` | Lower chassis |
| `DONE` | Complete |

Any step fails → aborted.

## Configuration

- `src/pick_action/config/recognition.yaml` — ROI, calibration, grid
- `src/pick_action/config/pick_action.yaml` — speeds, durations, offsets
