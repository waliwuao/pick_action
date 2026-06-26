# pick_action — LiDAR-based Autonomous Pick Sequence

ROS 2 Action that orchestrates a complete pick operation:
LiDAR recognition → X-alignment → forward approach → grasp → lift → retreat → lower.

## Requirements

- Ubuntu 24.04 / ROS 2 Jazzy
- `2d-lidar` workspace (for `ldlidar_stl_ros2`, `spear_locator`, `ares_tool_interfaces`)
- STL-27L LiDAR + ARES R2 Tool MCU

## Build

```bash
cd ~/workspace/2d-lidar          # must be built alongside 2d-lidar
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to pick_action
source install/setup.bash
```

The `pick_action` repo should live alongside `2d-lidar`, and both have `colcon build` run from the parent workspace. The action interface depends on `ares_tool_interfaces`; the launch file references `spear_locator` and `ldlidar_stl_ros2`.

## Usage

```bash
# 1. Launch entire pipeline
ros2 launch pick_action pick_action.launch.py port_name:=/dev/ttyUSB0

# 2. Trigger the sequence (blocks until done)
python3 scripts/trigger_pick.py

# Or via CLI with feedback:
ros2 action send_goal /pick_action pick_action/action/PickSequence \
  "{expected_count: 3}" --feedback
```

## Sequence

| Step | State | Action |
|---|---|---|
| 1 | `VALIDATING` | Wait for LiDAR recognition to report 3 stable targets |
| 2 | `ALIGN_X` | Align connector X via `ares_tool_node/tool_action {prepare}` |
| 3 | `FORWARD` | Chassis forward 0.2 m/s × 4 s on `/t0x0101_` |
| 4 | `GRASP` | Close gripper via `{grasp}` |
| 5 | `LIFT` | Raise chassis: publish `[0.10, 0.10, 0.10, 0.10]` on `/t0x0102_` |
| 6 | `RETREAT` | Chassis reverse 0.2 m/s × 2 s on `/t0x0101_` |
| 7 | `LOWER` | Lower chassis: publish `[-0.10, -0.10, -0.10, -0.10]` on `/t0x0102_` |
| 8 | `DONE` | Goal succeeded |

Any step fails → goal aborted with error message.

## Configuration

Edit `config/pick_action.yaml` to adjust speeds, durations, direction signs, and offsets.

## Topics / Services

| Interface | Type | Direction |
|---|---|---|
| `/pick_action` | `PickSequence.action` | Action Server |
| `/spear_recognition/result` | `std_msgs/String` (JSON) | Subscribed |
| `/ares_tool_node/tool_action` | `ares_tool_interfaces/srv/ToolAction` | Service Client |
| `/t0x0101_` | `std_msgs/Float32MultiArray` `[fwd, lat, 0]` | Published |
| `/t0x0102_` | `std_msgs/Float32MultiArray` `[leg1..leg4]` | Published |
