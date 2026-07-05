# pick_action

ROS 2 Jazzy monorepo for an autonomous pick sequence:

`validate target -> align -> forward -> grasp -> lift -> retreat -> lower -> done`

The action server supports two alignment modes:

- `lidar_recognition`: uses 2D LiDAR recognition results from `/spear_recognition/result`.
- `odin_sensor_projection`: uses Odin pose plus distance sensors 3 and 5, computes the gripper pose, projects a configured target point onto the gripper yaw line, and uses that projected offset for alignment.

## Packages

| Package | Type | Role |
|---|---|---|
| `pick_action_interfaces` | `ament_cmake` | `action/PickSequence.action` |
| `ares_tool_interfaces` | `ament_cmake` | `srv/ToolAction.srv` client interface for `/ares_tool_node/tool_action` |
| `ldlidar_stl_ros2` | `ament_cmake` | STL-27L/LD06/LD19 LiDAR driver, publishes `/scan` |
| `pick_action` | `ament_python` | Recognition node, action server, synthetic scan node, trigger CLI |

The real `ares_tool_control` node is not in this repository. It must run in the ARES workspace and provide `/ares_tool_node/tool_action`.

## Build

```bash
cd /home/gsp/pick_action
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Install dependencies once if needed:

```bash
rosdep install --from-paths src --ignore-src -r -y
```

## Configuration

Main config files:

- `src/pick_action/config/pick_action.yaml`: default action-server config, mode is `lidar_recognition`.
- `src/pick_action/config/odin_sensor_projection.yaml`: ready-to-edit config for Odin + sensor projection mode.
- `src/pick_action/config/recognition.yaml`: LiDAR ROI, calibration, temporal grid recognition parameters.

The mode is selected by the action-server parameter:

```yaml
alignment_mode: lidar_recognition
```

or:

```yaml
alignment_mode: odin_sensor_projection
```

The launch file accepts a `pick_config` argument, so the easiest way to switch mode is to pass a different YAML file.

## Run: LiDAR Recognition Mode

This is the default mode. It uses `/scan -> recognition_node -> /spear_recognition/result`.

With real LiDAR:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch pick_action pick_action.launch.py port_name:=/dev/ttyUSB0
```

Without LiDAR hardware, use synthetic scan:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch pick_action pick_action.launch.py use_synthetic:=true
```

Then trigger:

```bash
ros2 action send_goal /pick_action pick_action_interfaces/action/PickSequence \
  "{expected_count: 3}" --feedback
```

In this mode, `VALIDATING` waits for `recognized_count == expected_count`, then selects the target with the smallest `abs(x_m)`. `ALIGN_X` sends `prepare(length)` where:

```text
error_x = 0.0 - target_x_m
length = direction_sign_x * error_x + prepare_offset_m
```

## Run: Odin + Sensor Projection Mode

This mode does not use the 2D LiDAR recognition result for alignment. It reads:

- `/sensor_distances`: `std_msgs/Float32MultiArray`
- `/odin1/relocation`: `geometry_msgs/PoseStamped`

It uses sensor indexes 3 and 5 by default:

```yaml
sensor_3_index: 3
sensor_5_index: 5
```

Edit `src/pick_action/config/odin_sensor_projection.yaml` before running:

```yaml
field_origin_x_m: 0.0
field_origin_y_m: 0.0
gripper_forward_m: 0.0
gripper_left_m: 0.0
gripper_yaw_offset_rad: 0.0

projection_target_x_m: 0.0
projection_target_y_m: 0.0
projection_direction_sign: 1.0
```

The target point must be in the same field frame used by `scripts/correct_pose.py`. The server computes:

1. Odin `x/y/yaw` from `/odin1/relocation`.
2. Sensor 3 and 5 distances from `/sensor_distances`.
3. Corrected robot pose using the model from `scripts/correct_pose.py`.
4. Corrected gripper `x/y/yaw`.
5. A line from the gripper pose along `gripper_yaw_rad`.
6. The projection of `(projection_target_x_m, projection_target_y_m)` onto that line.
7. `along_offset_m`, used by `ALIGN_X`.

Run with the projection config:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch pick_action pick_action.launch.py \
  pick_config:=/home/gsp/pick_action/src/pick_action/config/odin_sensor_projection.yaml
```

Then trigger the same action:

```bash
ros2 action send_goal /pick_action pick_action_interfaces/action/PickSequence \
  "{expected_count: 3}" --feedback
```

In this mode, `expected_count` is ignored by validation. It is kept only because the action definition is shared with LiDAR mode.

`ALIGN_X` sends `prepare(length)` where:

```text
error_x = along_offset_m
length = projection_direction_sign * error_x + prepare_offset_m
```

Tune `projection_direction_sign` if the mechanism moves in the opposite direction.

## Action States

| State | LiDAR mode | Odin + sensor projection mode |
|---|---|---|
| `VALIDATING` | Waits for `/spear_recognition/result` with expected target count | Waits for fresh Odin pose and sensor 3/5 distances |
| `ALIGN_X` | Aligns to selected LiDAR target x | Aligns using target projection offset on gripper yaw line |
| `FORWARD` | Publishes timed chassis velocity to `/t0x0111_` | Same |
| `GRASP` | Calls `/ares_tool_node/tool_action` with `action='grasp'` | Same |
| `LIFT` | Publishes `lift_height_mm` to `/t0x0112_` | Same |
| `RETREAT` | Publishes timed reverse chassis velocity to `/t0x0111_` | Same |
| `LOWER` | Publishes `lower_height_mm` to `/t0x0112_` | Same |
| `DONE` | Succeeds the action | Same |

Any failed service call, timeout, or cancellation aborts the action.

## Topics and Services

| Name | Type | Direction | Notes |
|---|---|---|---|
| `/pick_action` | `pick_action_interfaces/action/PickSequence` | action server | Main trigger |
| `/spear_recognition/result` | `std_msgs/String` JSON | subscription | Used in LiDAR mode |
| `/spear_recognition/markers` | `visualization_msgs/MarkerArray` | publication | RViz recognition visualization |
| `/sensor_distances` | `std_msgs/Float32MultiArray` | subscription | Used in Odin projection mode |
| `/odin1/relocation` | `geometry_msgs/PoseStamped` | subscription | Used in Odin projection mode |
| `/ares_tool_node/tool_action` | `ares_tool_interfaces/srv/ToolAction` | client | `prepare`, `grasp` |
| `/t0x0111_` | `std_msgs/Float32MultiArray` | publication | Chassis velocity |
| `/t0x0112_` | `std_msgs/Float32MultiArray` | publication | Lift/lower heights |
| `/pick_action/status` | `std_msgs/String` JSON | publication | Current state and target/alignment data |

In `odin_sensor_projection` mode, `/pick_action/status` includes extra fields such as `sensor_3_mm`, `sensor_5_mm`, `odin_x_m`, `odin_y_m`, `gripper_x_m`, `gripper_y_m`, `projection_x_m`, `projection_y_m`, `along_offset_m`, and `lateral_error_m`.

## Useful Checks

Check action server parameters:

```bash
ros2 param get /pick_action_server alignment_mode
ros2 param get /pick_action_server projection_target_x_m
ros2 param get /pick_action_server projection_target_y_m
```

Watch status:

```bash
ros2 topic echo /pick_action/status
```

Check Odin and sensor input:

```bash
ros2 topic echo /sensor_distances
ros2 topic echo /odin1/relocation
```

## Notes

- Ubuntu 24.04 / ROS 2 Jazzy is the supported target.
- There are no automated tests in this repo; `test_lift.py` is a manual hardware utility.
- `setup.cfg` installs console scripts to `$base/lib/pick_action`, which is normal for ROS 2 Python packages.
