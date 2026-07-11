# pick_action

这是一个 ROS 2 Jazzy 工作空间，用来执行自动夹取流程：

```text
VALIDATING -> ALIGN_X -> FORWARD -> GRASP -> LIFT -> RETREAT -> LOWER -> DONE
```

当前支持 4 种模式：

| 模式 | `alignment_mode` | 用途 |
|---|---|---|
| 2D 雷达识别模式 | `lidar_recognition` | 用 `/scan` 识别目标，再用 `prepare` 做横向对齐 |
| Odin + 单点传感器纠错模式 | `odin_sensor_projection` | 用 `/sensor_distances` 和 `/odin1/relocation` 计算夹爪纠错位移，再用 `prepare` 对齐 |
| 无对齐模式 | `no_alignment` | 不读取识别/Odin/传感器数据，不纠错，不 `prepare`，直接前进夹取 |
| 单点测距扫描无对齐模式 | `sensor_scan_no_alignment` | 先用 1 号单点测距传感器做夹爪水平慢扫，检测突变后再前进夹取 |

真实的 `/ares_tool_node/tool_action` 节点不在本仓库里，需要在 ARES 工作空间中启动。本仓库只包含 `ares_tool_interfaces` 接口包，供 `pick_action` 编译和调用。

## 编译

```bash
cd /home/gsp/pick_action
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

如需安装依赖：

```bash
rosdep install --from-paths src --ignore-src -r -y
```

## 配置文件

现在只使用一份主配置：

```text
src/pick_action/config/pick_action.yaml
```

这份 YAML 同时配置：

- `pick_action_server`
- `spear_recognition`

也就是说，之前分散的 `recognition.yaml` 和 Odin 模式配置已经合并到 `pick_action.yaml` 里。

## 如何切换模式

修改 `src/pick_action/config/pick_action.yaml` 里的：

```yaml
alignment_mode: no_alignment
```

可选值：

```yaml
alignment_mode: lidar_recognition
alignment_mode: odin_sensor_projection
alignment_mode: no_alignment
alignment_mode: sensor_scan_no_alignment
```

当前默认已经设置为：

```yaml
alignment_mode: sensor_scan_no_alignment
```

## 启动方式

### 1. 启动主流程

真实 2D 雷达：

```bash
cd /home/gsp/pick_action
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch pick_action pick_action.launch.py port_name:=/dev/ttyUSB0
```

无 2D 雷达硬件时使用模拟 `/scan`：

```bash
cd /home/gsp/pick_action
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch pick_action pick_action.launch.py use_synthetic:=true
```

如果当前是 `odin_sensor_projection`、`no_alignment` 或 `sensor_scan_no_alignment`，2D 雷达识别结果不会决定对齐逻辑；但 launch 仍会启动识别节点。

### 2. 触发夹取

```bash
ros2 action send_goal /pick_action pick_action_interfaces/action/PickSequence \
  "{expected_count: 3}" --feedback
```

说明：

- 在 `lidar_recognition` 模式下，`expected_count` 会用于等待识别到指定数量目标。
- 在 `odin_sensor_projection`、`no_alignment` 和 `sensor_scan_no_alignment` 模式下，`expected_count` 基本只是 action 接口保留字段。

## 模式 1：2D 雷达识别模式

配置：

```yaml
alignment_mode: lidar_recognition
```

数据来源：

| 名称 | 类型 | 作用 |
|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | 2D 雷达原始扫描 |
| `/spear_recognition/result` | `std_msgs/String` JSON | 识别节点输出的目标 |

流程：

```text
VALIDATING:
  等待 /spear_recognition/result 中 recognized_count == expected_count
  选择 abs(x_m) 最小的目标

ALIGN_X:
  根据目标 x_m 计算 prepare(length)

FORWARD -> GRASP -> LIFT -> RETREAT -> LOWER -> DONE
```

相关参数：

```yaml
result_topic: /spear_recognition/result
direction_sign_x: -1.0
deadband_x_m: 0.005
prepare_base_length_m: 0.3
prepare_min_length_m: 0.0
prepare_max_length_m: 0.5
prepare_timeout_ms: 20000
```

`ALIGN_X` 的计算逻辑：

```text
error_x = 0.0 - target_x_m
length = direction_sign_x * error_x + prepare_base_length_m
```

`length` 会被限制在：

```text
prepare_min_length_m <= length <= prepare_max_length_m
```

识别节点参数也在同一个 YAML 里，位于：

```yaml
spear_recognition:
  ros__parameters:
    input_topic: /scan
    range_min_m: 0.05
    range_max_m: 0.43
    angle_min_deg: 10.0
    angle_max_deg: 170.0
    x_min_m: -0.43
    x_max_m: 0.43
    y_min_m: 0.05
    y_max_m: 0.43
```

## 模式 2：Odin + 单点传感器纠错模式

配置：

```yaml
alignment_mode: odin_sensor_projection
```

数据来源：

| 名称 | 类型 | 作用 |
|---|---|---|
| `/sensor_distances` | `std_msgs/Float32MultiArray` | 8 个单点测距值，单位 mm |
| `/odin1/relocation` | `geometry_msgs/PoseStamped` | 机器人原始 Odin 位姿 |

默认使用：

```yaml
sensor_3_index: 3
sensor_5_index: 5
```

流程：

```text
VALIDATING:
  等待新鲜的 /sensor_distances 和 /odin1/relocation
  使用传感器 3/5 + Odin yaw 计算纠错后的机器人位姿
  计算夹爪在蓝场坐标系下的 x/y/yaw
  根据目标点投影，得到夹爪需要移动的 gripper_forward_move_m

ALIGN_X:
  length = prepare_base_length_m + gripper_forward_move_m
  调用 /ares_tool_node/tool_action: action='prepare'

FORWARD -> GRASP -> LIFT -> RETREAT -> LOWER -> DONE
```

相关参数：

```yaml
sensor_topic: /sensor_distances
pose_topic: /odin1/relocation
sensor_count: 8
sensor_3_index: 3
sensor_5_index: 5
sensor_max_age_s: 0.5
pose_max_age_s: 0.5
```

坐标系和夹爪参数：

```yaml
field_origin_x_m: -0.4
field_origin_y_m: -1.25
gripper_forward_m: -0.5411111323
gripper_left_m: 0.0342431067
gripper_yaw_offset_rad: -1.5707963268
target_x_m: 1.05
target_y_m: -0.15
gripper_move_direct: -1.0
```

含义：

| 参数 | 含义 |
|---|---|
| `field_origin_x_m` / `field_origin_y_m` | 蓝场坐标系原点在原始 Odin 坐标系下的位置 |
| `gripper_forward_m` / `gripper_left_m` | 夹爪点相对 Odin 机器人点 S 的位置，机器人本体系，单位 m |
| `gripper_yaw_offset_rad` | 夹爪 yaw 相对 Odin yaw 的偏移 |
| `target_x_m` / `target_y_m` | 要夹取的目标点，蓝场坐标系下，单位 m |
| `gripper_move_direct` | 平移方向修正，方向反了就改成 `1.0` 或 `-1.0` |

夹爪长度控制参数：

```yaml
prepare_base_length_m: 0.3
prepare_min_length_m: 0.0
prepare_max_length_m: 0.5
deadband_x_m: 0.005
prepare_timeout_ms: 20000
```

计算逻辑：

```text
raw_projection_distance = 目标点投影到夹爪 yaw 直线后的有符号距离
gripper_forward_move_m = gripper_move_direct * raw_projection_distance
length = prepare_base_length_m + gripper_forward_move_m
```

也就是说，硬件收到的不是“位移”，而是 `prepare` 的目标长度：

```text
prepare(length)
```

例如：

```text
prepare_base_length_m = 0.3
gripper_forward_move_m = -0.15
length = 0.15
```

## 模式 3：无对齐模式

配置：

```yaml
alignment_mode: no_alignment
```

这个模式是最快路径：

```text
不等待 /spear_recognition/result
不读取 /sensor_distances
不读取 /odin1/relocation
不计算纠错
不执行 ALIGN_X
不调用 prepare 对齐
```

流程：

```text
VALIDATING:
  只发布状态，不等待识别/Odin/传感器

FORWARD -> GRASP -> LIFT -> RETREAT -> LOWER -> DONE
```

会使用的参数：

```yaml
forward_speed_mps: 0.2
forward_duration_s: 1.0
direction_sign_y: -1.0
grasp_timeout_ms: 15000
lift_height_mm: [70.0, 70.0, 70.0, 70.0]
lower_height_mm: [28.0, 28.0, 28.0, 28.0]
retreat_speed_mps: 0.2
retreat_duration_s: 1.0
publish_rate_hz: 100.0
```

不会使用的参数：

```yaml
result_topic
sensor_topic
pose_topic
field_origin_x_m
field_origin_y_m
gripper_forward_m
gripper_left_m
gripper_yaw_offset_rad
target_x_m
target_y_m
gripper_move_direct
prepare_base_length_m
prepare_min_length_m
prepare_max_length_m
deadband_x_m
prepare_timeout_ms
```

## 模式 4：单点测距扫描无对齐模式

配置：

```yaml
alignment_mode: sensor_scan_no_alignment
```

这个模式以 `TreeAction` 的 spear 控制语义为准：

```text
prepare args[0] = debug_scan_target_position_m
prepare args[1] = scan_prepare_speed_rpm
```

也就是让夹爪机构按配置目标位置和速度做水平移动扫描。它不读取 Odin，也不等待 2D 雷达识别；只读取 `/sensor_distances` 中配置的传感器，默认是索引 `1`。

流程：

```text
VALIDATING:
  只发布状态，不等待识别/Odin

SENSOR_SCAN:
  调用 prepare([debug_scan_target_position_m, scan_prepare_speed_rpm]) 直接开始慢速扫描
  读取 /sensor_distances[scan_sensor_index]
  如果测距 <= scan_present_threshold_mm 并持续 scan_present_duration_s
    认为夹爪已经在物体前方，停止慢移并进入 FORWARD
  如果相邻有效测距差值 >= scan_jump_threshold_mm
    继续慢移 scan_center_extra_time_s
    调用 scan_stop_action 停止慢移

FORWARD -> GRASP -> LIFT -> RETREAT -> LOWER -> DONE
```

在 `sensor_scan_no_alignment` 模式下，如果扫描已检测到目标并成功停止慢移，进入 `GRASP` 前会先等待 `scan_stop_to_grasp_delay_s`。如果第一次 `grasp` 返回超时，会等待 `grasp_retry_delay_s` 后只重试一次；第二次仍失败则本次 action 失败。

相关参数：

```yaml
scan_sensor_index: 1
scan_sensor_max_age_s: 0.5
scan_enable_jump_trigger: false
scan_jump_threshold_mm: 300.0
scan_present_threshold_mm: 300.0
scan_present_duration_s: 0.02
debug_scan_target_position_m: 0.45
scan_prepare_length_m: 0.05
scan_prepare_speed_rpm: 100.0
scan_center_extra_time_s: 0.0
scan_timeout_s: 5.0
scan_sample_period_s: 0.02
scan_stop_action: prepare
scan_stop_args: [0.0, 0.0]
scan_stop_timeout_ms: 3000
scan_stop_to_grasp_delay_s: 0.2
grasp_retry_delay_s: 0.1
```

含义：

| 参数 | 含义 |
|---|---|
| `scan_sensor_index` | 用哪个单点测距值扫描，默认 `1` |
| `scan_enable_jump_trigger` | 是否启用相邻距离突变触发；默认关闭 |
| `scan_jump_threshold_mm` | 相邻有效距离突变阈值，单位 mm |
| `scan_present_threshold_mm` | 已在物体前方的距离阈值，单位 mm |
| `scan_present_duration_s` | 测距持续低于阈值多久后直接触发后续动作 |
| `debug_scan_target_position_m` | 扫描启动时传给 `prepare` 的 `args[0]`，单位 m |
| `scan_prepare_length_m` | 备用参数，当前主流程不使用 |
| `scan_prepare_speed_rpm` | 水平慢移速度，传给 `prepare` 的 `args[1]` |
| `scan_center_extra_time_s` | 检测到突变后继续慢移的时间，用来移动到物体中心 |
| `scan_stop_action` | 停止持续慢移的 spear 命令，默认 `prepare` |
| `scan_stop_args` | 停止命令参数，默认 `[0.0, 0.0]`，表示停止水平慢移 |
| `scan_timeout_s` | 最长扫描时间，超时则本次 action 失败 |
| `scan_stop_to_grasp_delay_s` | 扫描检测到目标并成功停止后，进入 `grasp` 前等待多久 |
| `grasp_retry_delay_s` | 第一次 `grasp` 返回超时时，重试前等待多久；只重试一次 |

## 四种模式状态对比

| 状态 | `lidar_recognition` | `odin_sensor_projection` | `no_alignment` | `sensor_scan_no_alignment` |
|---|---|---|---|---|
| `VALIDATING` | 等待 2D 雷达识别结果 | 等待 Odin 和传感器数据 | 只发布状态 | 只发布状态 |
| `SENSOR_SCAN` | 无 | 无 | 无 | 1 号传感器水平慢扫 |
| `ALIGN_X` | 根据识别目标 x 对齐 | 根据纠错平移量对齐 | 跳过 | 跳过 |
| `FORWARD` | 前进 | 前进 | 前进 | 前进 |
| `GRASP` | 夹取 | 夹取 | 夹取 | 夹取 |
| `LIFT` | 抬升 | 抬升 | 抬升 | 抬升 |
| `RETREAT` | 后退 | 后退 | 后退 | 后退 |
| `LOWER` | 下降 | 下降 | 下降 | 下降 |
| `DONE` | 完成 | 完成 | 完成 | 完成 |

## 通用运动和夹取参数

这些参数四个模式都会用到：

```yaml
tool_service: /ares_tool_node/tool_action
chassis_topic: /t0x0111_
lift_topic: /t0x0112_
status_topic: /pick_action/status

forward_speed_mps: 0.2
forward_duration_s: 1.0
direction_sign_y: -1.0

grasp_timeout_ms: 15000
grasp_retry_delay_s: 0.1

lift_height_mm: [70.0, 70.0, 70.0, 70.0]
lower_height_mm: [28.0, 28.0, 28.0, 28.0]

retreat_speed_mps: 0.2
retreat_duration_s: 1.0

publish_rate_hz: 100.0
```

`grasp` 超时重试说明：

```text
第一次 grasp 成功 -> 继续 LIFT
第一次 grasp 失败且不是超时 -> action 失败
第一次 grasp 返回超时 -> 等待 grasp_retry_delay_s 后再发一次 grasp
第二次 grasp 成功 -> 继续 LIFT
第二次 grasp 失败 -> action 失败，不再重试
```

超时判断包括客户端等待 service 响应超时、服务返回 `ret == -ETIMEDOUT`，以及返回消息中包含 `timeout` / `timed out` / `time out` / `超时`。

## 调试日志

工具调用详细日志可通过参数开关控制：

```yaml
tool_debug_log_enabled: true
tool_debug_log_path: pick_action_tool_debug.jsonl
```

说明：

| 参数 | 含义 |
|---|---|
| `tool_debug_log_enabled` | 调试时设为 `true` 写入工具调用日志；比赛时可设为 `false`，避免文件 I/O |
| `tool_debug_log_path` | 工具调用日志文件路径；相对路径会写到启动节点时的当前工作目录 |

日志内容是 JSONL，每一行是一条工具调用事件，包含 `action`、`args`、`success`、`ret`、`message`、`timeout_s`、`timed_out` 和 `detail` 等字段。即使关闭文件日志，节点仍会在内存中保留最近一次工具调用结果，用于判断 `grasp` 是否需要超时重试。

## Topic 和 Service

| 名称 | 类型 | 方向 | 使用场景 |
|---|---|---|---|
| `/pick_action` | `pick_action_interfaces/action/PickSequence` | action server | 四种模式都用 |
| `/ares_tool_node/tool_action` | `ares_tool_interfaces/srv/ToolAction` | client | `prepare`、`grasp` |
| `/t0x0111_` | `std_msgs/Float32MultiArray` | publish | 底盘前进/后退 |
| `/t0x0112_` | `std_msgs/Float32MultiArray` | publish | 抬升/下降 |
| `/pick_action/status` | `std_msgs/String` JSON | publish | 状态输出 |
| `/scan` | `sensor_msgs/LaserScan` | subscribe | 2D 雷达识别模式 |
| `/spear_recognition/result` | `std_msgs/String` JSON | subscribe | 2D 雷达识别模式 |
| `/spear_recognition/markers` | `visualization_msgs/MarkerArray` | publish | 2D 雷达识别可视化 |
| `/sensor_distances` | `std_msgs/Float32MultiArray` | subscribe | Odin + 单点传感器纠错模式、单点测距扫描无对齐模式 |
| `/odin1/relocation` | `geometry_msgs/PoseStamped` | subscribe | Odin + 单点传感器纠错模式 |

## 常用检查命令

查看当前模式：

```bash
ros2 param get /pick_action_server alignment_mode
```

查看状态：

```bash
ros2 topic echo /pick_action/status
```

检查工具服务：

```bash
ros2 service list | grep /ares_tool_node/tool_action
```

检查 Odin 和单点传感器：

```bash
ros2 topic echo /sensor_distances
ros2 topic echo /odin1/relocation
```

检查 2D 雷达识别：

```bash
ros2 topic echo /scan
ros2 topic echo /spear_recognition/result
```

## 注意事项

- 支持环境：Ubuntu 24.04 / ROS 2 Jazzy。
- `no_alignment` 模式最快，但不会做任何横向对齐或纠错，前进时间和初始位置要靠你保证。
- `sensor_scan_no_alignment` 会在前进前做一次夹爪水平慢扫，但不会读取 Odin，也不会执行 `ALIGN_X` 对齐。
- `odin_sensor_projection` 模式会使用 `sensor_3_index` 和 `sensor_5_index`，传感器 topic 中的数据单位是 mm。
- `prepare(length)` 控制的是夹爪目标长度，不是直接位移；默认基准长度是 `0.3m`，允许范围是 `0.0m ~ 0.5m`。
- `test_lift.py` 是手动硬件测试工具，不是自动化测试。
- `setup.cfg` 会把 console scripts 安装到 `$base/lib/pick_action`，这是 ROS 2 Python 包的常见布局。
