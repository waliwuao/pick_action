# ares_ws — ROS2 封装：控制 ARES R2 tool 的 connector

通过 ROS2 (Jazzy) 服务远程触发 MCU（`app/ares_r2_tool`，运行 ZephyrRTOS）上的对接机构
（connector / spear）动作。底层用 `app/ares_controller` 同款 C++ libusb 客户端，按
`ares_r2_tool` 的 **dual_protocol SYNC 协议**收发（语言与现有 example 一致）。

## 协议

- 指令帧（host → MCU）：`head=0x5A5A` + `id=动作对象` + `action`(uint32) + 4×`float32`=0，共 24 字节。
- 反馈帧（MCU → host）：同 `head/id`，`action` 回显刚完成的动作。
- 动作对象 `id`：`0x0204`=connector，`0x0203`=arm。

节点发出指令帧后**等待匹配 `action` 的反馈帧**（即等动作执行完）再返回，超时由参数
`completion_timeout_ms` 控制（默认 15000ms）。

> **保活**：下位机 dual_protocol 在 2s（`10×HEART_BEAT_DELAY`）无上行数据时会判定断连
> （`online=false`）并抑制完成帧。本节点是请求/响应模式（非连续流），所以在发指令前先
> 发一帧 no-op 保活并等一个心跳周期，且在等待完成期间每 500ms 发一帧保活，保证长动作
> （如 prepare ≈2.7s）执行完时仍能收到反馈帧。

## 包结构

- `ares_tool_interfaces` — 自定义服务接口 `srv/ToolAction.srv`（ament_cmake / rosidl）。
- `ares_tool_control` — C++ 节点（ament_cmake / rclcpp）：
  - `include/ares_tool_control/ares_usb.hpp` + `src/ares_usb.cpp`：libusb USB 传输层，
    发送 SYNC 指令帧并等待反馈帧。
  - `src/tool_node.cpp`：节点 `ares_tool_node`，提供单个分发服务。
  - `launch/tool_node.launch.py`：一键启动。

## 动作表

服务 `~/tool_action`（完整名 `/ares_tool_node/tool_action`），类型
`ares_tool_interfaces/srv/ToolAction`。请求字段：`action`（字符串）+ `args`（`float32[4]`，透传到帧的
4 个预留 float，动作自取所需位；不填默认全 0）。

| action | 说明 | id | payload action |
|---|---|---|---|
| `prepare` | 张开夹爪、解锁机构、移动到待抓取位姿；`args[0]`=length(m) 经线性拟合(0.067m→0°, 0.2975m→-770°)换算成 wye 角度 | 0x0204 | 1 |
| `grasp` | 夹爪闭合抓取矛头（气动待接入） | 0x0204 | 2 |
| `dock_extend` | 转动 roll 将矛头伸出到准备对接位置 | 0x0204 | 3 |
| `arm_grasp` | 机械臂抓取 KFS | 0x0203 | 1 |
| `arm_store_to_body` | 转存到车体储存位，再回空闲位 | 0x0203 | 2 |
| `arm_store_on_arm` | KFS 暂持在机械臂上 | 0x0203 | 3 |
| `arm_get_body` | 从车体取回 | 0x0203 | 4 |
| `arm_place_mid` | 放置到中位 | 0x0203 | 5 |
| `arm_place_high` | 放置到高位 | 0x0203 | 6 |

> 响应 `success=true / ret=0` 表示**收到完成反馈帧**（动作已执行完）；`ret=-110`(ETIMEDOUT)
> 表示已下发但超时未收到反馈；`ret=-5`(EIO) 为 USB 收发失败。
> `dock_release`(payload 4) 在固件里不由指令触发，故不在动作表内。

## 依赖

仅需 `libusb-1.0` 开发库（本机已就绪），rosdep key 为 `libusb-1.0`：

```bash
rosdep install --from-paths src --ignore-src -r -y   # 或: sudo apt install libusb-1.0-0-dev
```

## 构建

```bash
cd app/ares_ws
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

## 运行

```bash
ros2 run ares_tool_control tool_node
# 或
ros2 launch ares_tool_control tool_node.launch.py
```

无硬件时节点可正常启动；调用服务时会回 `USB device not available`。

## 调用示例

```bash
ros2 service call /ares_tool_node/tool_action ares_tool_interfaces/srv/ToolAction "{action: 'prepare'}"
ros2 service call /ares_tool_node/tool_action ares_tool_interfaces/srv/ToolAction "{action: 'grasp'}"
ros2 service call /ares_tool_node/tool_action ares_tool_interfaces/srv/ToolAction "{action: 'dock_extend'}"
```

## USB 权限

设备 VID `0x1209`。免 `sudo` 运行需加 udev 规则，例如
`/etc/udev/rules.d/99-ares.rules`：

```
SUBSYSTEM=="usb", ATTR{idVendor}=="1209", ATTR{idProduct}=="0001", MODE="0666"
```

然后 `sudo udevadm control --reload-rules && sudo udevadm trigger`。
