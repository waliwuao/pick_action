#!/usr/bin/env python3
"""Interactive CLI debugger for the sensor-scan pick mode."""

from __future__ import annotations

import argparse
import math
import threading
import time
from pathlib import Path
from typing import Any

import rclpy
import yaml
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

try:
    from r2_interfaces.srv import ToolAction
except ImportError:
    from ares_tool_interfaces.srv import ToolAction


def _yes(text: str) -> bool:
    return text.strip().lower() in ('y', 'yes')


def _quit(text: str) -> bool:
    return text.strip().lower() in ('q', 'quit', 'exit')


def _ask(prompt: str, default: bool = False) -> bool:
    suffix = '[Y/n/q] ' if default else '[y/N/q] '
    answer = input(prompt + suffix)
    if _quit(answer):
        raise KeyboardInterrupt
    if not answer.strip():
        return default
    return _yes(answer)


def _package_config_path() -> Path:
    try:
        from ament_index_python.packages import get_package_share_directory

        return Path(get_package_share_directory('pick_action')) / 'config' / 'pick_action.yaml'
    except Exception:
        return Path(__file__).resolve().parents[1] / 'config' / 'pick_action.yaml'


def _load_params(path: str) -> dict[str, Any]:
    with open(path, 'r', encoding='utf-8') as handle:
        data = yaml.safe_load(handle) or {}
    return dict(data.get('pick_action_server', {}).get('ros__parameters', {}))


def _param(params: dict[str, Any], name: str, default: Any) -> Any:
    return params[name] if name in params else default


class SensorScanDebugCli(Node):
    def __init__(self, args: argparse.Namespace, params: dict[str, Any]) -> None:
        super().__init__('sensor_scan_debug_cli')
        self.args = args
        self.params = params
        self._lock = threading.Lock()
        self._latest_distances = [math.nan] * args.sensor_count
        self._latest_sensor_receive_time_s = math.nan

        self.create_subscription(
            Float32MultiArray,
            args.sensor_topic,
            self._sensor_callback,
            10,
        )
        self._chassis_pub = self.create_publisher(
            Float32MultiArray,
            args.chassis_topic,
            10,
        )
        self._lift_pub = self.create_publisher(
            Float32MultiArray,
            args.lift_topic,
            10,
        )
        self._tool_client = self.create_client(ToolAction, args.tool_service)

    def _sensor_callback(self, msg: Float32MultiArray) -> None:
        distances = [math.nan] * self.args.sensor_count
        for index in range(min(self.args.sensor_count, len(msg.data))):
            distances[index] = float(msg.data[index])
        with self._lock:
            self._latest_distances = distances
            self._latest_sensor_receive_time_s = self._now_s()

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def sensor_snapshot(self) -> tuple[float, float, list[float]]:
        now_s = self._now_s()
        with self._lock:
            distances = list(self._latest_distances)
            receive_time_s = self._latest_sensor_receive_time_s
        age_s = now_s - receive_time_s if math.isfinite(receive_time_s) else math.inf
        index = self.args.scan_sensor_index
        if index < 0 or index >= len(distances):
            return math.nan, math.inf, distances
        return distances[index], age_s, distances

    def is_valid_scan_distance(self, distance_mm: float, age_s: float) -> bool:
        return (
            math.isfinite(distance_mm)
            and math.isfinite(age_s)
            and age_s <= self.args.scan_sensor_max_age_s
            and distance_mm >= self.args.scan_min_valid_mm
            and distance_mm <= self.args.scan_max_valid_mm
        )

    def wait_for_tool_service(self) -> bool:
        return self._tool_client.wait_for_service(
            timeout_sec=self.args.service_wait_timeout_s
        )

    def start_tool_async(self, action: str, tool_args: list[float]):
        if not self.wait_for_tool_service():
            print('工具服务不可用: %s' % self.args.tool_service)
            return None

        req = ToolAction.Request()
        req.action = action
        req.args = tool_args[:4] + [0.0] * max(0, 4 - len(tool_args))
        print('发送异步工具命令: action=%s args=%s' % (req.action, req.args))
        return self._tool_client.call_async(req)

    def call_tool(self, action: str, tool_args: list[float], timeout_s: float) -> bool:
        if not self.wait_for_tool_service():
            print('工具服务不可用: %s' % self.args.tool_service)
            return False

        req = ToolAction.Request()
        req.action = action
        req.args = tool_args[:4] + [0.0] * max(0, 4 - len(tool_args))
        print('发送同步工具命令: action=%s args=%s timeout=%.3fs' % (
            req.action,
            req.args,
            timeout_s,
        ))
        future = self._tool_client.call_async(req)
        deadline_s = time.monotonic() + timeout_s
        while rclpy.ok() and not future.done() and time.monotonic() < deadline_s:
            time.sleep(0.02)
        if not future.done() or future.result() is None:
            print('工具命令超时')
            return False
        result = future.result()
        print('工具命令返回: success=%s ret=%d msg=%s' % (
            result.success,
            result.ret,
            result.message,
        ))
        return bool(result.success)

    def stop_scan_motion(self) -> bool:
        return self.call_tool(
            self.args.scan_stop_action,
            list(self.args.scan_stop_args),
            self.args.scan_stop_timeout_s,
        )

    def publish_chassis_for(self, speed: float, duration_s: float) -> None:
        period_s = 1.0 / self.args.publish_rate_hz
        msg = Float32MultiArray()
        msg.data = [float(speed), 0.0, 0.0]
        deadline_s = time.monotonic() + duration_s
        while time.monotonic() < deadline_s:
            self._chassis_pub.publish(msg)
            time.sleep(period_s)
        msg.data = [0.0, 0.0, 0.0]
        self._chassis_pub.publish(msg)

    def publish_height(self, heights: list[float]) -> None:
        msg = Float32MultiArray()
        msg.data = [float(value) for value in heights[:4]]
        print('发布高度: %s -> %s' % (self.args.lift_topic, msg.data))
        self._lift_pub.publish(msg)


def _parse_args() -> argparse.Namespace:
    defaults = _load_params(str(_package_config_path()))

    parser = argparse.ArgumentParser(
        description='交互式 debug 当前 sensor_scan_no_alignment 抓取流程。'
    )
    parser.add_argument(
        '--config',
        default=str(_package_config_path()),
        help='pick_action.yaml 路径',
    )
    known, _ = parser.parse_known_args()
    params = _load_params(known.config)

    parser.set_defaults(_params=params)
    parser.add_argument('--sensor-topic', default=_param(params, 'sensor_topic', '/sensor_distances'))
    parser.add_argument('--tool-service', default=_param(params, 'tool_service', '/ares_tool_node/tool_action'))
    parser.add_argument('--chassis-topic', default=_param(params, 'chassis_topic', '/t0x0111_'))
    parser.add_argument('--lift-topic', default=_param(params, 'lift_topic', '/t0x0112_'))
    parser.add_argument('--sensor-count', type=int, default=int(_param(params, 'sensor_count', 8)))
    parser.add_argument('--scan-sensor-index', type=int, default=int(_param(params, 'scan_sensor_index', 1)))
    parser.add_argument('--scan-sensor-max-age-s', type=float, default=float(_param(params, 'scan_sensor_max_age_s', 0.5)))
    parser.add_argument('--scan-min-valid-mm', type=float, default=float(_param(params, 'scan_min_valid_mm', 20.0)))
    parser.add_argument('--scan-max-valid-mm', type=float, default=float(_param(params, 'scan_max_valid_mm', 2000.0)))
    parser.add_argument('--scan-jump-threshold-mm', type=float, default=float(_param(params, 'scan_jump_threshold_mm', 300.0)))
    parser.add_argument('--scan-present-threshold-mm', type=float, default=float(_param(params, 'scan_present_threshold_mm', 150.0)))
    parser.add_argument('--scan-present-duration-s', type=float, default=float(_param(params, 'scan_present_duration_s', 0.1)))
    parser.add_argument('--scan-prepare-length-m', type=float, default=float(_param(params, 'scan_prepare_length_m', 0.05)))
    parser.add_argument('--scan-prepare-speed-rpm', type=float, default=float(_param(params, 'scan_prepare_speed_rpm', 30.0)))
    parser.add_argument('--scan-center-extra-time-s', type=float, default=float(_param(params, 'scan_center_extra_time_s', 0.05)))
    parser.add_argument('--scan-timeout-s', type=float, default=float(_param(params, 'scan_timeout_s', 5.0)))
    parser.add_argument('--scan-sample-period-s', type=float, default=float(_param(params, 'scan_sample_period_s', 0.02)))
    parser.add_argument('--scan-stop-action', default=str(_param(params, 'scan_stop_action', 'prepare')))
    parser.add_argument('--scan-stop-args', nargs=4, type=float, default=list(_param(params, 'scan_stop_args', [0.0, 0.0, 0.0, 0.0])))
    parser.add_argument('--scan-stop-timeout-s', type=float, default=float(_param(params, 'scan_stop_timeout_ms', 3000)) / 1000.0)
    parser.add_argument('--service-wait-timeout-s', type=float, default=3.0)
    parser.add_argument('--forward-speed-mps', type=float, default=float(_param(params, 'forward_speed_mps', 0.2)))
    parser.add_argument('--forward-duration-s', type=float, default=float(_param(params, 'forward_duration_s', 1.8)))
    parser.add_argument('--direction-sign-y', type=float, default=float(_param(params, 'direction_sign_y', -1.0)))
    parser.add_argument('--grasp-timeout-s', type=float, default=float(_param(params, 'grasp_timeout_ms', 15000)) / 1000.0)
    parser.add_argument('--lift-height-mm', nargs=4, type=float, default=list(_param(params, 'lift_height_mm', [70.0, 70.0, 70.0, 70.0])))
    parser.add_argument('--lower-height-mm', nargs=4, type=float, default=list(_param(params, 'lower_height_mm', [28.0, 28.0, 28.0, 28.0])))
    parser.add_argument('--retreat-speed-mps', type=float, default=float(_param(params, 'retreat_speed_mps', 0.2)))
    parser.add_argument('--retreat-duration-s', type=float, default=float(_param(params, 'retreat_duration_s', 1.0)))
    parser.add_argument('--publish-rate-hz', type=float, default=float(_param(params, 'publish_rate_hz', 100.0)))
    return parser.parse_args()


def _print_config(args: argparse.Namespace) -> None:
    print('\n当前 debug 配置:')
    print('  sensor_topic = %s' % args.sensor_topic)
    print('  scan_sensor_index = %d' % args.scan_sensor_index)
    print('  tool_service = %s' % args.tool_service)
    print('  scan start = prepare([%.4f, %.4f, 0.0, 0.0])' % (
        args.scan_prepare_length_m,
        args.scan_prepare_speed_rpm,
    ))
    print('  scan stop = %s(%s)' % (args.scan_stop_action, args.scan_stop_args))
    print('  present trigger: distance <= %.3f mm for %.3f s' % (
        args.scan_present_threshold_mm,
        args.scan_present_duration_s,
    ))
    print('  jump trigger: abs(delta) >= %.3f mm' % args.scan_jump_threshold_mm)
    print('  forward: speed %.3f * sign %.1f for %.3f s' % (
        args.forward_speed_mps,
        args.direction_sign_y,
        args.forward_duration_s,
    ))
    print('')


def _show_sensor(node: SensorScanDebugCli, label: str = '传感器快照') -> bool:
    distance_mm, age_s, distances = node.sensor_snapshot()
    valid = node.is_valid_scan_distance(distance_mm, age_s)
    print('\n%s:' % label)
    print('  all distances = %s' % [
        None if not math.isfinite(value) else round(value, 3)
        for value in distances
    ])
    print('  sensor[%d] = %s mm, age = %.3f s, valid = %s' % (
        node.args.scan_sensor_index,
        'nan' if not math.isfinite(distance_mm) else '%.3f' % distance_mm,
        age_s,
        valid,
    ))
    return valid


def _monitor_scan(node: SensorScanDebugCli) -> dict[str, Any] | None:
    args = node.args
    deadline_s = time.monotonic() + args.scan_timeout_s
    previous_mm = math.nan
    previous_age_s = math.inf
    present_start_s = math.nan
    sample_count = 0
    start_s = time.monotonic()

    print('\n开始监控传感器，按 Ctrl+C 可中断。')
    while time.monotonic() < deadline_s and rclpy.ok():
        current_mm, age_s, _distances = node.sensor_snapshot()
        if not node.is_valid_scan_distance(current_mm, age_s):
            print('  无效/过旧: value=%s age=%.3fs' % (
                'nan' if not math.isfinite(current_mm) else '%.3f' % current_mm,
                age_s,
            ))
            time.sleep(args.scan_sample_period_s)
            continue

        sample_count += 1
        now_s = time.monotonic()
        present_elapsed_s = 0.0
        if current_mm <= args.scan_present_threshold_mm:
            if not math.isfinite(present_start_s):
                present_start_s = now_s
            present_elapsed_s = now_s - present_start_s
            if present_elapsed_s >= args.scan_present_duration_s:
                print(
                    '  触发 object_present: %.3f <= %.3f mm, 持续 %.3fs'
                    % (
                        current_mm,
                        args.scan_present_threshold_mm,
                        present_elapsed_s,
                    )
                )
                return {
                    'trigger': 'object_present',
                    'current_mm': current_mm,
                    'present_elapsed_s': present_elapsed_s,
                    'elapsed_s': now_s - start_s,
                    'sample_count': sample_count,
                }
        else:
            present_start_s = math.nan

        delta_mm = math.nan
        if node.is_valid_scan_distance(previous_mm, previous_age_s):
            delta_mm = current_mm - previous_mm
            if abs(delta_mm) >= args.scan_jump_threshold_mm:
                print(
                    '  触发 jump: previous=%.3f current=%.3f delta=%.3f mm'
                    % (previous_mm, current_mm, delta_mm)
                )
                return {
                    'trigger': 'jump',
                    'previous_mm': previous_mm,
                    'current_mm': current_mm,
                    'delta_mm': delta_mm,
                    'elapsed_s': now_s - start_s,
                    'sample_count': sample_count,
                }

        print(
            '  sample=%d sensor[%d]=%.3fmm age=%.3fs delta=%s present=%.3fs'
            % (
                sample_count,
                args.scan_sensor_index,
                current_mm,
                age_s,
                'nan' if not math.isfinite(delta_mm) else '%.3f' % delta_mm,
                present_elapsed_s,
            )
        )
        previous_mm = current_mm
        previous_age_s = age_s
        time.sleep(args.scan_sample_period_s)

    print('扫描监控超时: %.3fs' % args.scan_timeout_s)
    return None


def _run_debug_flow(node: SensorScanDebugCli) -> None:
    args = node.args

    _show_sensor(node, '步骤 1/8: 启动前传感器')
    if not _ask('步骤 2/8: 是否启动水平扫描 prepare([%.4f, %.4f])？' % (
        args.scan_prepare_length_m,
        args.scan_prepare_speed_rpm,
    )):
        print('取消本轮。')
        return

    future = node.start_tool_async(
        'prepare',
        [args.scan_prepare_length_m, args.scan_prepare_speed_rpm],
    )
    if future is None:
        return

    trigger = None
    try:
        trigger = _monitor_scan(node)
        if trigger and trigger.get('trigger') == 'jump':
            print('按配置额外移动 %.3fs 到物体中心。' % args.scan_center_extra_time_s)
            time.sleep(max(0.0, args.scan_center_extra_time_s))
    finally:
        if _ask('步骤 3/8: 是否发送停止扫描命令？', default=True):
            node.stop_scan_motion()

    if trigger is None:
        print('没有检测到触发条件，本轮不继续执行抓取动作。')
        return
    print('触发结果: %s' % trigger)

    if _ask('步骤 4/8: 是否执行 FORWARD 前进？'):
        speed = args.direction_sign_y * args.forward_speed_mps
        print('FORWARD: speed=%.3fm/s duration=%.3fs' % (
            speed,
            args.forward_duration_s,
        ))
        node.publish_chassis_for(speed, args.forward_duration_s)

    if _ask('步骤 5/8: 是否执行 GRASP 夹取？'):
        node.call_tool('grasp', [0.0], args.grasp_timeout_s)

    if _ask('步骤 6/8: 是否执行 LIFT 抬升？'):
        node.publish_height(list(args.lift_height_mm))

    if _ask('步骤 7/8: 是否执行 RETREAT 后退？'):
        speed = -args.direction_sign_y * args.retreat_speed_mps
        print('RETREAT: speed=%.3fm/s duration=%.3fs' % (
            speed,
            args.retreat_duration_s,
        ))
        node.publish_chassis_for(speed, args.retreat_duration_s)

    if _ask('步骤 8/8: 是否执行 LOWER 下降？'):
        node.publish_height(list(args.lower_height_mm))

    print('本轮 debug 流程结束。')


def main() -> None:
    args = _parse_args()

    rclpy.init()
    node = SensorScanDebugCli(args, args._params)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        _print_config(args)
        while rclpy.ok():
            answer = input('是否开始一轮 sensor_scan_no_alignment debug？[y/N/q] ')
            if _quit(answer):
                break
            if not _yes(answer):
                continue
            _run_debug_flow(node)
    except KeyboardInterrupt:
        print('\n退出 debug CLI。')
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=1.0)


if __name__ == '__main__':
    main()
