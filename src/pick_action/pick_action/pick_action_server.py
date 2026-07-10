"""Pick sequence action server.

Orchestrates LiDAR recognition → X-alignment (prepare) → forward approach →
grasp → lift → retreat → lower.
"""

import json
import math
import os
import threading
import time
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String

from .pose_alignment import (
    correct_pose_from_odin,
    yaw_from_quaternion,
)
from pick_action_interfaces.action import PickSequence
from r2_interfaces.srv import ToolAction


@dataclass
class ToolCallResult:
    success: bool
    action: str
    detail: str
    ret: int | None = None
    message: str = ''
    timeout_s: float | None = None
    timed_out: bool = False

    def __bool__(self) -> bool:
        return self.success


class PickActionServer(Node):
    """Action server for a complete autonomous pick sequence."""

    def __init__(self) -> None:
        super().__init__('pick_action_server')

        self.declare_parameter('result_topic', '/spear_recognition/result')
        self.declare_parameter('alignment_mode', 'lidar_recognition')
        self.declare_parameter('tool_service', '/ares_tool_node/tool_action')
        self.declare_parameter('chassis_topic', '/t0x0111_')
        self.declare_parameter('lift_topic', '/t0x0112_')
        self.declare_parameter('status_topic', '/pick_action/status')

        self.declare_parameter('sensor_topic', '/sensor_distances')
        self.declare_parameter('pose_topic', '/odin1/relocation')
        self.declare_parameter('sensor_count', 8)
        self.declare_parameter('sensor_3_index', 3)
        self.declare_parameter('sensor_5_index', 5)
        self.declare_parameter('sensor_max_age_s', 0.5)
        self.declare_parameter('pose_max_age_s', 0.5)

        self.declare_parameter('field_origin_x_m', 0.0)
        self.declare_parameter('field_origin_y_m', 0.0)
        self.declare_parameter('gripper_forward_m', 0.0)
        self.declare_parameter('gripper_left_m', 0.0)
        self.declare_parameter('gripper_yaw_offset_rad', 0.0)
        self.declare_parameter('target_x_m', 1.05)
        self.declare_parameter('target_y_m', -0.15)
        self.declare_parameter('gripper_move_direct', -1.0)

        self.declare_parameter('prepare_base_length_m', 0.3)
        self.declare_parameter('prepare_min_length_m', 0.0)
        self.declare_parameter('prepare_max_length_m', 0.5)
        self.declare_parameter('direction_sign_x', -1.0)
        self.declare_parameter('deadband_x_m', 0.005)
        self.declare_parameter('prepare_timeout_ms', 20000)

        self.declare_parameter('scan_sensor_index', 1)
        self.declare_parameter('scan_sensor_max_age_s', 0.5)
        self.declare_parameter('scan_enable_jump_trigger', False)
        self.declare_parameter('scan_jump_threshold_mm', 80.0)
        self.declare_parameter('scan_present_threshold_mm', 250.0)
        self.declare_parameter('scan_present_duration_s', 0.2)
        self.declare_parameter('debug_scan_target_position_m', 0.5)
        self.declare_parameter('scan_prepare_length_m', 0.05)
        self.declare_parameter('scan_prepare_speed_rpm', 30.0)
        self.declare_parameter('scan_center_extra_time_s', 0.25)
        self.declare_parameter('scan_timeout_s', 5.0)
        self.declare_parameter('scan_sample_period_s', 0.02)
        self.declare_parameter('scan_stop_action', 'prepare')
        self.declare_parameter('scan_stop_args', [0.0, 0.0])
        self.declare_parameter('scan_stop_timeout_ms', 3000)
        self.declare_parameter('scan_stop_to_grasp_delay_s', 0.2)
        self.declare_parameter(
            'scan_debug_log_path',
            '/tmp/pick_action_sensor_scan_debug.jsonl',
        )

        self.declare_parameter('forward_speed_mps', 0.2)
        self.declare_parameter('forward_duration_s', 2.0)
        self.declare_parameter('direction_sign_y', -1.0)

        self.declare_parameter('grasp_timeout_ms', 15000)
        self.declare_parameter('grasp_retry_delay_s', 0.1)

        self.declare_parameter('lift_height_mm', [70.0, 70.0, 70.0, 70.0])
        self.declare_parameter('lower_height_mm', [20.0, 20.0, 20.0, 20.0])

        self.declare_parameter('retreat_speed_mps', 0.2)
        self.declare_parameter('retreat_duration_s', 2.0)

        self.declare_parameter('publish_rate_hz', 100.0)
        self.declare_parameter('alignment_data_path', '/tmp/alignment_result.json')

        self._latest_recognition: dict | None = None
        self._recognition_lock = threading.Lock()
        self._latest_distances = [
            math.nan
            for _ in range(int(self.get_parameter('sensor_count').value))
        ]
        self._latest_sensor_receive_time_s = math.nan
        self._latest_pose: PoseStamped | None = None
        self._latest_pose_receive_time_s = math.nan
        self._pose_lock = threading.Lock()
        self._callback_group = ReentrantCallbackGroup()
        self._active_scan_debug_id: str | None = None
        self._active_goal_condition = threading.Condition()
        self._active_goal_running = False
        self._last_active_goal_result: tuple[bool, str] | None = None

        self._chassis_pub = self.create_publisher(
            Float32MultiArray,
            self.get_parameter('chassis_topic').value,
            10,
        )
        self._lift_pub = self.create_publisher(
            Float32MultiArray,
            self.get_parameter('lift_topic').value,
            10,
        )
        self._status_pub = self.create_publisher(
            String,
            self.get_parameter('status_topic').value,
            10,
        )
        self._subscription = self.create_subscription(
            String,
            self.get_parameter('result_topic').value,
            self._recognition_callback,
            10,
        )
        self._sensor_subscription = self.create_subscription(
            Float32MultiArray,
            self.get_parameter('sensor_topic').value,
            self._sensor_callback,
            10,
        )
        self._pose_subscription = self.create_subscription(
            PoseStamped,
            self.get_parameter('pose_topic').value,
            self._pose_callback,
            10,
        )

        self._action_server = ActionServer(
            self,
            PickSequence,
            'pick_action',
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._callback_group,
        )

        self._tool_client = None
        self._init_tool_client()

        self.get_logger().info(
            'Pick action server ready; alignment_mode=%s'
            % self.get_parameter('alignment_mode').value
        )

    def _init_tool_client(self) -> None:
        try:
            self._tool_client = self.create_client(
                ToolAction,
                self.get_parameter('tool_service').value,
                callback_group=self._callback_group,
            )
            self.get_logger().info('ToolAction client created')
        except Exception as exc:
            self.get_logger().warn(
                'ToolAction service not available (%s); tool disabled' % exc
            )

    def _ensure_tool_available(self) -> bool:
        if self._tool_client is None:
            return False
        return self._tool_client.wait_for_service(timeout_sec=3.0)

    def _recognition_callback(self, message: String) -> None:
        try:
            data = json.loads(message.data)
            with self._recognition_lock:
                self._latest_recognition = data
        except (TypeError, json.JSONDecodeError):
            pass

    def _sensor_callback(self, msg: Float32MultiArray) -> None:
        sensor_count = int(self.get_parameter('sensor_count').value)
        distances = [math.nan] * sensor_count
        for index in range(min(sensor_count, len(msg.data))):
            distances[index] = float(msg.data[index])
        with self._pose_lock:
            self._latest_distances = distances
            self._latest_sensor_receive_time_s = self._now_sec()

    def _pose_callback(self, msg: PoseStamped) -> None:
        with self._pose_lock:
            self._latest_pose = msg
            self._latest_pose_receive_time_s = self._now_sec()

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _uses_odin_sensor_projection(self) -> bool:
        mode = str(self.get_parameter('alignment_mode').value)
        return mode.lower() in (
            'odin_sensor_projection',
            'odin_sensor',
            'projection',
        )

    def _uses_no_alignment(self) -> bool:
        mode = str(self.get_parameter('alignment_mode').value)
        return mode.lower() in (
            'no_alignment',
            'none',
            'direct',
        )

    def _uses_sensor_scan_no_alignment(self) -> bool:
        mode = str(self.get_parameter('alignment_mode').value)
        return mode.lower() in (
            'sensor_scan_no_alignment',
            'sensor_scan',
            'scan_no_alignment',
            'scan_direct',
        )

    def _goal_callback(self, goal_request: PickSequence.Goal) -> GoalResponse:
        self.get_logger().info('Received goal: expected_count=%d' % goal_request.expected_count)
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle) -> CancelResponse:
        self.get_logger().info('Cancel requested')
        return CancelResponse.ACCEPT

    def _wait_for_recognition(self, expected_count: int, timeout_s: float) -> bool:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            with self._recognition_lock:
                data = self._latest_recognition
            if data is not None:
                status = data.get('status', '')
                count = data.get('recognized_count', 0)
                if status == 'recognized' and count == expected_count:
                    return True
            time.sleep(0.05)
        return False

    def _clear_recognition(self) -> None:
        with self._recognition_lock:
            self._latest_recognition = None

    def _pick_best_target(self) -> tuple[int, float, float]:
        with self._recognition_lock:
            targets = list(self._latest_recognition.get('targets', []))
        if not targets:
            return -1, 0.0, 0.0
        best = min(targets, key=lambda t: abs(float(t.get('x_m', 0.0))))
        tid = int(best.get('id', -1))
        x_m = float(best['x_m'])
        y_m = float(best['y_m'])
        return tid, x_m, y_m

    def _compute_odin_sensor_alignment(self) -> dict | None:
        now_s = self._now_sec()
        with self._pose_lock:
            distances = list(self._latest_distances)
            sensor_receive_time_s = self._latest_sensor_receive_time_s
            pose = self._latest_pose
            pose_receive_time_s = self._latest_pose_receive_time_s

        if pose is None:
            return None
        sensor_age_s = now_s - sensor_receive_time_s
        pose_age_s = now_s - pose_receive_time_s
        if (
            not math.isfinite(sensor_age_s)
            or sensor_age_s > float(self.get_parameter('sensor_max_age_s').value)
            or pose_age_s > float(self.get_parameter('pose_max_age_s').value)
        ):
            return None

        sensor_3_index = int(self.get_parameter('sensor_3_index').value)
        sensor_5_index = int(self.get_parameter('sensor_5_index').value)
        if (
            sensor_3_index >= len(distances)
            or sensor_5_index >= len(distances)
            or sensor_3_index < 0
            or sensor_5_index < 0
        ):
            self.get_logger().error('Configured sensor index is out of range')
            return None

        sensor_3_mm = distances[sensor_3_index]
        sensor_5_mm = distances[sensor_5_index]
        if not math.isfinite(sensor_3_mm) or not math.isfinite(sensor_5_mm):
            return None

        position = pose.pose.position
        yaw_rad = yaw_from_quaternion(pose.pose.orientation)
        corrected = correct_pose_from_odin(
            sensor_3_mm,
            sensor_5_mm,
            float(position.x),
            float(position.y),
            yaw_rad,
            float(self.get_parameter('field_origin_x_m').value),
            float(self.get_parameter('field_origin_y_m').value),
            float(self.get_parameter('gripper_forward_m').value),
            float(self.get_parameter('gripper_left_m').value),
            float(self.get_parameter('gripper_yaw_offset_rad').value),
            float(self.get_parameter('target_x_m').value),
            float(self.get_parameter('target_y_m').value),
            float(self.get_parameter('gripper_move_direct').value),
        )
        return {
            'target_id': 0,
            'target_x_m': corrected['target_x_m'],
            'target_y_m': corrected['target_y_m'],
            'sensor_3_mm': sensor_3_mm,
            'sensor_5_mm': sensor_5_mm,
            'sensor_age_s': sensor_age_s,
            'pose_age_s': pose_age_s,
            'odin_x_m': float(position.x),
            'odin_y_m': float(position.y),
            'odin_yaw_rad': yaw_rad,
            'corrected': corrected,
            'gripper_x_m': corrected['corrected_gripper_x_m'],
            'gripper_y_m': corrected['corrected_gripper_y_m'],
            'gripper_yaw_rad': corrected['corrected_gripper_yaw_rad'],
            'projection_x_m': corrected['target_projection_x_m'],
            'projection_y_m': corrected['target_projection_y_m'],
            'raw_along_offset_m': corrected['raw_gripper_forward_move_m'],
            'along_offset_m': corrected['gripper_forward_move_m'],
            'direct': corrected['direct'],
            'lateral_error_m': corrected['gripper_lateral_error_m'],
        }

    def _wait_for_odin_sensor_alignment(self, timeout_s: float) -> dict | None:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            alignment = self._compute_odin_sensor_alignment()
            if alignment is not None:
                return alignment
            time.sleep(0.05)
        return None

    def _call_tool_action(self, action: str, args: list[float],
                          timeout_ms: float) -> ToolCallResult:
        if not self._ensure_tool_available():
            detail = (
                'Tool service unavailable: service=%s wait_timeout=3.0s'
                % self.get_parameter('tool_service').value
            )
            self.get_logger().error(detail)
            self._write_scan_debug_log(
                'tool_unavailable',
                action=action,
                args=args,
            )
            return ToolCallResult(False, action, detail)

        req = ToolAction.Request()
        req.action = action
        req.args = args[:4] + [0.0] * max(0, 4 - len(args))

        timeout_s = timeout_ms / 1000.0
        self._write_scan_debug_log(
            'tool_call_start',
            action=action,
            args=list(req.args),
            timeout_s=timeout_s,
        )
        future = self._tool_client.call_async(req)
        deadline_s = time.monotonic() + timeout_s
        while rclpy.ok() and not future.done() and time.monotonic() < deadline_s:
            time.sleep(0.02)

        if future.done():
            try:
                r = future.result()
            except Exception as exc:
                detail = 'Tool %s raised exception: %s' % (action, exc)
                self.get_logger().error(detail)
                self._write_scan_debug_log(
                    'tool_call_exception',
                    action=action,
                    error=str(exc),
                )
                return ToolCallResult(False, action, detail)
            if r is None:
                detail = 'Tool %s returned no result' % action
                self.get_logger().error(detail)
                self._write_scan_debug_log(
                    'tool_call_no_result',
                    action=action,
                )
                return ToolCallResult(False, action, detail)
            self._write_scan_debug_log(
                'tool_call_result',
                action=action,
                success=bool(r.success),
                ret=int(r.ret),
                message=str(r.message),
            )
            if r.success:
                self.get_logger().info('Tool %s completed' % action)
                return ToolCallResult(
                    True,
                    action,
                    'Tool %s completed' % action,
                    ret=int(r.ret),
                    message=str(r.message),
                    timeout_s=timeout_s,
                )
            detail = (
                'Tool %s failed: ret=%d msg="%s" timeout=%.3fs args=%s'
                % (action, r.ret, r.message, timeout_s, list(req.args))
            )
            self.get_logger().warn(detail)
            return ToolCallResult(
                False,
                action,
                detail,
                ret=int(r.ret),
                message=str(r.message),
                timeout_s=timeout_s,
            )
        detail = (
            'Tool %s timed out after %.3fs waiting for response; args=%s'
            % (action, timeout_s, list(req.args))
        )
        self.get_logger().error(detail)
        self._write_scan_debug_log(
            'tool_call_timeout',
            action=action,
            timeout_s=timeout_s,
        )
        return ToolCallResult(
            False,
            action,
            detail,
            timeout_s=timeout_s,
            timed_out=True,
        )

    def _call_grasp_with_optional_retry(
        self,
        timeout_ms: float,
        retry_on_timeout: bool,
    ) -> ToolCallResult:
        first = self._call_tool_action('grasp', [0.0], timeout_ms)
        if first:
            return first
        if not retry_on_timeout or not first.timed_out:
            return first

        retry_delay_s = float(self.get_parameter('grasp_retry_delay_s').value)
        self.get_logger().warn(
            'Grasp timed out; retrying once after %.3f s. First failure: %s'
            % (retry_delay_s, first.detail)
        )
        self._write_scan_debug_log(
            'grasp_retry_after_timeout',
            retry_delay_s=retry_delay_s,
            first_failure=first.detail,
        )
        time.sleep(retry_delay_s)

        second = self._call_tool_action('grasp', [0.0], timeout_ms)
        if second:
            return second

        return ToolCallResult(
            False,
            'grasp',
            'grasp failed after one retry; first="%s"; second="%s"'
            % (first.detail, second.detail),
            ret=second.ret,
            message=second.message,
            timeout_s=second.timeout_s,
            timed_out=second.timed_out,
        )

    def _start_tool_action_async(self, action: str, args: list[float]):
        if not self._ensure_tool_available():
            self.get_logger().error('Tool service unavailable')
            self._write_scan_debug_log(
                'tool_unavailable',
                action=action,
                args=args,
                async_call=True,
            )
            return None

        req = ToolAction.Request()
        req.action = action
        req.args = args[:4] + [0.0] * max(0, 4 - len(args))
        self.get_logger().info(
            'Starting tool %s asynchronously: args=%s' % (action, req.args)
        )
        self._write_scan_debug_log(
            'tool_async_start',
            action=action,
            args=list(req.args),
        )
        return self._tool_client.call_async(req)

    def _get_scan_sensor_distance_mm(self) -> tuple[float, float]:
        now_s = self._now_sec()
        with self._pose_lock:
            distances = list(self._latest_distances)
            receive_time_s = self._latest_sensor_receive_time_s

        index = int(self.get_parameter('scan_sensor_index').value)
        if index < 0 or index >= len(distances):
            return math.nan, math.inf

        age_s = now_s - receive_time_s
        if not math.isfinite(age_s):
            return math.nan, math.inf
        return distances[index], age_s

    def _is_valid_scan_distance(self, distance_mm: float, age_s: float) -> bool:
        return (
            math.isfinite(distance_mm)
            and math.isfinite(age_s)
            and age_s <= float(self.get_parameter('scan_sensor_max_age_s').value)
        )

    def _stop_scan_motion(self) -> bool:
        stop_action = str(self.get_parameter('scan_stop_action').value)
        stop_args = [
            float(value)
            for value in self.get_parameter('scan_stop_args').value
        ]
        stop_timeout = float(self.get_parameter('scan_stop_timeout_ms').value)
        self.get_logger().info(
            'Stopping sensor scan with tool %s args=%s'
            % (stop_action, stop_args)
        )
        return self._call_tool_action(stop_action, stop_args, stop_timeout)

    def _json_safe(self, value):
        if isinstance(value, float):
            return value if math.isfinite(value) else None
        if isinstance(value, (str, int, bool)) or value is None:
            return value
        if isinstance(value, list):
            return [self._json_safe(item) for item in value]
        if isinstance(value, tuple):
            return [self._json_safe(item) for item in value]
        if isinstance(value, dict):
            return {
                str(key): self._json_safe(item)
                for key, item in value.items()
            }
        return str(value)

    def _write_scan_debug_log(self, event: str, **fields) -> None:
        path = str(self.get_parameter('scan_debug_log_path').value)
        if not path:
            return
        record = {
            'stamp_s': self._json_safe(self._now_sec()),
            'monotonic_s': self._json_safe(time.monotonic()),
            'event': event,
        }
        if self._active_scan_debug_id is not None:
            record['scan_id'] = self._active_scan_debug_id
        record.update({
            key: self._json_safe(value)
            for key, value in fields.items()
        })
        try:
            os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
            with open(path, 'a', encoding='utf-8') as handle:
                handle.write(json.dumps(record, ensure_ascii=False) + '\n')
        except OSError as exc:
            self.get_logger().warn(
                'Failed to write sensor scan debug log %s: %s' % (path, exc)
            )

    def _run_sensor_scan(self, goal_handle) -> dict | None:
        scan_target_position_m = float(
            self.get_parameter('debug_scan_target_position_m').value
        )
        scan_speed_rpm = float(self.get_parameter('scan_prepare_speed_rpm').value)
        present_threshold_mm = float(
            self.get_parameter('scan_present_threshold_mm').value
        )
        present_duration_s = float(
            self.get_parameter('scan_present_duration_s').value
        )
        enable_jump_trigger = bool(
            self.get_parameter('scan_enable_jump_trigger').value
        )
        jump_threshold_mm = float(
            self.get_parameter('scan_jump_threshold_mm').value
        )
        timeout_s = float(self.get_parameter('scan_timeout_s').value)
        sample_period_s = float(self.get_parameter('scan_sample_period_s').value)
        sensor_index = int(self.get_parameter('scan_sensor_index').value)

        self._active_scan_debug_id = 'scan-%.6f' % time.time()
        self._write_scan_debug_log(
            'scan_start',
            sensor_index=sensor_index,
            scan_target_position_m=scan_target_position_m,
            scan_speed_rpm=scan_speed_rpm,
            present_threshold_mm=present_threshold_mm,
            present_duration_s=present_duration_s,
            enable_jump_trigger=enable_jump_trigger,
            jump_threshold_mm=jump_threshold_mm,
            timeout_s=timeout_s,
            sample_period_s=sample_period_s,
        )
        self.get_logger().info(
            'Sensor scan target: prepare([%.4f, %.4f])'
            % (scan_target_position_m, scan_speed_rpm)
        )
        future = self._start_tool_action_async(
            'prepare',
            [scan_target_position_m, scan_speed_rpm],
        )
        if future is None:
            self._write_scan_debug_log('scan_target_start_failed')
            self._active_scan_debug_id = None
            return None

        start_s = time.monotonic()
        present_start_s = math.nan
        previous_mm = math.nan
        previous_age_s = math.inf
        sample_count = 0
        stopped = False

        try:
            while time.monotonic() - start_s < timeout_s:
                if goal_handle.is_cancel_requested:
                    self.get_logger().info('Cancelled during sensor scan')
                    self._write_scan_debug_log('scan_cancelled')
                    return None

                current_mm, age_s = self._get_scan_sensor_distance_mm()
                if not self._is_valid_scan_distance(current_mm, age_s):
                    self._write_scan_debug_log(
                        'scan_sample_invalid',
                        sensor_index=sensor_index,
                        distance_mm=current_mm,
                        age_s=age_s,
                    )
                    time.sleep(sample_period_s)
                    continue

                sample_count += 1
                now_monotonic_s = time.monotonic()
                present_elapsed_s = 0.0
                delta_mm = math.nan
                if current_mm <= present_threshold_mm:
                    if not math.isfinite(present_start_s):
                        present_start_s = now_monotonic_s
                    present_elapsed_s = now_monotonic_s - present_start_s
                    if present_elapsed_s >= present_duration_s:
                        scan_elapsed_s = now_monotonic_s - start_s
                        self.get_logger().info(
                            'Sensor scan object-present trigger: sensor[%d] '
                            'distance=%.1f mm <= %.1f mm for %.3f s'
                            % (
                                sensor_index,
                                current_mm,
                                present_threshold_mm,
                                present_elapsed_s,
                            )
                        )
                        stopped = self._stop_scan_motion()
                        self._write_scan_debug_log(
                            'scan_trigger',
                            trigger='object_present',
                            sensor_index=sensor_index,
                            distance_mm=current_mm,
                            age_s=age_s,
                            present_elapsed_s=present_elapsed_s,
                            elapsed_s=scan_elapsed_s,
                            sample_count=sample_count,
                            stop_success=stopped,
                        )
                        return {
                            'alignment_mode': 'sensor_scan_no_alignment',
                            'scan_trigger': 'object_present',
                            'scan_sensor_index': sensor_index,
                            'scan_target_position_m': round(
                                scan_target_position_m, 4
                            ),
                            'scan_current_mm': round(current_mm, 3),
                            'scan_present_threshold_mm': round(
                                present_threshold_mm, 3
                            ),
                            'scan_present_duration_s': round(
                                present_duration_s, 3
                            ),
                            'scan_present_elapsed_s': round(present_elapsed_s, 3),
                            'scan_elapsed_s': round(scan_elapsed_s, 3),
                            'scan_sample_count': sample_count,
                            'scan_stop_success': stopped,
                        }
                else:
                    present_start_s = math.nan

                if (
                    enable_jump_trigger
                    and self._is_valid_scan_distance(previous_mm, previous_age_s)
                ):
                    delta_mm = current_mm - previous_mm
                    if abs(delta_mm) >= jump_threshold_mm:
                        scan_elapsed_s = now_monotonic_s - start_s
                        self.get_logger().info(
                            'Sensor scan jump trigger: sensor[%d] previous=%.1f '
                            'current=%.1f delta=%.1f mm'
                            % (
                                sensor_index,
                                previous_mm,
                                current_mm,
                                delta_mm,
                            )
                        )
                        stopped = self._stop_scan_motion()
                        self._write_scan_debug_log(
                            'scan_trigger',
                            trigger='jump',
                            sensor_index=sensor_index,
                            previous_mm=previous_mm,
                            distance_mm=current_mm,
                            age_s=age_s,
                            delta_mm=delta_mm,
                            elapsed_s=scan_elapsed_s,
                            sample_count=sample_count,
                            stop_success=stopped,
                        )
                        return {
                            'alignment_mode': 'sensor_scan_no_alignment',
                            'scan_trigger': 'jump',
                            'scan_sensor_index': sensor_index,
                            'scan_target_position_m': round(
                                scan_target_position_m, 4
                            ),
                            'scan_previous_mm': round(previous_mm, 3),
                            'scan_current_mm': round(current_mm, 3),
                            'scan_delta_mm': round(delta_mm, 3),
                            'scan_jump_threshold_mm': round(
                                jump_threshold_mm, 3
                            ),
                            'scan_elapsed_s': round(scan_elapsed_s, 3),
                            'scan_sample_count': sample_count,
                            'scan_stop_success': stopped,
                        }

                self._write_scan_debug_log(
                    'scan_sample',
                    sensor_index=sensor_index,
                    sample_count=sample_count,
                    distance_mm=current_mm,
                    age_s=age_s,
                    delta_mm=delta_mm,
                    present_elapsed_s=present_elapsed_s,
                )
                previous_mm = current_mm
                previous_age_s = age_s

                time.sleep(sample_period_s)

            self.get_logger().error(
                'Sensor scan timed out after %.2f s without distance <= %.1f mm '
                'for %.3f s'
                % (timeout_s, present_threshold_mm, present_duration_s)
            )
            self._write_scan_debug_log(
                'scan_timeout',
                timeout_s=timeout_s,
                present_threshold_mm=present_threshold_mm,
                present_duration_s=present_duration_s,
                sample_count=sample_count,
            )
            return None
        finally:
            if not stopped:
                stop_success = self._stop_scan_motion()
                self._write_scan_debug_log(
                    'scan_final_stop',
                    stop_success=stop_success,
                )
            self._active_scan_debug_id = None

    def _run_timed_publish(self, speed: float, duration_s: float,
                           goal_handle) -> None:
        rate_hz = float(self.get_parameter('publish_rate_hz').value)
        period = 1.0 / rate_hz
        msg = Float32MultiArray()
        msg.data = [speed, 0.0, 0.0]

        start = time.monotonic()
        while time.monotonic() - start < duration_s:
            if goal_handle.is_cancel_requested:
                self.get_logger().info('Cancelled during timed movement')
                break
            self._chassis_pub.publish(msg)
            time.sleep(period)

        msg.data = [0.0, 0.0, 0.0]
        self._chassis_pub.publish(msg)

    def _publish_height(self, heights: list[float]) -> None:
        msg = Float32MultiArray()
        msg.data = [float(h) for h in heights[:4]]
        self._lift_pub.publish(msg)

    def _publish_status(self, state: str, target_id: int,
                        x_m: float, y_m: float,
                        extra: dict | None = None) -> None:
        msg = String()
        payload = {
            'state': state,
            'target_id': target_id,
            'target_x_m': round(x_m, 4),
            'target_y_m': round(y_m, 4),
        }
        if extra:
            payload.update(extra)
        msg.data = json.dumps(payload, ensure_ascii=False)
        self._status_pub.publish(msg)

    def _projection_status_extra(self, alignment: dict | None) -> dict:
        if alignment is None:
            return {}
        return {
            'alignment_mode': 'odin_sensor_projection',
            'sensor_3_mm': round(float(alignment['sensor_3_mm']), 3),
            'sensor_5_mm': round(float(alignment['sensor_5_mm']), 3),
            'odin_x_m': round(float(alignment['odin_x_m']), 4),
            'odin_y_m': round(float(alignment['odin_y_m']), 4),
            'odin_yaw_rad': round(float(alignment['odin_yaw_rad']), 6),
            'gripper_x_m': round(float(alignment['gripper_x_m']), 4),
            'gripper_y_m': round(float(alignment['gripper_y_m']), 4),
            'gripper_yaw_rad': round(float(alignment['gripper_yaw_rad']), 6),
            'projection_x_m': round(float(alignment['projection_x_m']), 4),
            'projection_y_m': round(float(alignment['projection_y_m']), 4),
            'along_offset_m': round(float(alignment['along_offset_m']), 4),
            'raw_along_offset_m': round(float(alignment['raw_along_offset_m']), 4),
            'direct': round(float(alignment['direct']), 1),
            'lateral_error_m': round(float(alignment['lateral_error_m']), 4),
        }

    def _save_alignment_data(self, alignment: dict, corrected: dict,
                             sensor_3_mm: float, sensor_5_mm: float) -> None:
        """Save correction results, projection distance, and target coordinates to file."""
        save_path = str(self.get_parameter('alignment_data_path').value)
        data = {
            'timestamp_s': self._now_sec(),
            'correction_result': {
                'sensor_3_mm': round(float(sensor_3_mm), 3),
                'sensor_5_mm': round(float(sensor_5_mm), 3),
                'input_field_x_m': round(float(corrected['input_field_x_m']), 6),
                'input_field_y_m': round(float(corrected['input_field_y_m']), 6),
                'input_field_yaw_rad': round(float(corrected['input_field_yaw_rad']), 6),
                'corrected_robot_x_m': round(float(corrected['corrected_robot_x_m']), 6),
                'corrected_robot_y_m': round(float(corrected['corrected_robot_y_m']), 6),
                'corrected_robot_yaw_rad': round(float(corrected['corrected_robot_yaw_rad']), 6),
                'corrected_gripper_x_m': round(float(corrected['corrected_gripper_x_m']), 6),
                'corrected_gripper_y_m': round(float(corrected['corrected_gripper_y_m']), 6),
                'corrected_gripper_yaw_rad': round(float(corrected['corrected_gripper_yaw_rad']), 6),
                'target_x_m': round(float(corrected['target_x_m']), 6),
                'target_y_m': round(float(corrected['target_y_m']), 6),
                'target_projection_x_m': round(float(corrected['target_projection_x_m']), 6),
                'target_projection_y_m': round(float(corrected['target_projection_y_m']), 6),
                'raw_gripper_forward_move_m': round(float(corrected['raw_gripper_forward_move_m']), 6),
                'gripper_forward_move_m': round(float(corrected['gripper_forward_move_m']), 6),
                'direct': round(float(corrected['direct']), 1),
                'gripper_lateral_error_m': round(float(corrected['gripper_lateral_error_m']), 6),
                'robot_delta_x_m': round(float(corrected['robot_delta_x_m']), 6),
                'robot_delta_y_m': round(float(corrected['robot_delta_y_m']), 6),
            },
            'projection_distance': {
                'along_offset_m': round(float(alignment['along_offset_m']), 6),
                'lateral_error_m': round(float(alignment['lateral_error_m']), 6),
                'projection_x_m': round(float(alignment['projection_x_m']), 6),
                'projection_y_m': round(float(alignment['projection_y_m']), 6),
                'gripper_x_m': round(float(alignment['gripper_x_m']), 6),
                'gripper_y_m': round(float(alignment['gripper_y_m']), 6),
                'gripper_yaw_rad': round(float(alignment['gripper_yaw_rad']), 6),
            },
            'target_coordinates': {
                'target_x_m': round(float(alignment['target_x_m']), 6),
                'target_y_m': round(float(alignment['target_y_m']), 6),
            },
        }
        try:
            os.makedirs(os.path.dirname(save_path) or '.', exist_ok=True)
            with open(save_path, 'w', encoding='utf-8') as f:
                json.dump(data, f, ensure_ascii=False, indent=2)
            self.get_logger().info(
                'Alignment data saved to %s' % save_path
            )
        except (OSError, TypeError) as exc:
            self.get_logger().error(
                'Failed to save alignment data to %s: %s' % (save_path, exc)
            )

    def _execute_callback(self, goal_handle) -> PickSequence.Result:
        with self._active_goal_condition:
            if self._active_goal_running:
                self.get_logger().warn(
                    'Joining active pick sequence instead of starting another one'
                )
                while self._active_goal_running:
                    if goal_handle.is_cancel_requested:
                        goal_handle.abort()
                        return PickSequence.Result(
                            success=False,
                            message='cancelled while waiting for active pick',
                        )
                    self._active_goal_condition.wait(timeout=0.1)

                success, message = self._last_active_goal_result or (
                    False,
                    'active pick sequence ended without result',
                )
                if success:
                    goal_handle.succeed()
                else:
                    goal_handle.abort()
                return PickSequence.Result(
                    success=success,
                    message='Joined active pick sequence: %s' % message,
                )

            self._active_goal_running = True
            self._last_active_goal_result = None

        result = None
        try:
            result = self._execute_pick_sequence(goal_handle)
            return result
        finally:
            with self._active_goal_condition:
                if result is None:
                    self._last_active_goal_result = (
                        False,
                        'active pick sequence ended without result',
                    )
                else:
                    self._last_active_goal_result = (
                        bool(result.success),
                        str(result.message),
                    )
                self._active_goal_running = False
                self._active_goal_condition.notify_all()

    def _execute_pick_sequence(self, goal_handle) -> PickSequence.Result:
        expected_count = goal_handle.request.expected_count
        start_time = time.monotonic()
        use_projection = self._uses_odin_sensor_projection()
        use_no_alignment = self._uses_no_alignment()
        use_sensor_scan = self._uses_sensor_scan_no_alignment()
        projection_alignment = None
        sensor_scan_result = None

        def feedback(state: str) -> None:
            elapsed = time.monotonic() - start_time
            goal_handle.publish_feedback(
                PickSequence.Feedback(state=state, elapsed_s=float(elapsed))
            )

        # ---- VALIDATE ----
        feedback('VALIDATING')
        if use_no_alignment or use_sensor_scan:
            tid = 0
            x_m = 0.0
            y_m = 0.0
            mode_name = (
                'sensor_scan_no_alignment'
                if use_sensor_scan else 'no_alignment'
            )
            self._publish_status(
                'VALIDATING',
                tid,
                x_m,
                y_m,
                {'alignment_mode': mode_name},
            )
            if use_sensor_scan:
                self.get_logger().info(
                    'Sensor-scan no-alignment mode: skipping recognition and '
                    'Odin correction; sensor scan will run before FORWARD'
                )
            else:
                self.get_logger().info(
                    'No-alignment mode: skipping recognition, Odin/sensor '
                    'correction, and ALIGN_X prepare'
                )
        elif use_projection:
            projection_alignment = self._wait_for_odin_sensor_alignment(
                timeout_s=10.0
            )
            if projection_alignment is None:
                self.get_logger().error(
                    'Odin/sensor projection data not ready'
                )
                goal_handle.abort()
                return PickSequence.Result(
                    success=False,
                    message='Odin pose or sensor 3/5 data not ready',
                )
            tid = int(projection_alignment['target_id'])
            x_m = float(projection_alignment['target_x_m'])
            y_m = float(projection_alignment['target_y_m'])
            self._publish_status(
                'VALIDATING',
                tid,
                x_m,
                y_m,
                self._projection_status_extra(projection_alignment),
            )
            self.get_logger().info(
                'Projection target: target=(%.4f, %.4f) '
                'gripper=(%.4f, %.4f, yaw=%.4f) move=%.4f '
                'raw=%.4f direct=%.1f lateral=%.4f'
                % (
                    x_m,
                    y_m,
                    projection_alignment['gripper_x_m'],
                    projection_alignment['gripper_y_m'],
                    projection_alignment['gripper_yaw_rad'],
                    projection_alignment['along_offset_m'],
                    projection_alignment['raw_along_offset_m'],
                    projection_alignment['direct'],
                    projection_alignment['lateral_error_m'],
                )
            )
            self._save_alignment_data(
                projection_alignment,
                projection_alignment['corrected'],
                float(projection_alignment['sensor_3_mm']),
                float(projection_alignment['sensor_5_mm']),
            )
        else:
            feedback('PRE_RECOGNITION_FORWARD')
            sign_y = float(self.get_parameter('direction_sign_y').value)
            fwd_speed = sign_y * float(
                self.get_parameter('forward_speed_mps').value
            )
            fwd_duration = float(self.get_parameter('forward_duration_s').value)
            self._publish_status(
                'PRE_RECOGNITION_FORWARD',
                0,
                0.0,
                0.0,
                {'alignment_mode': 'lidar_recognition'},
            )
            self.get_logger().info(
                'Pre-recognition forward: %.2f m/s for %.1f s'
                % (fwd_speed, fwd_duration)
            )
            self._run_timed_publish(fwd_speed, fwd_duration, goal_handle)
            if goal_handle.is_cancel_requested:
                goal_handle.abort()
                return PickSequence.Result(success=False, message='cancelled')

            self._clear_recognition()
            if not self._wait_for_recognition(expected_count, timeout_s=10.0):
                self.get_logger().error(
                    'Recognition failed (expected %d targets)' % expected_count
                )
                goal_handle.abort()
                return PickSequence.Result(
                    success=False,
                    message='Recognition did not reach %d targets within timeout'
                            % expected_count,
                )

            tid, x_m, y_m = self._pick_best_target()
            if tid < 0:
                goal_handle.abort()
                return PickSequence.Result(success=False, message='No targets found')

            self._publish_status('VALIDATING', tid, x_m, y_m)
            self.get_logger().info(
                'Selected target %d: x=%.4f y=%.4f' % (tid, x_m, y_m)
            )

        if use_sensor_scan:
            # ---- SENSOR_SCAN ----
            feedback('SENSOR_SCAN')
            self._publish_status(
                'SENSOR_SCAN',
                tid,
                x_m,
                y_m,
                {'alignment_mode': 'sensor_scan_no_alignment'},
            )
            sensor_scan_result = self._run_sensor_scan(goal_handle)
            if sensor_scan_result is None:
                goal_handle.abort()
                return PickSequence.Result(
                    success=False,
                    message=(
                        'sensor scan failed or timed out; check '
                        'scan_debug_log_path=%s for sensor samples and tool '
                        'stop result'
                    ) % self.get_parameter('scan_debug_log_path').value,
                )
            if not bool(sensor_scan_result.get('scan_stop_success', False)):
                goal_handle.abort()
                return PickSequence.Result(
                    success=False,
                    message=(
                        'sensor scan detected target but failed to stop scan '
                        'motion; stop_action=%s stop_args=%s timeout_ms=%.1f'
                    ) % (
                        self.get_parameter('scan_stop_action').value,
                        list(self.get_parameter('scan_stop_args').value),
                        float(self.get_parameter('scan_stop_timeout_ms').value),
                    ),
                )
            self._publish_status(
                'SENSOR_SCAN',
                tid,
                x_m,
                y_m,
                sensor_scan_result,
            )

        if not use_no_alignment and not use_sensor_scan:
            # ---- ALIGN_X ----
            feedback('ALIGN_X')
            if use_projection:
                error_x = float(projection_alignment['along_offset_m'])
            else:
                error_x = 0.0 - x_m
            db_x = float(self.get_parameter('deadband_x_m').value)
            if abs(error_x) > db_x:
                if use_projection:
                    length = (
                        float(self.get_parameter('prepare_base_length_m').value)
                        + error_x
                    )
                else:
                    sign = float(self.get_parameter('direction_sign_x').value)
                    length = (
                        sign * error_x
                        + float(self.get_parameter('prepare_base_length_m').value)
                    )
                min_length = float(self.get_parameter('prepare_min_length_m').value)
                max_length = float(self.get_parameter('prepare_max_length_m').value)
                self.get_logger().info(
                    'Align X: error=%.4f length=%.4f' % (error_x, length)
                )
                if length < min_length or length > max_length:
                    self.get_logger().error(
                        'prepare length %.4f out of range [%.4f, %.4f]'
                        % (length, min_length, max_length)
                    )
                    goal_handle.abort()
                    return PickSequence.Result(
                        success=False,
                        message='prepare length out of range',
                    )
                prepare_timeout = float(
                    self.get_parameter('prepare_timeout_ms').value
                )
                prepare_result = self._call_tool_action(
                    'prepare',
                    [length],
                    prepare_timeout,
                )
                if not prepare_result:
                    goal_handle.abort()
                    return PickSequence.Result(
                        success=False,
                        message='prepare failed (ALIGN_X): %s'
                                % prepare_result.detail,
                    )
            else:
                self.get_logger().info(
                    'X already in deadband (error=%.4f)' % error_x
                )

            self._publish_status(
                'ALIGN_X',
                tid,
                x_m,
                y_m,
                (
                    self._projection_status_extra(projection_alignment)
                    if use_projection else None
                ),
            )

            if use_projection:
                time.sleep(0.3)
                refreshed = self._compute_odin_sensor_alignment()
                if refreshed is not None:
                    projection_alignment = refreshed
                    tid = int(projection_alignment['target_id'])
                    x_m = float(projection_alignment['target_x_m'])
                    y_m = float(projection_alignment['target_y_m'])
                    self._save_alignment_data(
                        projection_alignment,
                        projection_alignment['corrected'],
                        float(projection_alignment['sensor_3_mm']),
                        float(projection_alignment['sensor_5_mm']),
                    )
            else:
                # Re-sample recognition for updated Y after alignment
                time.sleep(0.3)
                with self._recognition_lock:
                    data = self._latest_recognition
                if data is not None and data.get('status') == 'recognized':
                    targets = data.get('targets', [])
                    if targets:
                        best = min(
                            targets,
                            key=lambda t: abs(float(t.get('x_m', 0.0))),
                        )
                        y_m = float(best['y_m'])
                        x_m = float(best['x_m'])
                        tid = int(best.get('id', tid))

        # ---- FORWARD ----
        feedback('FORWARD')
        sign_y = float(self.get_parameter('direction_sign_y').value)
        fwd_speed = sign_y * float(self.get_parameter('forward_speed_mps').value)
        fwd_duration = float(self.get_parameter('forward_duration_s').value)
        self.get_logger().info(
            'Forward: %.2f m/s for %.1f s' % (fwd_speed, fwd_duration)
        )
        self._publish_status('FORWARD', tid, x_m, y_m)
        self._run_timed_publish(fwd_speed, fwd_duration, goal_handle)

        if goal_handle.is_cancel_requested:
            goal_handle.abort()
            return PickSequence.Result(success=False, message='cancelled')

        # ---- GRASP ----
        feedback('GRASP')
        self.get_logger().info('Grasping...')
        self._publish_status('GRASP', tid, x_m, y_m)
        grasp_timeout = float(self.get_parameter('grasp_timeout_ms').value)
        retry_grasp = (
            use_sensor_scan
            and sensor_scan_result is not None
            and bool(sensor_scan_result.get('scan_stop_success', False))
        )
        if retry_grasp:
            delay_s = float(
                self.get_parameter('scan_stop_to_grasp_delay_s').value
            )
            self.get_logger().info(
                'Sensor scan stopped successfully; delaying %.3f s before grasp'
                % delay_s
            )
            self._write_scan_debug_log(
                'scan_stop_to_grasp_delay',
                delay_s=delay_s,
            )
            time.sleep(delay_s)

        grasp_result = self._call_grasp_with_optional_retry(
            grasp_timeout,
            retry_on_timeout=retry_grasp,
        )
        if not grasp_result:
            goal_handle.abort()
            return PickSequence.Result(
                success=False,
                message='grasp failed: %s' % grasp_result.detail,
            )

        if goal_handle.is_cancel_requested:
            goal_handle.abort()
            return PickSequence.Result(success=False, message='cancelled')

        # ---- LIFT ----
        feedback('LIFT')
        heights = [float(h) for h in self.get_parameter('lift_height_mm').value]
        self.get_logger().info('Lifting: %s' % heights)
        self._publish_status('LIFT', tid, x_m, y_m)
        msg = Float32MultiArray()
        msg.data = heights
        self._lift_pub.publish(msg)
        time.sleep(0.2)

        # ---- RETREAT ----
        feedback('RETREAT')
        retreat_speed = -sign_y * float(self.get_parameter('retreat_speed_mps').value)
        retreat_duration = float(self.get_parameter('retreat_duration_s').value)
        self.get_logger().info(
            'Retreat: %.2f m/s for %.1f s' % (retreat_speed, retreat_duration)
        )
        self._publish_status('RETREAT', tid, x_m, y_m)
        self._run_timed_publish(retreat_speed, retreat_duration, goal_handle)

        if goal_handle.is_cancel_requested:
            goal_handle.abort()
            return PickSequence.Result(success=False, message='cancelled')

        # ---- LOWER ----
        feedback('LOWER')
        lower_heights = [float(h) for h in self.get_parameter('lower_height_mm').value]
        self.get_logger().info('Lowering: %s' % lower_heights)
        self._publish_status('LOWER', tid, x_m, y_m)
        self._publish_height(lower_heights)
        time.sleep(0.2)

        # ---- DONE ----
        feedback('DONE')
        elapsed = time.monotonic() - start_time
        self._publish_status('DONE', tid, x_m, y_m)
        self.get_logger().info(
            'Pick sequence complete in %.1f s' % elapsed
        )

        goal_handle.succeed()
        return PickSequence.Result(
            success=True,
            message='Pick sequence complete in %.1f s' % elapsed,
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PickActionServer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
