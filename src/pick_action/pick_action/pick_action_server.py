"""Pick sequence action server.

Orchestrates LiDAR recognition → X-alignment (prepare) → forward approach →
grasp → lift → retreat → lower.
"""

import json
import threading
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String

from pick_action_interfaces.action import PickSequence


class PickActionServer(Node):
    """Action server for a complete autonomous pick sequence."""

    def __init__(self) -> None:
        super().__init__('pick_action_server')

        self.declare_parameter('result_topic', '/spear_recognition/result')
        self.declare_parameter('tool_service', '/ares_tool_node/tool_action')
        self.declare_parameter('chassis_topic', '/t0x0111_')
        self.declare_parameter('lift_topic', '/t0x0112_')
        self.declare_parameter('status_topic', '/pick_action/status')

        self.declare_parameter('prepare_offset_m', 0.3)
        self.declare_parameter('direction_sign_x', -1.0)
        self.declare_parameter('deadband_x_m', 0.005)
        self.declare_parameter('prepare_timeout_ms', 20000)

        self.declare_parameter('forward_speed_mps', 0.2)
        self.declare_parameter('forward_duration_s', 2.0)
        self.declare_parameter('direction_sign_y', -1.0)

        self.declare_parameter('grasp_timeout_ms', 15000)

        self.declare_parameter('lift_height_mm', [70.0, 70.0, 70.0, 70.0])
        self.declare_parameter('lower_height_mm', [20.0, 20.0, 20.0, 20.0])

        self.declare_parameter('retreat_speed_mps', 0.2)
        self.declare_parameter('retreat_duration_s', 2.0)

        self.declare_parameter('publish_rate_hz', 100.0)

        self._latest_recognition: dict | None = None
        self._recognition_lock = threading.Lock()

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

        self._action_server = ActionServer(
            self,
            PickSequence,
            'pick_action',
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
        )

        self._tool_client = None
        self._init_tool_client()

        self.get_logger().info('Pick action server ready')

    def _init_tool_client(self) -> None:
        try:
            from ares_tool_interfaces.srv import ToolAction
            self._tool_client = self.create_client(
                ToolAction,
                self.get_parameter('tool_service').value,
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

    def _call_tool_action(self, action: str, args: list[float],
                          timeout_ms: float) -> bool:
        if not self._ensure_tool_available():
            self.get_logger().error('Tool service unavailable')
            return False

        from ares_tool_interfaces.srv import ToolAction
        req = ToolAction.Request()
        req.action = action
        req.args = args[:4] + [0.0] * max(0, 4 - len(args))

        timeout_s = timeout_ms / 1000.0
        future = self._tool_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_s)

        if future.done() and future.result() is not None:
            r = future.result()
            if r.success:
                self.get_logger().info('Tool %s completed' % action)
                return True
            self.get_logger().warn(
                'Tool %s failed: ret=%d msg="%s"'
                % (action, r.ret, r.message)
            )
            return False
        self.get_logger().error('Tool %s timed out (%.1f s)' % (action, timeout_s))
        return False

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

    def _publish_status(self, state: str, target_id: int,
                        x_m: float, y_m: float) -> None:
        msg = String()
        msg.data = json.dumps(
            {
                'state': state,
                'target_id': target_id,
                'target_x_m': round(x_m, 4),
                'target_y_m': round(y_m, 4),
            },
            ensure_ascii=False,
        )
        self._status_pub.publish(msg)

    def _execute_callback(self, goal_handle) -> PickSequence.Result:
        expected_count = goal_handle.request.expected_count
        start_time = time.monotonic()

        def feedback(state: str) -> None:
            elapsed = time.monotonic() - start_time
            goal_handle.publish_feedback(
                PickSequence.Feedback(state=state, elapsed_s=float(elapsed))
            )

        # ---- VALIDATE ----
        feedback('VALIDATING')
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

        # ---- ALIGN_X ----
        feedback('ALIGN_X')
        error_x = 0.0 - x_m
        db_x = float(self.get_parameter('deadband_x_m').value)
        if abs(error_x) > db_x:
            sign = float(self.get_parameter('direction_sign_x').value)
            offset = float(self.get_parameter('prepare_offset_m').value)
            length = sign * error_x + offset
            self.get_logger().info(
                'Align X: error=%.4f length=%.4f' % (error_x, length)
            )
            prepare_timeout = float(self.get_parameter('prepare_timeout_ms').value)
            if not self._call_tool_action('prepare', [length], prepare_timeout):
                goal_handle.abort()
                return PickSequence.Result(
                    success=False, message='prepare failed (ALIGN_X)'
                )
        else:
            self.get_logger().info('X already in deadband (error=%.4f)' % error_x)

        self._publish_status('ALIGN_X', tid, x_m, y_m)

        # Re-sample recognition for updated Y after alignment
        time.sleep(0.3)
        with self._recognition_lock:
            data = self._latest_recognition
        if data is not None and data.get('status') == 'recognized':
            targets = data.get('targets', [])
            if targets:
                best = min(targets, key=lambda t: abs(float(t.get('x_m', 0.0))))
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
        if not self._call_tool_action('grasp', [0.0], grasp_timeout):
            goal_handle.abort()
            return PickSequence.Result(
                success=False, message='grasp failed'
            )

        if goal_handle.is_cancel_requested:
            goal_handle.abort()
            return PickSequence.Result(success=False, message='cancelled')

        # ---- LIFT ----
        feedback('LIFT')
        heights = list(self.get_parameter('lift_height_mm').value)
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
        lower_heights = list(self.get_parameter('lower_height_mm').value)
        self.get_logger().info('Lowering: %s' % lower_heights)
        self._publish_status('LOWER', tid, x_m, y_m)
        msg = Float32MultiArray()
        msg.data = lower_heights
        self._lift_pub.publish(msg)
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
    executor = MultiThreadedExecutor()
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
