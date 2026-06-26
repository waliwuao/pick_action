#!/usr/bin/env python3
"""Trigger the pick action sequence and wait for completion."""

import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from pick_action.action import PickSequence


class TriggerPickAction(Node):
    """Action client that sends a goal and prints feedback."""

    def __init__(self) -> None:
        super().__init__('trigger_pick_client')
        self._client = ActionClient(self, PickSequence, 'pick_action')
        self._done = False
        self._result: PickSequence.Result | None = None

    def _feedback_callback(self, feedback_msg) -> None:
        fb = feedback_msg.feedback
        print('[%s] elapsed=%.1f s' % (fb.state, fb.elapsed_s))

    def _goal_response_callback(self, future) -> None:
        goal_handle = future.result()
        if goal_handle is None:
            print('ERROR: goal rejected', file=sys.stderr)
            self._done = True
            return
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self._result_callback)

    def _result_callback(self, future) -> None:
        result = future.result()
        if result is not None:
            self._result = result.result
        else:
            print('ERROR: future resolved with no result', file=sys.stderr)
        self._done = True

    def send_goal(self, expected_count: int = 3) -> bool:
        if not self._client.wait_for_server(timeout_sec=5.0):
            print('ERROR: action server not available', file=sys.stderr)
            return False

        goal = PickSequence.Goal()
        goal.expected_count = expected_count
        send_future = self._client.send_goal_async(
            goal, feedback_callback=self._feedback_callback
        )
        send_future.add_done_callback(self._goal_response_callback)
        return True


def main(args=None) -> None:
    rclpy.init(args=args)

    node = TriggerPickAction()
    expected_count = int(sys.argv[1]) if len(sys.argv) > 1 else 3

    print('Sending goal: expected_count=%d ...' % expected_count)
    if not node.send_goal(expected_count):
        sys.exit(1)

    deadline = time.monotonic() + 120.0
    while rclpy.ok() and not node._done:
        rclpy.spin_once(node, timeout_sec=0.1)
        if time.monotonic() > deadline:
            print('ERROR: timed out', file=sys.stderr)
            sys.exit(1)

    if node._result is not None:
        result = node._result
        print('success=%s  message="%s"' % (result.success, result.message))
        if not result.success:
            sys.exit(1)
    else:
        print('ERROR: no result returned', file=sys.stderr)
        sys.exit(1)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
