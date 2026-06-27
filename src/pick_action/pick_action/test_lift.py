#!/usr/bin/env python3
"""Test chassis lift/lower on /t0x0102_.

Usage:
  python3 test_lift.py                  # send [100, 100, 100, 100] (lift 100mm)
  python3 test_lift.py -100 -100 -100 -100  # lower 100mm
  python3 test_lift.py 50 50 50 50      # lift 50mm
"""

import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Node('test_lift')
    pub = node.create_publisher(Float32MultiArray, '/t0x0102_', 10)
    time.sleep(0.1)

    heights = [float(a) for a in sys.argv[1:]] if len(sys.argv) > 1 else [100.0] * 4
    if len(heights) != 4:
        heights = list(heights) + [heights[-1]] * (4 - len(heights))

    msg = Float32MultiArray()
    msg.data = heights
    print('Publishing to /t0x0102_: %s' % heights)
    pub.publish(msg)
    time.sleep(0.2)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
