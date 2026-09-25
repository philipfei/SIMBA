#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage


class TfRelay(Node):
    """Expose Create 3 transforms through the Pi's dual-interface DDS participant."""

    def __init__(self):
        super().__init__('create3_tf_relay')
        dynamic_sub_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        dynamic_pub_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        static_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.dynamic_pub = self.create_publisher(TFMessage, '/tf_relay', dynamic_pub_qos)
        self.static_pub = self.create_publisher(TFMessage, '/tf_static_relay', static_qos)
        self.create_subscription(TFMessage, '/tf', self.dynamic_pub.publish, dynamic_sub_qos)
        self.static_cache = {}
        self.create_subscription(TFMessage, '/tf_static', self.receive_static, static_qos)

    def receive_static(self, message):
        for transform in message.transforms:
            self.static_cache[(transform.header.frame_id, transform.child_frame_id)] = transform
        self.static_pub.publish(TFMessage(transforms=list(self.static_cache.values())))


def main(args=None):
    rclpy.init(args=args)
    node = TfRelay()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
