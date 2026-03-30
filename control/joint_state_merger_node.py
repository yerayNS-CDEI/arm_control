#!/usr/bin/env python3

from typing import Dict, List

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointStateMergerNode(Node):
    def __init__(self):
        super().__init__("joint_state_merger_node")

        self.declare_parameter("input_topics", ["/joint_states", "/arm/joint_states"])
        self.declare_parameter("output_topic", "/moveit_joint_states")
        self.declare_parameter("publish_rate", 30.0)

        self.input_topics: List[str] = list(self.get_parameter("input_topics").value)
        self.output_topic: str = str(self.get_parameter("output_topic").value)
        self.publish_rate = float(self.get_parameter("publish_rate").value)

        self._latest_positions: Dict[str, float] = {}
        self._latest_velocities: Dict[str, float] = {}
        self._latest_efforts: Dict[str, float] = {}
        self._joint_order: List[str] = []

        self.publisher = self.create_publisher(JointState, self.output_topic, 10)
        self.subscribers = [
            self.create_subscription(
                JointState,
                topic,
                lambda msg, source_topic=topic: self._joint_state_callback(msg, source_topic),
                10,
            )
            for topic in self.input_topics
        ]
        self.timer = self.create_timer(1.0 / max(self.publish_rate, 1.0), self._publish_merged_state)

        self.get_logger().info(
            f"Merging joint states from {self.input_topics} into {self.output_topic}"
        )

    def _joint_state_callback(self, msg: JointState, source_topic: str) -> None:
        for index, joint_name in enumerate(msg.name):
            if joint_name not in self._joint_order:
                self._joint_order.append(joint_name)
                self.get_logger().debug(
                    f"Discovered joint '{joint_name}' from {source_topic}"
                )

            if index < len(msg.position):
                self._latest_positions[joint_name] = msg.position[index]
            if index < len(msg.velocity):
                self._latest_velocities[joint_name] = msg.velocity[index]
            if index < len(msg.effort):
                self._latest_efforts[joint_name] = msg.effort[index]

    def _publish_merged_state(self) -> None:
        if not self._joint_order:
            return

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self._joint_order)
        msg.position = [self._latest_positions.get(name, 0.0) for name in self._joint_order]

        if self._latest_velocities:
            msg.velocity = [self._latest_velocities.get(name, 0.0) for name in self._joint_order]

        if self._latest_efforts:
            msg.effort = [self._latest_efforts.get(name, 0.0) for name in self._joint_order]

        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = JointStateMergerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
