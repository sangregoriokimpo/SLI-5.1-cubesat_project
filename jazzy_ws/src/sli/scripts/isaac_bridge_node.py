#!/usr/bin/env python3
"""
isaac_bridge_node.py

One instance per body.
Subscribes to OrbitState, calls /set_entity_state to move USD prim in Isaac Sim.

SetEntityState response structure:
  response.result.result        uint8  (RESULT_OK=1)
  response.result.error_message string

Parameters:
  prim_path      string   USD prim path e.g. /World/Sat1
  state_topic    string   topic to subscribe, default: orbit_state
  attractor_path string   attractor prim path (future: query world pos)
  scale          float    unit scale: 1.0 if scene in km, 1000.0 if meters
"""

import rclpy
from rclpy.node import Node

from orbit_interfaces.msg import OrbitState
from simulation_interfaces.srv import SetEntityState

RESULT_OK = 1


class IsaacBridgeNode(Node):

    def __init__(self):
        super().__init__("isaac_bridge_node")

        self.declare_parameter("prim_path",      "/World/Sat")
        self.declare_parameter("state_topic",    "orbit_state")
        self.declare_parameter("attractor_path", "/World/Earth")
        self.declare_parameter("scale",          1.0)

        self._prim_path      = self.get_parameter("prim_path").value
        self._state_topic    = self.get_parameter("state_topic").value
        self._attractor_path = self.get_parameter("attractor_path").value
        self._scale          = self.get_parameter("scale").value

        self._cli            = self.create_client(SetEntityState, "/set_entity_state")
        self._call_in_flight = False
        self._service_ready  = False

        self._sub = self.create_subscription(
            OrbitState, self._state_topic, self._on_orbit_state, 10
        )

        self.create_timer(2.0, self._check_service)

        self.get_logger().info(
            f"isaac_bridge_node: {self._state_topic} → {self._prim_path} "
            f"scale={self._scale}"
        )

    def _check_service(self):
        if self._cli.service_is_ready():
            if not self._service_ready:
                self.get_logger().info("/set_entity_state service ready.")
            self._service_ready = True
        else:
            self.get_logger().warn(
                "/set_entity_state not ready yet — is Isaac Sim running?"
            )

    def _on_orbit_state(self, msg: OrbitState):
        if not self._service_ready or self._call_in_flight:
            return

        s = self._scale

        req = SetEntityState.Request()
        req.entity = self._prim_path
        req.state.header.stamp    = self.get_clock().now().to_msg()
        req.state.header.frame_id = "world"
        req.state.pose.position.x    = msg.position.x * s
        req.state.pose.position.y    = msg.position.y * s
        req.state.pose.position.z    = msg.position.z * s
        req.state.pose.orientation.w = msg.attitude.w
        req.state.pose.orientation.x = msg.attitude.x
        req.state.pose.orientation.y = msg.attitude.y
        req.state.pose.orientation.z = msg.attitude.z
        req.state.twist.linear.x     = msg.velocity.x * s
        req.state.twist.linear.y     = msg.velocity.y * s
        req.state.twist.linear.z     = msg.velocity.z * s

        self._call_in_flight = True
        future = self._cli.call_async(req)
        future.add_done_callback(self._on_done)

    def _on_done(self, future):
        self._call_in_flight = False
        try:
            response = future.result()
            # response.result.result: uint8  (RESULT_OK = 1)
            # response.result.error_message: string
            if response.result.result != RESULT_OK:
                self.get_logger().warn(
                    f"/set_entity_state failed for {self._prim_path}: "
                    f"code={response.result.result} "
                    f"msg={response.result.error_message}"
                )
        except Exception as e:
            self.get_logger().error(f"/set_entity_state exception: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = IsaacBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()


if __name__ == "__main__":
    main()