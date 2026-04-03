#!/usr/bin/env python3
"""
isaac_bridge_node.py

One instance per body.
Subscribes to OrbitState on ~/orbit_state (or ~/cw_state for CW bodies).
Calls /set_entity_state to move the corresponding USD prim in Isaac Sim.

Parameters:
  prim_path       string   USD prim path to move e.g. /World/Sat1
  state_topic     string   topic to subscribe to, default: orbit_state
  attractor_path  string   attractor prim path (to get world position offset)
  scale           float    unit scale factor, default 1.0
                           set to 0.001 if your scene is in meters but
                           orbit state is in km

Note on coordinates:
  OrbitState position is relative to the attractor in km.
  Isaac Sim /set_entity_state expects world-frame meters (or scene units).
  The attractor world position is assumed to be at the origin for now —
  a follow-up can query /get_entity_state for the attractor to handle
  non-origin attractors.
"""

import sys

import rclpy
from rclpy.node import Node

from orbit_interfaces.msg import OrbitState
from simulation_interfaces.srv import SetEntityState
from geometry_msgs.msg import PoseStamped


class IsaacBridgeNode(Node):

    SERVICE_TIMEOUT = 30.0

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

        # Service client
        self._cli = self.create_client(SetEntityState, "/set_entity_state")

        # Subscriber — topic name is relative so namespace applies
        self._sub = self.create_subscription(
            OrbitState,
            self._state_topic,
            self._on_orbit_state,
            10,
        )

        # Pending call guard — don't stack up calls if Isaac Sim is slow
        self._call_in_flight = False

        self.get_logger().info(
            f"isaac_bridge_node: {self._state_topic} → {self._prim_path} "
            f"scale={self._scale}"
        )

        # Wait for service in background (non-blocking warn only)
        self._service_ready = False
        self._check_service_timer = self.create_timer(
            2.0, self._check_service
        )

    def _check_service(self):
        if self._cli.service_is_ready():
            if not self._service_ready:
                self.get_logger().info("/set_entity_state service ready.")
            self._service_ready = True
            self._check_service_timer.cancel()
        else:
            self.get_logger().warn(
                "/set_entity_state not ready yet — is Isaac Sim running?"
            )

    def _on_orbit_state(self, msg: OrbitState):
        if not self._service_ready:
            return
        if self._call_in_flight:
            return   # drop frame, don't queue up

        # Attractor offset: for now assume attractor is at world origin.
        # TODO: query /get_entity_state for attractor_path to handle
        #       non-origin attractors.
        ax, ay, az = 0.0, 0.0, 0.0

        s = self._scale
        wx = (ax + msg.position.x) * s
        wy = (ay + msg.position.y) * s
        wz = (az + msg.position.z) * s

        req = SetEntityState.Request()
        req.entity = self._prim_path

        req.state.header.stamp    = self.get_clock().now().to_msg()
        req.state.header.frame_id = "world"

        req.state.pose.position.x    = wx
        req.state.pose.position.y    = wy
        req.state.pose.position.z    = wz
        req.state.pose.orientation.w = 1.0
        req.state.pose.orientation.x = 0.0
        req.state.pose.orientation.y = 0.0
        req.state.pose.orientation.z = 0.0

        req.state.twist.linear.x  = msg.velocity.x * s
        req.state.twist.linear.y  = msg.velocity.y * s
        req.state.twist.linear.z  = msg.velocity.z * s

        self._call_in_flight = True
        future = self._cli.call_async(req)
        future.add_done_callback(self._on_set_state_done)

    def _on_set_state_done(self, future):
        self._call_in_flight = False
        try:
            result = future.result()
            if result.result.result_code != 0:
                self.get_logger().warn(
                    f"/set_entity_state failed for {self._prim_path}: "
                    f"code={result.result.result_code} "
                    f"msg={result.result.error_message}"
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
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()