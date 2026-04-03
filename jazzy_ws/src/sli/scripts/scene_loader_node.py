#!/usr/bin/env python3
"""
scene_loader_node.py

Runs once at startup:
  1. Reads orbit_scene.yaml
  2. Calls /load_world  → loads earth base scene into Isaac Sim
  3. Calls /spawn_entity → spawns each body with spawn: true
  4. Calls /set_simulation_state → play
  5. Exits cleanly

Requires:
  ros-jazzy-simulation-interfaces
  PyYAML
"""

import os
import sys

import rclpy
from rclpy.node import Node

import yaml

from simulation_interfaces.srv import (
    LoadWorld,
    SpawnEntity,
    SetSimulationState,
)
from simulation_interfaces.msg import SimulationState
from geometry_msgs.msg import PoseStamped


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def resolve_path(raw: str, workspace_root: str) -> str:
    """
    Resolve a path token from the config.

    Priority:
      1. Absolute path → use as-is
      2. Starts with a known URI scheme (omniverse://, http://) → use as-is
      3. Relative → join with workspace_root
    """
    if os.path.isabs(raw):
        return raw
    if "://" in raw:
        return raw
    return os.path.join(workspace_root, raw)


def load_config(config_path: str) -> dict:
    with open(config_path, "r") as f:
        cfg = yaml.safe_load(f)

    # Resolve workspace_root
    raw_root = cfg.get("workspace_root", "")
    # Expand env vars like ${ORBIT_WS}
    raw_root = os.path.expandvars(raw_root)

    # Fallback: directory containing the config file
    if not raw_root or raw_root == "${ORBIT_WS}":
        raw_root = os.path.dirname(os.path.abspath(config_path))

    cfg["_workspace_root"] = raw_root
    return cfg


def identity_pose() -> PoseStamped:
    """Return a PoseStamped at origin with identity orientation."""
    p = PoseStamped()
    p.header.frame_id = "world"
    p.pose.position.x = 0.0
    p.pose.position.y = 0.0
    p.pose.position.z = 0.0
    p.pose.orientation.w = 1.0
    p.pose.orientation.x = 0.0
    p.pose.orientation.y = 0.0
    p.pose.orientation.z = 0.0
    return p


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------

class SceneLoaderNode(Node):

    # Timeout waiting for each Isaac Sim service (seconds)
    SERVICE_TIMEOUT = 30.0

    def __init__(self, config_path: str):
        super().__init__("scene_loader_node")
        self._cfg = load_config(config_path)
        self._workspace_root = self._cfg["_workspace_root"]
        self.get_logger().info(f"workspace_root = {self._workspace_root}")

        # Service clients
        self._cli_load_world = self.create_client(
            LoadWorld, "/load_world"
        )
        self._cli_spawn = self.create_client(
            SpawnEntity, "/spawn_entity"
        )
        self._cli_sim_state = self.create_client(
            SetSimulationState, "/set_simulation_state"
        )

    # ------------------------------------------------------------------
    # Public entry point
    # ------------------------------------------------------------------

    def run(self):
        """Blocking: wait for services, load world, spawn bodies, play."""
        self._wait_for_services()
        self._load_world()
        self._spawn_bodies()
        self._play()
        self.get_logger().info("Scene loader done — exiting.")

    # ------------------------------------------------------------------
    # Steps
    # ------------------------------------------------------------------

    def _wait_for_services(self):
        for cli, name in [
            (self._cli_load_world, "/load_world"),
            (self._cli_spawn,      "/spawn_entity"),
            (self._cli_sim_state,  "/set_simulation_state"),
        ]:
            self.get_logger().info(f"Waiting for {name} ...")
            if not cli.wait_for_service(timeout_sec=self.SERVICE_TIMEOUT):
                self.get_logger().error(
                    f"Service {name} not available after "
                    f"{self.SERVICE_TIMEOUT}s — is Isaac Sim running with "
                    f"isaacsim.ros2.sim_control enabled?"
                )
                sys.exit(1)
            self.get_logger().info(f"{name} ready.")

    def _load_world(self):
        world_cfg = self._cfg.get("world", {})
        raw_scene = world_cfg.get("base_scene", "")
        if not raw_scene:
            self.get_logger().warn("No base_scene specified — skipping /load_world.")
            return

        uri = resolve_path(raw_scene, self._workspace_root)
        self.get_logger().info(f"Loading world: {uri}")

        req = LoadWorld.Request()
        req.uri = uri

        future = self._cli_load_world.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=60.0)

        if future.result() is None:
            self.get_logger().error("/load_world call failed (timeout or exception).")
            sys.exit(1)

        result = future.result()
        if result.result.result_code != 0:
            self.get_logger().error(
                f"/load_world failed: code={result.result.result_code} "
                f"msg={result.result.error_message}"
            )
            sys.exit(1)

        self.get_logger().info("World loaded successfully.")

    def _spawn_bodies(self):
        bodies = self._cfg.get("bodies", [])
        for body in bodies:
            if not body.get("spawn", False):
                self.get_logger().info(
                    f"Skipping spawn for '{body['name']}' (spawn: false)"
                )
                continue
            self._spawn_one(body)

    def _spawn_one(self, body: dict):
        name      = body["name"]
        prim_path = body["prim_path"]
        raw_usd   = body.get("usd", "")

        # Resolve USD path
        if raw_usd:
            uri = resolve_path(raw_usd, self._workspace_root)
        else:
            # Empty URI → Isaac Sim creates an Xform (invisible anchor)
            uri = ""

        self.get_logger().info(
            f"Spawning '{name}' at {prim_path} uri='{uri}'"
        )

        req = SpawnEntity.Request()
        req.name           = prim_path      # full USD prim path as the entity name
        req.allow_renaming = False
        req.uri            = uri
        req.initial_pose   = identity_pose()

        future = self._cli_spawn.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)

        if future.result() is None:
            self.get_logger().error(
                f"/spawn_entity for '{name}' timed out."
            )
            return

        result = future.result()
        if result.result.result_code != 0:
            self.get_logger().error(
                f"/spawn_entity for '{name}' failed: "
                f"code={result.result.result_code} "
                f"msg={result.result.error_message}"
            )
        else:
            self.get_logger().info(f"Spawned '{name}' OK.")

    def _play(self):
        self.get_logger().info("Setting simulation state → PLAYING ...")
        req = SetSimulationState.Request()
        req.state.state = SimulationState.STATE_PLAYING   # 1

        future = self._cli_sim_state.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=15.0)

        if future.result() is None:
            self.get_logger().error("/set_simulation_state timed out.")
            return

        result = future.result()
        if result.result.result_code != 0:
            self.get_logger().error(
                f"/set_simulation_state failed: "
                f"code={result.result.result_code} "
                f"msg={result.result.error_message}"
            )
        else:
            self.get_logger().info("Simulation playing.")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    rclpy.init()

    # Config path: first CLI arg, or default relative to this file
    if len(sys.argv) > 1:
        config_path = sys.argv[1]
    else:
        config_path = os.path.join(
            os.path.dirname(__file__), "orbit_scene.yaml"
        )

    if not os.path.exists(config_path):
        print(f"[scene_loader] ERROR: config not found: {config_path}")
        sys.exit(1)

    node = SceneLoaderNode(config_path)
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()