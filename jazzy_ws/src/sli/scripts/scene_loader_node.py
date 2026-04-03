#!/usr/bin/env python3
"""
scene_loader_node.py

Runs once at startup:
  1. Reads orbit_scene.yaml
  2. /load_world  → loads earth base scene
  3. /spawn_entity → spawns each body with spawn: true
  4. /set_simulation_state → play
  5. Exits

simulation_interfaces Result structure:
  response.result.result        uint8  (RESULT_OK = 1)
  response.result.error_message string
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

RESULT_OK = 1




def resolve_path(raw: str, workspace_root: str) -> str:
    if os.path.isabs(raw) or "://" in raw:
        return raw
    return os.path.join(workspace_root, raw)


def load_config(config_path: str) -> dict:
    with open(config_path, "r") as f:
        cfg = yaml.safe_load(f)
    raw_root = os.path.expandvars(cfg.get("workspace_root", ""))
    if not raw_root or raw_root == "${ORBIT_WS}":
        raw_root = os.path.dirname(os.path.dirname(os.path.abspath(config_path)))
    cfg["_workspace_root"] = raw_root
    return cfg


def identity_pose() -> PoseStamped:
    p = PoseStamped()
    p.header.frame_id = "world"
    p.pose.orientation.w = 1.0
    return p


def check_result(response, name: str, logger) -> bool:
    """
    Check simulation_interfaces service response.
    Structure: response.result.result (uint8), response.result.error_message
    RESULT_OK = 1
    """
    code = response.result.result
    if code != RESULT_OK:
        logger.error(
            f"{name} failed: code={code} "
            f"msg={response.result.error_message}"
        )
        return False
    return True




class SceneLoaderNode(Node):

    SERVICE_TIMEOUT = 30.0

    def __init__(self, config_path: str):
        super().__init__("scene_loader_node")
        self._cfg = load_config(config_path)
        self._workspace_root = self._cfg["_workspace_root"]
        self.get_logger().info(f"workspace_root = {self._workspace_root}")

        self._cli_load  = self.create_client(LoadWorld,          "/load_world")
        self._cli_spawn = self.create_client(SpawnEntity,        "/spawn_entity")
        self._cli_state = self.create_client(SetSimulationState, "/set_simulation_state")

    def run(self):
        self._wait_for_services()
        self._load_world()
        self._spawn_bodies()
        self._play()
        self.get_logger().info("Scene loader complete — exiting.")


    def _wait_for_services(self):
        for cli, name in [
            (self._cli_load,  "/load_world"),
            (self._cli_spawn, "/spawn_entity"),
            (self._cli_state, "/set_simulation_state"),
        ]:
            self.get_logger().info(f"Waiting for {name} ...")
            if not cli.wait_for_service(timeout_sec=self.SERVICE_TIMEOUT):
                self.get_logger().error(
                    f"{name} not available after {self.SERVICE_TIMEOUT}s "
                    f"— is Isaac Sim running with isaacsim.ros2.sim_control enabled?"
                )
                sys.exit(1)
            self.get_logger().info(f"{name} ready.")

    def _load_world(self):
        raw = self._cfg.get("world", {}).get("base_scene", "")
        if not raw:
            self.get_logger().warn("No base_scene — skipping /load_world.")
            return

        uri = resolve_path(raw, self._workspace_root)
        self.get_logger().info(f"Loading world: {uri}")

        req = LoadWorld.Request()
        req.uri = uri

        future = self._cli_load.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=60.0)

        if future.result() is None:
            self.get_logger().error("/load_world timed out.")
            sys.exit(1)

        if not check_result(future.result(), "/load_world", self.get_logger()):
            sys.exit(1)

        self.get_logger().info("World loaded.")

    def _spawn_bodies(self):
        for body in self._cfg.get("bodies", []):
            if not body.get("spawn", False):
                self.get_logger().info(f"Skipping '{body['name']}' (spawn: false)")
                continue
            self._spawn_one(body)

    def _spawn_one(self, body: dict):
        name      = body["name"]
        prim_path = body["prim_path"]
        raw_usd   = body.get("usd", "")
        uri       = resolve_path(raw_usd, self._workspace_root) if raw_usd else ""

        self.get_logger().info(f"Spawning '{name}' → {prim_path}  uri='{uri}'")

        req = SpawnEntity.Request()
        req.name           = prim_path
        req.allow_renaming = False
        req.uri            = uri
        req.initial_pose   = identity_pose()

        future = self._cli_spawn.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)

        if future.result() is None:
            self.get_logger().error(f"/spawn_entity for '{name}' timed out.")
            return

        if check_result(future.result(), f"/spawn_entity({name})", self.get_logger()):
            self.get_logger().info(f"Spawned '{name}' OK.")

    def _play(self):
        self.get_logger().info("Setting simulation → PLAYING ...")
        req = SetSimulationState.Request()
        req.state.state = SimulationState.STATE_PLAYING

        future = self._cli_state.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=15.0)

        if future.result() is None:
            self.get_logger().error("/set_simulation_state timed out.")
            return

        if check_result(future.result(), "/set_simulation_state", self.get_logger()):
            self.get_logger().info("Simulation playing.")




def main():
    rclpy.init()

    config_path = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
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


if __name__ == "__main__":
    main()