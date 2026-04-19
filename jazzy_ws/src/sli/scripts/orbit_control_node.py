#!/usr/bin/env python3

import threading
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from std_msgs.msg import Bool
from std_srvs.srv import SetBool, Trigger
from simulation_interfaces.srv import SetSimulationState
from orbit_interfaces.srv import StepIntegration

STATE_PLAYING = 1
STATE_PAUSED  = 2


class OrbitControlNode(Node):

    def __init__(self):
        super().__init__("orbit_control_node")

        self.declare_parameter("dt_sim",      0.00833)
        self.declare_parameter("warp_factor", 1.0)
        self.declare_parameter("auto_pause",  False)

        self._dt_sim      = self.get_parameter("dt_sim").value
        self._warp        = max(0.001, self.get_parameter("warp_factor").value)
        self._paused      = self.get_parameter("auto_pause").value
        self._step_thread  = None
        self._cancel_event = threading.Event()

        latch_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._pause_pub = self.create_publisher(Bool, "/sim_pause", latch_qos)

        self._sim_state_cli = self.create_client(
            SetSimulationState, "/set_simulation_state"
        )

        self.create_service(SetBool,         "/orbit_control/pause",  self._handle_pause)
        self.create_service(StepIntegration, "/orbit_control/step",   self._handle_step)
        self.create_service(Trigger,         "/orbit_control/cancel", self._handle_cancel)

        self._publish_pause_topic(self._paused)

        self.get_logger().info(
            f"orbit_control_node ready  "
            f"dt_sim={self._dt_sim}s  warp={self._warp}x  "
            f"initial={'PAUSED' if self._paused else 'RUNNING'}"
        )


    def _publish_pause_topic(self, paused: bool):
        """Publish /sim_pause latch only — does NOT call Isaac Sim."""
        msg = Bool()
        msg.data = paused
        self._pause_pub.publish(msg)

    def _set_isaac_state_async(self, state: int):
        """Fire-and-forget Isaac Sim state change."""
        if not self._sim_state_cli.service_is_ready():
            self.get_logger().warn("/set_simulation_state not ready")
            return
        req = SetSimulationState.Request()
        req.state.state = state
        future = self._sim_state_cli.call_async(req)
        future.add_done_callback(self._on_sim_state_done)

    def _set_isaac_state_sync(self, state: int, timeout: float = 5.0) -> bool:
        """
        Blocking Isaac Sim state change — waits for confirmation before returning.
        Called from background thread only (not from ROS callback).
        """
        if not self._sim_state_cli.wait_for_service(timeout_sec=timeout):
            self.get_logger().warn("/set_simulation_state not available")
            return False

        req = SetSimulationState.Request()
        req.state.state = state

        future = self._sim_state_cli.call_async(req)
        deadline = time.time() + timeout
        while not future.done():
            if time.time() > deadline:
                self.get_logger().warn("/set_simulation_state timed out")
                return False
            time.sleep(0.01)

        try:
            resp = future.result()
            if resp.result.result != 1:
                self.get_logger().warn(
                    f"/set_simulation_state failed: {resp.result.error_message}"
                )
                return False
            return True
        except Exception as e:
            self.get_logger().error(f"/set_simulation_state exception: {e}")
            return False

    def _on_sim_state_done(self, future):
        try:
            resp = future.result()
            if resp.result.result != 1:
                self.get_logger().warn(
                    f"/set_simulation_state: {resp.result.error_message}"
                )
        except Exception as e:
            self.get_logger().error(f"/set_simulation_state exception: {e}")


    def _handle_pause(self, request, response):
        if self._step_thread and self._step_thread.is_alive():
            self._cancel_event.set()
            self._step_thread.join(timeout=2.0)

        self._paused = request.data
        self._publish_pause_topic(self._paused)
        self._set_isaac_state_async(STATE_PAUSED if self._paused else STATE_PLAYING)

        state_str = "PAUSED" if self._paused else "RESUMED"
        self.get_logger().info(f"Simulation {state_str}")
        response.success = True
        response.message = state_str
        return response

    def _handle_step(self, request, response):
        if self._step_thread and self._step_thread.is_alive():
            response.success = False
            response.message = "Step in progress — call /orbit_control/cancel first"
            return response

        steps = request.steps
        if steps <= 0:
            response.success = False
            response.message = f"steps must be > 0, got {steps}"
            return response

        dt        = request.dt_sim if request.dt_sim > 0.0 else self._dt_sim
        sim_time  = steps * dt
        wall_time = sim_time / self._warp

        self._cancel_event.clear()
        self._step_thread = threading.Thread(
            target=self._step_worker,
            args=(steps, sim_time, wall_time),
            daemon=True,
        )
        self._step_thread.start()

        self.get_logger().info(
            f"Step started: {steps} ticks  sim={sim_time:.3f}s  "
            f"warp={self._warp}x  wall={wall_time:.3f}s"
        )
        response.success = True
        response.message = (
            f"Stepping {steps} ticks  sim={sim_time:.3f}s  "
            f"wall≈{wall_time:.3f}s  warp={self._warp}x  "
            f"— /orbit_control/cancel to stop"
        )
        return response

    def _step_worker(self, steps: int, sim_time: float, wall_time: float):

        self._paused = False
        self._publish_pause_topic(False)

        self.get_logger().info("Resuming Isaac Sim renderer...")
        ok = self._set_isaac_state_sync(STATE_PLAYING)
        if not ok:
            self.get_logger().warn(
                "Could not confirm Isaac Sim playing — "
                "stepping anyway (renderer may lag)"
            )


        chunk = 0.1
        elapsed = 0.0
        log_interval = max(1.0, wall_time / 10.0)  
        last_log = 0.0

        while elapsed < wall_time:
            if self._cancel_event.is_set():
                self.get_logger().info("Step cancelled")
                break
            sleep_time = min(chunk, wall_time - elapsed)
            time.sleep(sleep_time)
            elapsed += sleep_time

            if elapsed - last_log >= log_interval:
                pct = 100.0 * elapsed / wall_time
                self.get_logger().info(
                    f"Step {pct:.0f}%  "
                    f"sim={elapsed * self._warp:.1f}/{sim_time:.1f}s  "
                    f"wall={elapsed:.2f}/{wall_time:.2f}s"
                )
                last_log = elapsed

        self.get_logger().info("Pausing Isaac Sim renderer...")
        self._set_isaac_state_sync(STATE_PAUSED)

        self._paused = True
        self._publish_pause_topic(True)

        if not self._cancel_event.is_set():
            self.get_logger().info(
                f"Step complete — {steps} ticks  "
                f"sim={sim_time:.3f}s  simulation paused"
            )

    def _handle_cancel(self, request, response):
        if self._step_thread and self._step_thread.is_alive():
            self._cancel_event.set()
            self._step_thread.join(timeout=3.0)
            response.success = True
            response.message = "Step cancelled — simulation paused"
        else:
            response.success = False
            response.message = "No step in progress"
        return response


def main(args=None):
    rclpy.init(args=args)
    node = OrbitControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()


if __name__ == "__main__":
    main()