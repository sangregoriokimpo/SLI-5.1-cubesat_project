#!/usr/bin/env python3
"""
orbit_vel_node.py

One instance per orbital body (role: orbit).
Reads initial conditions from ROS2 parameters (set by launch file from config).
Runs RK4 integrator at fixed dt_sim, publishes OrbitState at ~publish_rate Hz.
Subscribes to ThrustCmd for impulse / continuous thrust.

Parameters (set via --ros-args -p):
  mu          float   gravitational parameter km³/s²
  dt_sim      float   integrator timestep s
  prim_path   string  USD prim path (used as frame id in published state)
  attractor   string  attractor prim path
  orbit_type  string  circular | elements
  radius      float   (circular) km
  plane       string  (circular) xy | xz | yz
  a           float   (elements) semi-major axis km
  e           float   (elements) eccentricity
  inc         float   (elements) inclination deg
  raan        float   (elements) RAAN deg
  argp        float   (elements) argument of periapsis deg
  nu          float   (elements) true anomaly deg
  publish_rate float  Hz, default 30.0

Published topics (within node namespace):
  orbit_state   orbit_interfaces/msg/OrbitState

Subscribed topics (within node namespace):
  cmd_thrust    orbit_interfaces/msg/ThrustCmd
"""

import math
import os
import sys

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from orbit_interfaces.msg import OrbitState, ThrustCmd


# ---------------------------------------------------------------------------
# Minimal pure-Python orbital math (no Isaac dependency)
# ---------------------------------------------------------------------------

Vec3 = tuple  # (float, float, float)


def v_add(a: Vec3, b: Vec3) -> Vec3:
    return (a[0]+b[0], a[1]+b[1], a[2]+b[2])


def v_mul(s: float, a: Vec3) -> Vec3:
    return (s*a[0], s*a[1], s*a[2])


def v_norm(a: Vec3) -> float:
    return math.sqrt(a[0]**2 + a[1]**2 + a[2]**2)


def v_sub(a: Vec3, b: Vec3) -> Vec3:
    return (a[0]-b[0], a[1]-b[1], a[2]-b[2])


def rot_z(angle: float, v: Vec3) -> Vec3:
    c, s = math.cos(angle), math.sin(angle)
    return (c*v[0] - s*v[1], s*v[0] + c*v[1], v[2])


def rot_x(angle: float, v: Vec3) -> Vec3:
    c, s = math.cos(angle), math.sin(angle)
    return (v[0], c*v[1] - s*v[2], s*v[1] + c*v[2])


def circular_ic(mu: float, radius: float, plane: str) -> tuple[Vec3, Vec3]:
    """Initial conditions for circular orbit."""
    v = math.sqrt(mu / radius)
    plane = plane.lower()
    if plane == "xy":
        return (radius, 0.0, 0.0), (0.0, v, 0.0)
    if plane == "xz":
        return (radius, 0.0, 0.0), (0.0, 0.0, v)
    if plane == "yz":
        return (0.0, radius, 0.0), (0.0, 0.0, v)
    raise ValueError(f"Unknown plane: {plane}")


def elements_ic(mu, a, e, inc_deg, raan_deg, argp_deg, nu_deg) -> tuple[Vec3, Vec3]:
    """Classical orbital elements → (r, v) inertial."""
    inc  = math.radians(inc_deg)
    raan = math.radians(raan_deg)
    argp = math.radians(argp_deg)
    nu   = math.radians(nu_deg)

    p    = a * (1.0 - e*e)
    rmag = p / (1.0 + e * math.cos(nu))

    r_pqw = (rmag * math.cos(nu), rmag * math.sin(nu), 0.0)
    fac   = math.sqrt(mu / p)
    v_pqw = (-fac * math.sin(nu), fac * (e + math.cos(nu)), 0.0)

    def to_world(vec):
        return rot_z(raan, rot_x(inc, rot_z(argp, vec)))

    return to_world(r_pqw), to_world(v_pqw)


def rk4_step(mu: float, r: Vec3, v: Vec3, dt: float, a_cmd: Vec3) -> tuple[Vec3, Vec3]:
    """Single RK4 step. r and v in km, dt in s, a_cmd in km/s²."""

    def gravity(rr: Vec3) -> Vec3:
        d = v_norm(rr)
        if d < 1e-12:
            return (0.0, 0.0, 0.0)
        s = -mu / d**3
        return (s*rr[0], s*rr[1], s*rr[2])

    def accel(rr: Vec3) -> Vec3:
        g = gravity(rr)
        return (g[0]+a_cmd[0], g[1]+a_cmd[1], g[2]+a_cmd[2])

    # k1
    k1r, k1v = v, accel(r)
    # k2
    r2 = v_add(r, v_mul(0.5*dt, k1r))
    v2 = v_add(v, v_mul(0.5*dt, k1v))
    k2r, k2v = v2, accel(r2)
    # k3
    r3 = v_add(r, v_mul(0.5*dt, k2r))
    v3 = v_add(v, v_mul(0.5*dt, k2v))
    k3r, k3v = v3, accel(r3)
    # k4
    r4 = v_add(r, v_mul(dt, k3r))
    v4 = v_add(v, v_mul(dt, k3v))
    k4r, k4v = v4, accel(r4)

    def combine(k1, k2, k3, k4):
        return v_mul(dt/6.0, v_add(
            v_add(k1, v_mul(2.0, k2)),
            v_add(v_mul(2.0, k3), k4)
        ))

    r_next = v_add(r, combine(k1r, k2r, k3r, k4r))
    v_next = v_add(v, combine(k1v, k2v, k3v, k4v))
    return r_next, v_next


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------

class OrbitVelNode(Node):

    def __init__(self):
        super().__init__("orbit_vel_node")

        # Declare parameters
        self.declare_parameter("mu",           398600.4418)
        self.declare_parameter("dt_sim",       0.00833)
        self.declare_parameter("prim_path",    "/World/Sat")
        self.declare_parameter("attractor",    "/World/Earth")
        self.declare_parameter("orbit_type",   "circular")
        # circular params
        self.declare_parameter("radius",       6778.0)
        self.declare_parameter("plane",        "xy")
        # elements params
        self.declare_parameter("a",            6778.0)
        self.declare_parameter("e",            0.0)
        self.declare_parameter("inc",          0.0)
        self.declare_parameter("raan",         0.0)
        self.declare_parameter("argp",         0.0)
        self.declare_parameter("nu",           0.0)
        # publish rate
        self.declare_parameter("publish_rate", 30.0)

        # Read parameters
        self._mu        = self.get_parameter("mu").value
        self._dt_sim    = self.get_parameter("dt_sim").value
        self._prim_path = self.get_parameter("prim_path").value
        self._attractor = self.get_parameter("attractor").value
        orbit_type      = self.get_parameter("orbit_type").value
        publish_rate    = self.get_parameter("publish_rate").value

        # Initial conditions
        if orbit_type == "circular":
            radius = self.get_parameter("radius").value
            plane  = self.get_parameter("plane").value
            self._r, self._v = circular_ic(self._mu, radius, plane)
        elif orbit_type == "elements":
            self._r, self._v = elements_ic(
                self._mu,
                self.get_parameter("a").value,
                self.get_parameter("e").value,
                self.get_parameter("inc").value,
                self.get_parameter("raan").value,
                self.get_parameter("argp").value,
                self.get_parameter("nu").value,
            )
        else:
            self.get_logger().error(f"Unknown orbit_type: {orbit_type}")
            raise ValueError(f"Unknown orbit_type: {orbit_type}")

        # Commanded thrust accumulator (km/s² in inertial frame)
        self._a_cmd: Vec3 = (0.0, 0.0, 0.0)
        # Pending instantaneous impulse (km/s), applied once then zeroed
        self._dv_pending: Vec3 = (0.0, 0.0, 0.0)
        self._has_pending_dv: bool = False

        # Publisher
        self._pub = self.create_publisher(OrbitState, "orbit_state", 10)

        # Subscriber
        self._sub = self.create_subscription(
            ThrustCmd, "cmd_thrust", self._on_thrust, 10
        )

        # Integrator timer — runs at dt_sim rate
        self._integ_timer = self.create_timer(
            self._dt_sim, self._integrate_step
        )

        # Publish timer — runs at publish_rate
        self._pub_timer = self.create_timer(
            1.0 / publish_rate, self._publish_state
        )

        self.get_logger().info(
            f"orbit_vel_node started: {self._prim_path} "
            f"orbit_type={orbit_type} mu={self._mu} dt_sim={self._dt_sim}"
        )

    # ------------------------------------------------------------------

    def _integrate_step(self):
        """Advance state by one dt_sim."""
        # Apply pending instantaneous impulse
        if self._has_pending_dv:
            dv = self._dv_pending
            self._v = v_add(self._v, dv)
            self._dv_pending    = (0.0, 0.0, 0.0)
            self._has_pending_dv = False

        self._r, self._v = rk4_step(
            self._mu, self._r, self._v, self._dt_sim, self._a_cmd
        )

    def _publish_state(self):
        """Publish current state."""
        msg = OrbitState()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        msg.prim_path       = self._prim_path
        msg.attractor_path  = self._attractor
        msg.position.x      = self._r[0]
        msg.position.y      = self._r[1]
        msg.position.z      = self._r[2]
        msg.velocity.x      = self._v[0]
        msg.velocity.y      = self._v[1]
        msg.velocity.z      = self._v[2]
        msg.mu              = self._mu
        self._pub.publish(msg)

    def _on_thrust(self, msg: ThrustCmd):
        """Handle incoming thrust command."""
        if msg.mode == ThrustCmd.MODE_IMPULSE:
            # Instantaneous delta-v (km/s), applied at next integrator tick
            self._dv_pending = (msg.accel.x, msg.accel.y, msg.accel.z)
            self._has_pending_dv = True
            self.get_logger().info(
                f"Impulse received: dv={self._dv_pending}"
            )
        elif msg.mode == ThrustCmd.MODE_CONTINUOUS:
            # Continuous acceleration (km/s²), held until next command
            self._a_cmd = (msg.accel.x, msg.accel.y, msg.accel.z)
        elif msg.mode == ThrustCmd.MODE_CLEAR:
            self._a_cmd          = (0.0, 0.0, 0.0)
            self._dv_pending     = (0.0, 0.0, 0.0)
            self._has_pending_dv = False


def main(args=None):
    rclpy.init(args=args)
    node = OrbitVelNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()