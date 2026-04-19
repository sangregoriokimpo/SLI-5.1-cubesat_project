#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from orbit_interfaces.msg import OrbitState, ThrustCmd
from std_msgs.msg import Bool
 
 
# ---------------------------------------------------------------------------
# Orbital math
# ---------------------------------------------------------------------------
 
def v_add(a, b): return (a[0]+b[0], a[1]+b[1], a[2]+b[2])
def v_mul(s, a): return (s*a[0], s*a[1], s*a[2])
def v_norm(a):   return math.sqrt(a[0]**2 + a[1]**2 + a[2]**2)
 
 
def rot_z(angle, v):
    c, s = math.cos(angle), math.sin(angle)
    return (c*v[0] - s*v[1], s*v[0] + c*v[1], v[2])
 
 
def rot_x(angle, v):
    c, s = math.cos(angle), math.sin(angle)
    return (v[0], c*v[1] - s*v[2], s*v[1] + c*v[2])
 
 
def circular_ic(mu, radius, plane):
    spd = math.sqrt(mu / radius)
    plane = plane.lower()
    if plane == "xy": return (radius, 0.0, 0.0), (0.0, spd, 0.0)
    if plane == "xz": return (radius, 0.0, 0.0), (0.0, 0.0, spd)
    if plane == "yz": return (0.0, radius, 0.0), (0.0, 0.0, spd)
    raise ValueError(f"Unknown plane: {plane}")
 
 
def elements_ic(mu, a, e, inc_deg, raan_deg, argp_deg, nu_deg):
    inc  = math.radians(inc_deg)
    raan = math.radians(raan_deg)
    argp = math.radians(argp_deg)
    nu   = math.radians(nu_deg)
    p    = a * (1.0 - e * e)
    rm   = p / (1.0 + e * math.cos(nu))
    r_pqw = (rm * math.cos(nu), rm * math.sin(nu), 0.0)
    fac   = math.sqrt(mu / p)
    v_pqw = (-fac * math.sin(nu), fac * (e + math.cos(nu)), 0.0)
    def world(vec): return rot_z(raan, rot_x(inc, rot_z(argp, vec)))
    return world(r_pqw), world(v_pqw)
 
 
def rk4_step(mu, r, v, dt, a_cmd):
    def gravity(rr):
        d = v_norm(rr)
        if d < 1e-12: return (0.0, 0.0, 0.0)
        s = -mu / d**3
        return (s*rr[0], s*rr[1], s*rr[2])
 
    def accel(rr):
        g = gravity(rr)
        return (g[0]+a_cmd[0], g[1]+a_cmd[1], g[2]+a_cmd[2])
 
    k1r, k1v = v, accel(r)
    r2 = v_add(r, v_mul(0.5*dt, k1r)); v2 = v_add(v, v_mul(0.5*dt, k1v))
    k2r, k2v = v2, accel(r2)
    r3 = v_add(r, v_mul(0.5*dt, k2r)); v3 = v_add(v, v_mul(0.5*dt, k2v))
    k3r, k3v = v3, accel(r3)
    r4 = v_add(r, v_mul(dt, k3r));     v4 = v_add(v, v_mul(dt, k3v))
    k4r, k4v = v4, accel(r4)
 
    def comb(k1, k2, k3, k4):
        return v_mul(dt/6.0, v_add(v_add(k1, v_mul(2.0, k2)),
                                    v_add(v_mul(2.0, k3), k4)))
    return v_add(r, comb(k1r, k2r, k3r, k4r)), \
           v_add(v, comb(k1v, k2v, k3v, k4v))
 
 
def thrust_to_accel(throttle, pitch, yaw):
    if throttle == 0.0:
        return (0.0, 0.0, 0.0)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw),   math.sin(yaw)
    return (throttle * cp * cy,
            throttle * cp * sy,
            throttle * sp)
 
 
# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------
 
class OrbitVelNode(Node):
 
    def __init__(self):
        super().__init__("orbit_vel_node")
 
        self.declare_parameter("mu",           398600.4418)
        self.declare_parameter("dt_sim",       0.00833)
        self.declare_parameter("prim_path",    "/World/Sat")
        self.declare_parameter("attractor",    "/World/Earth")
        self.declare_parameter("orbit_type",   "circular")
        self.declare_parameter("radius",       6778.0)
        self.declare_parameter("plane",        "xy")
        self.declare_parameter("a",            6778.0)
        self.declare_parameter("e",            0.0)
        self.declare_parameter("inc",          0.0)
        self.declare_parameter("raan",         0.0)
        self.declare_parameter("argp",         0.0)
        self.declare_parameter("nu",           0.0)
        self.declare_parameter("publish_rate", 30.0)
 
        self._mu        = self.get_parameter("mu").value
        self._dt_sim    = self.get_parameter("dt_sim").value
        self._prim_path = self.get_parameter("prim_path").value
        orbit_type      = self.get_parameter("orbit_type").value
        publish_rate    = self.get_parameter("publish_rate").value
 
        if orbit_type == "circular":
            self._r, self._v = circular_ic(
                self._mu,
                self.get_parameter("radius").value,
                self.get_parameter("plane").value,
            )
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
            raise ValueError(f"Unknown orbit_type: {orbit_type}")
 
        self._a_cmd  = (0.0, 0.0, 0.0)
        self._paused = False          # pause gate
 
        self._pub = self.create_publisher(OrbitState, "orbit_state", 10)
 
        self._sub_thrust = self.create_subscription(
            ThrustCmd, "cmd_thrust", self._on_thrust, 10
        )
        # Global pause gate — all physics nodes share this topic
        self._sub_pause = self.create_subscription(
            Bool, "/sim_pause", self._on_pause, 10
        )
 
        self.create_timer(self._dt_sim,       self._integrate_step)
        self.create_timer(1.0 / publish_rate, self._publish_state)
 
        self.get_logger().info(
            f"orbit_vel_node started: {self._prim_path} "
            f"orbit_type={orbit_type} mu={self._mu} dt_sim={self._dt_sim}"
        )
 
    # ------------------------------------------------------------------
 
    def _on_pause(self, msg: Bool):
        if msg.data != self._paused:
            self._paused = msg.data
            state_str = "PAUSED" if self._paused else "RESUMED"
            self.get_logger().info(f"[{self._prim_path}] {state_str}")
 
    def _integrate_step(self):
        if self._paused:
            return
        self._r, self._v = rk4_step(
            self._mu, self._r, self._v, self._dt_sim, self._a_cmd
        )
 
    def _publish_state(self):
        if self._paused:
            return
        msg = OrbitState()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        msg.body_id         = self._prim_path
        msg.position.x      = self._r[0]
        msg.position.y      = self._r[1]
        msg.position.z      = self._r[2]
        msg.velocity.x      = self._v[0]
        msg.velocity.y      = self._v[1]
        msg.velocity.z      = self._v[2]
        msg.attitude.w      = 1.0
        msg.attitude.x      = 0.0
        msg.attitude.y      = 0.0
        msg.attitude.z      = 0.0
        self._pub.publish(msg)
 
    def _on_thrust(self, msg: ThrustCmd):
        self._a_cmd = thrust_to_accel(
            msg.throttle, msg.gimbal_pitch, msg.gimbal_yaw
        )
        self.get_logger().info(
            f"ThrustCmd: throttle={msg.throttle:.4f} "
            f"pitch={msg.gimbal_pitch:.3f} yaw={msg.gimbal_yaw:.3f} "
            f"→ a_cmd={self._a_cmd}"
        )
 
 
def main(args=None):
    rclpy.init(args=args)
    node = OrbitVelNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
 
 
if __name__ == "__main__":
    main()