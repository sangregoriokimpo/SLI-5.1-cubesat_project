#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from orbit_interfaces.msg import OrbitState, ThrustCmd
from std_msgs.msg import Bool


def v_add(a, b): return (a[0]+b[0], a[1]+b[1], a[2]+b[2])
def v_sub(a, b): return (a[0]-b[0], a[1]-b[1], a[2]-b[2])
def v_mul(s, a): return (s*a[0], s*a[1], s*a[2])
def v_norm(a):   return math.sqrt(a[0]**2 + a[1]**2 + a[2]**2)
def v_dot(a, b): return a[0]*b[0] + a[1]*b[1] + a[2]*b[2]


def v_cross(a, b):
    return (
        a[1]*b[2] - a[2]*b[1],
        a[2]*b[0] - a[0]*b[2],
        a[0]*b[1] - a[1]*b[0],
    )


def v_normalize(a):
    n = v_norm(a)
    if n < 1e-12:
        return (0.0, 0.0, 0.0)
    return (a[0]/n, a[1]/n, a[2]/n)


def lvlh_to_inertial_matrix(r_chief, v_chief):
    x_hat = v_normalize(r_chief)
    h = v_cross(r_chief, v_chief)
    z_hat = v_normalize(h)
    y_hat = v_cross(z_hat, x_hat)
    return x_hat, y_hat, z_hat


def lvlh_to_inertial(dr_lvlh, r_chief, v_chief):
    x_hat, y_hat, z_hat = lvlh_to_inertial_matrix(r_chief, v_chief)
    dx, dy, dz = dr_lvlh
    return (
        x_hat[0]*dx + y_hat[0]*dy + z_hat[0]*dz,
        x_hat[1]*dx + y_hat[1]*dy + z_hat[1]*dz,
        x_hat[2]*dx + y_hat[2]*dy + z_hat[2]*dz,
    )


def inertial_to_lvlh(dr_inertial, r_chief, v_chief):
    x_hat, y_hat, z_hat = lvlh_to_inertial_matrix(r_chief, v_chief)
    dx = v_dot(dr_inertial, x_hat)
    dy = v_dot(dr_inertial, y_hat)
    dz = v_dot(dr_inertial, z_hat)
    return (dx, dy, dz)


def cw_rk4_step(n, rho, rho_dot, dt, a_cmd=(0.0, 0.0, 0.0)):
    def f(r, v):
        x, y, z = r
        xd, yd, zd = v
        ax, ay, az = a_cmd
        xdd = 2*n*yd + 3*n*n*x + ax
        ydd = -2*n*xd         + ay
        zdd = -n*n*z          + az
        return v, (xdd, ydd, zdd)

    def add(a, b): return v_add(a, b)
    def mul(s, a): return v_mul(s, a)

    k1r, k1v = f(rho, rho_dot)
    r2 = add(rho,     mul(0.5*dt, k1r))
    v2 = add(rho_dot, mul(0.5*dt, k1v))
    k2r, k2v = f(r2, v2)
    r3 = add(rho,     mul(0.5*dt, k2r))
    v3 = add(rho_dot, mul(0.5*dt, k2v))
    k3r, k3v = f(r3, v3)
    r4 = add(rho,     mul(dt, k3r))
    v4 = add(rho_dot, mul(dt, k3v))
    k4r, k4v = f(r4, v4)

    def comb(k1, k2, k3, k4):
        return mul(dt/6.0, add(add(k1, mul(2.0, k2)),
                               add(mul(2.0, k3), k4)))

    rho_next     = add(rho,     comb(k1r, k2r, k3r, k4r))
    rho_dot_next = add(rho_dot, comb(k1v, k2v, k3v, k4v))
    return rho_next, rho_dot_next


def thrust_to_accel(throttle, pitch, yaw):
    if throttle == 0.0:
        return (0.0, 0.0, 0.0)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw),   math.sin(yaw)
    return (throttle * cp * cy,
            throttle * cp * sy,
            throttle * sp)


class CWNode(Node):

    def __init__(self):
        super().__init__("cw_node")

        self.declare_parameter("mu",           398600.4418)
        self.declare_parameter("dt_sim",       0.00833)
        self.declare_parameter("prim_path",    "/World/Sat3")
        self.declare_parameter("chief_topic",  "/sat1/orbit_state")
        self.declare_parameter("dr_x",         1.0)
        self.declare_parameter("dr_y",         0.0)
        self.declare_parameter("dr_z",         0.5)
        self.declare_parameter("dv_x",         0.0)
        self.declare_parameter("dv_y",         0.001)
        self.declare_parameter("dv_z",         0.0)
        self.declare_parameter("publish_rate", 30.0)

        self._mu          = self.get_parameter("mu").value
        self._dt_sim      = self.get_parameter("dt_sim").value
        self._prim_path   = self.get_parameter("prim_path").value
        self._chief_topic = self.get_parameter("chief_topic").value
        publish_rate      = self.get_parameter("publish_rate").value

        self._rho = (
            self.get_parameter("dr_x").value,
            self.get_parameter("dr_y").value,
            self.get_parameter("dr_z").value,
        )
        self._rho_dot = (
            self.get_parameter("dv_x").value,
            self.get_parameter("dv_y").value,
            self.get_parameter("dv_z").value,
        )

        self._r_chief       = None
        self._v_chief       = None
        self._chief_received = False
        self._a_cmd         = (0.0, 0.0, 0.0)
        self._paused        = False   # pause gate

        # Subscriptions
        self._chief_sub = self.create_subscription(
            OrbitState, self._chief_topic, self._on_chief_state, 10
        )
        self._thrust_sub = self.create_subscription(
            ThrustCmd, "cmd_thrust", self._on_thrust, 10
        )
        # Global pause gate
        self._pause_sub = self.create_subscription(
            Bool, "/sim_pause", self._on_pause, 10
        )

        # Publisher
        self._pub = self.create_publisher(OrbitState, "orbit_state", 10)

        # Timers
        self.create_timer(self._dt_sim,       self._integrate_step)
        self.create_timer(1.0 / publish_rate, self._publish_state)

        self.get_logger().info(
            f"cw_node started: {self._prim_path} "
            f"chief={self._chief_topic} "
            f"dr={self._rho} dv={self._rho_dot}"
        )

    # ------------------------------------------------------------------

    def _on_pause(self, msg: Bool):
        if msg.data != self._paused:
            self._paused = msg.data
            state_str = "PAUSED" if self._paused else "RESUMED"
            self.get_logger().info(f"[{self._prim_path}] {state_str}")

    def _on_chief_state(self, msg: OrbitState):
        # Always update chief state even when paused — keeps reference fresh
        # for when we resume or do a chief swap
        self._r_chief = (msg.position.x, msg.position.y, msg.position.z)
        self._v_chief = (msg.velocity.x, msg.velocity.y, msg.velocity.z)
        self._chief_received = True

    def _on_thrust(self, msg: ThrustCmd):
        self._a_cmd = thrust_to_accel(
            msg.throttle, msg.gimbal_pitch, msg.gimbal_yaw
        )
        self.get_logger().info(
            f"ThrustCmd LVLH: throttle={msg.throttle:.4f} "
            f"→ a_cmd={self._a_cmd}"
        )

    def _integrate_step(self):
        if self._paused or not self._chief_received:
            return

        r_chief = self._r_chief
        r_mag   = v_norm(r_chief)
        if r_mag < 1e-6:
            return

        n = math.sqrt(self._mu / r_mag**3)

        self._rho, self._rho_dot = cw_rk4_step(
            n, self._rho, self._rho_dot, self._dt_sim, self._a_cmd
        )

    def _publish_state(self):
        if self._paused or not self._chief_received:
            return

        r_chief = self._r_chief
        v_chief = self._v_chief

        dr_inertial = lvlh_to_inertial(self._rho,     r_chief, v_chief)
        dv_inertial = lvlh_to_inertial(self._rho_dot, r_chief, v_chief)

        r_body = v_add(r_chief, dr_inertial)
        v_body = v_add(v_chief, dv_inertial)

        msg = OrbitState()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        msg.body_id         = self._prim_path
        msg.position.x      = r_body[0]
        msg.position.y      = r_body[1]
        msg.position.z      = r_body[2]
        msg.velocity.x      = v_body[0]
        msg.velocity.y      = v_body[1]
        msg.velocity.z      = v_body[2]
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CWNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()


if __name__ == "__main__":
    main()