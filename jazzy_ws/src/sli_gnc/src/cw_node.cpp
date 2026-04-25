#include <cmath>
#include <array>
#include <string>
#include <chrono>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "orbit_interfaces/msg/orbit_state.hpp"
#include "orbit_interfaces/msg/thrust_cmd.hpp"
#include "std_msgs/msg/bool.hpp"

using V3 = std::array<double, 3>;
using Q4 = std::array<double, 4>;


static V3 v_add(V3 a, V3 b) { return {a[0]+b[0], a[1]+b[1], a[2]+b[2]}; }
static V3 v_mul(double s, V3 a) { return {s*a[0], s*a[1], s*a[2]}; }
static double v_norm(V3 a) { return std::sqrt(a[0]*a[0]+a[1]*a[1]+a[2]*a[2]); }

static V3 v_cross(V3 a, V3 b) {
    return {a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]};
}

static V3 v_normalize(V3 a) {
    double n = v_norm(a);
    if (n < 1e-12) return {0,0,0};
    return {a[0]/n, a[1]/n, a[2]/n};
}

static V3 lvlh_to_inertial(V3 vec, V3 r_chief, V3 v_chief) {
    V3 x = v_normalize(r_chief);
    V3 z = v_normalize(v_cross(r_chief, v_chief));
    V3 y = v_cross(z, x);
    return {
        x[0]*vec[0] + y[0]*vec[1] + z[0]*vec[2],
        x[1]*vec[0] + y[1]*vec[1] + z[1]*vec[2],
        x[2]*vec[0] + y[2]*vec[1] + z[2]*vec[2],
    };
}


static Q4 q_mul(Q4 p, Q4 q) {
    return {
        p[0]*q[0] - p[1]*q[1] - p[2]*q[2] - p[3]*q[3],
        p[0]*q[1] + p[1]*q[0] + p[2]*q[3] - p[3]*q[2],
        p[0]*q[2] - p[1]*q[3] + p[2]*q[0] + p[3]*q[1],
        p[0]*q[3] + p[1]*q[2] - p[2]*q[1] + p[3]*q[0],
    };
}

static Q4 q_normalize(Q4 q) {
    double n = std::sqrt(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]);
    if (n < 1e-12) return {1,0,0,0};
    return {q[0]/n, q[1]/n, q[2]/n, q[3]/n};
}

static Q4 q_integrate(Q4 q, V3 omega, double dt) {
    Q4 om = {0.0, omega[0], omega[1], omega[2]};
    Q4 qd = q_mul(q, om);
    return q_normalize({
        q[0] + 0.5*qd[0]*dt,
        q[1] + 0.5*qd[1]*dt,
        q[2] + 0.5*qd[2]*dt,
        q[3] + 0.5*qd[3]*dt,
    });
}


static std::pair<V3,V3> cw_rk4_step(double n, V3 rho, V3 rho_dot,
                                      double dt, V3 a) {
    auto f = [&](V3 r, V3 v) -> std::pair<V3,V3> {
        double xdd =  2*n*v[1] + 3*n*n*r[0] + a[0];
        double ydd = -2*n*v[0]               + a[1];
        double zdd =   -n*n*r[2]             + a[2];
        return {v, {xdd, ydd, zdd}};
    };

    auto [k1r, k1v] = f(rho, rho_dot);
    V3 r2 = v_add(rho,     v_mul(0.5*dt, k1r));
    V3 v2 = v_add(rho_dot, v_mul(0.5*dt, k1v));
    auto [k2r, k2v] = f(r2, v2);
    V3 r3 = v_add(rho,     v_mul(0.5*dt, k2r));
    V3 v3 = v_add(rho_dot, v_mul(0.5*dt, k2v));
    auto [k3r, k3v] = f(r3, v3);
    V3 r4 = v_add(rho,     v_mul(dt, k3r));
    V3 v4 = v_add(rho_dot, v_mul(dt, k3v));
    auto [k4r, k4v] = f(r4, v4);

    auto comb = [&](V3 k1, V3 k2, V3 k3, V3 k4) -> V3 {
        return v_mul(dt/6.0,
            v_add(v_add(k1, v_mul(2.0, k2)),
                  v_add(v_mul(2.0, k3), k4)));
    };
    return {v_add(rho,     comb(k1r,k2r,k3r,k4r)),
            v_add(rho_dot, comb(k1v,k2v,k3v,k4v))};
}


class CWNode : public rclcpp::Node {
public:
    CWNode() : Node("cw_node") {
        declare_parameter("mu",           398600.4418);
        declare_parameter("dt_sim",       0.00833);     // 120Hz default
        declare_parameter("publish_rate", 30.0);
        declare_parameter("prim_path",    "/World/Sat3");
        declare_parameter("chief_topic",  "/sat1/orbit_state");
        declare_parameter("dr_x",         1.0);
        declare_parameter("dr_y",         0.0);
        declare_parameter("dr_z",         0.5);
        declare_parameter("dv_x",         0.0);
        declare_parameter("dv_y",         0.001);
        declare_parameter("dv_z",         0.0);

        mu_          = get_parameter("mu").as_double();
        dt_sim_      = get_parameter("dt_sim").as_double();
        prim_path_   = get_parameter("prim_path").as_string();
        chief_topic_ = get_parameter("chief_topic").as_string();
        auto publish_rate = get_parameter("publish_rate").as_double();

        rho_     = {get_parameter("dr_x").as_double(),
                    get_parameter("dr_y").as_double(),
                    get_parameter("dr_z").as_double()};
        rho_dot_ = {get_parameter("dv_x").as_double(),
                    get_parameter("dv_y").as_double(),
                    get_parameter("dv_z").as_double()};

        pub_ = create_publisher<orbit_interfaces::msg::OrbitState>(
            "orbit_state", 10);

        // Chief state subscription — always update even when paused
        sub_chief_ = create_subscription<orbit_interfaces::msg::OrbitState>(
            chief_topic_, 10,
            [this](orbit_interfaces::msg::OrbitState::SharedPtr msg) {
                r_chief_ = {msg->position.x, msg->position.y, msg->position.z};
                v_chief_ = {msg->velocity.x, msg->velocity.y, msg->velocity.z};
                chief_received_ = true;
            });

        sub_thrust_ = create_subscription<orbit_interfaces::msg::ThrustCmd>(
            "cmd_thrust", 10,
            [this](orbit_interfaces::msg::ThrustCmd::SharedPtr msg) {
                f_lvlh_ = {msg->fx, msg->fy, msg->fz};
                omega_  = {msg->wx, msg->wy, msg->wz};
            });

        rclcpp::QoS latch(1);
        latch.transient_local().reliable();
        sub_pause_ = create_subscription<std_msgs::msg::Bool>(
            "/sim_pause", latch,
            [this](std_msgs::msg::Bool::SharedPtr msg) {
                if (msg->data != paused_) {
                    paused_ = msg->data;
                    RCLCPP_INFO(get_logger(), "[%s] %s",
                        prim_path_.c_str(),
                        paused_ ? "PAUSED" : "RESUMED");
                }
            });

        // Decoupled timers
        auto dt_ms = std::chrono::duration<double>(dt_sim_);
        integrate_timer_ = create_wall_timer(dt_ms,
            [this]() { integrate_step(); });

        auto pub_ms = std::chrono::duration<double>(1.0 / publish_rate);
        publish_timer_ = create_wall_timer(pub_ms,
            [this]() { publish_state(); });

        RCLCPP_INFO(get_logger(),
            "cw_node: %s  chief=%s  dt=%.5fs  pub=%.1fHz",
            prim_path_.c_str(), chief_topic_.c_str(),
            dt_sim_, publish_rate);
    }

private:
    double      mu_;
    double      dt_sim_;
    std::string prim_path_;
    std::string chief_topic_;
    bool        paused_         = false;
    bool        chief_received_ = false;

    V3 rho_     = {0,0,0};
    V3 rho_dot_ = {0,0,0};
    V3 r_chief_ = {0,0,0};
    V3 v_chief_ = {0,0,0};
    Q4 q_       = {1,0,0,0};
    V3 f_lvlh_  = {0,0,0};
    V3 omega_   = {0,0,0};

    rclcpp::Publisher<orbit_interfaces::msg::OrbitState>::SharedPtr pub_;
    rclcpp::Subscription<orbit_interfaces::msg::OrbitState>::SharedPtr sub_chief_;
    rclcpp::Subscription<orbit_interfaces::msg::ThrustCmd>::SharedPtr sub_thrust_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_pause_;
    rclcpp::TimerBase::SharedPtr integrate_timer_;
    rclcpp::TimerBase::SharedPtr publish_timer_;

    void integrate_step() {
        if (paused_ || !chief_received_) return;

        double r_mag = v_norm(r_chief_);
        if (r_mag < 1e-6) return;

        double n = std::sqrt(mu_ / (r_mag*r_mag*r_mag));

        auto [rn, vn] = cw_rk4_step(n, rho_, rho_dot_, dt_sim_, f_lvlh_);
        rho_ = rn; rho_dot_ = vn;

        if (omega_[0] || omega_[1] || omega_[2])
            q_ = q_integrate(q_, omega_, dt_sim_);
    }

    void publish_state() {
        if (paused_ || !chief_received_) return;

        V3 dr = lvlh_to_inertial(rho_,     r_chief_, v_chief_);
        V3 dv = lvlh_to_inertial(rho_dot_, r_chief_, v_chief_);

        V3 r_body = v_add(r_chief_, dr);
        V3 v_body = v_add(v_chief_, dv);

        auto msg = orbit_interfaces::msg::OrbitState();
        msg.header.stamp    = now();
        msg.header.frame_id = "world";
        msg.body_id         = prim_path_;
        msg.position.x      = r_body[0];
        msg.position.y      = r_body[1];
        msg.position.z      = r_body[2];
        msg.velocity.x      = v_body[0];
        msg.velocity.y      = v_body[1];
        msg.velocity.z      = v_body[2];
        msg.attitude.w      = q_[0];
        msg.attitude.x      = q_[1];
        msg.attitude.y      = q_[2];
        msg.attitude.z      = q_[3];
        pub_->publish(msg);
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CWNode>());
    rclcpp::shutdown();
    return 0;
}