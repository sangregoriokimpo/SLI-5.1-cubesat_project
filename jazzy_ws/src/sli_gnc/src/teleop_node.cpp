#include <cmath>
#include <csignal>
#include <cstdio>
#include <memory>
#include <string>
#include <termios.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "orbit_interfaces/msg/orbit_state.hpp"
#include "orbit_interfaces/msg/thrust_cmd.hpp"
#include "simulation_interfaces/srv/set_simulation_state.hpp"
#include "simulation_interfaces/msg/simulation_state.hpp"
#include "std_msgs/msg/bool.hpp"

// ── constants ────────────────────────────────────────────────────────────────
static constexpr double THRUST_MAG = 1e-4;   // km/s² per keypress
static constexpr int    STATE_PLAYING = 1;
static constexpr int    STATE_PAUSED  = 2;

// ── terminal raw-mode helpers ─────────────────────────────────────────────────
struct TermRAII {
    termios old{};
    bool    active = false;

    void enable() {
        tcgetattr(STDIN_FILENO, &old);
        termios raw = old;
        raw.c_lflag &= ~(ICANON | ECHO);
        raw.c_cc[VMIN]  = 0;
        raw.c_cc[VTIME] = 1;   // 100 ms timeout
        tcsetattr(STDIN_FILENO, TCSANOW, &raw);
        active = true;
    }
    ~TermRAII() {
        if (active) tcsetattr(STDIN_FILENO, TCSANOW, &old);
    }
};

// ── math helpers ──────────────────────────────────────────────────────────────
using V3 = std::array<double, 3>;

static V3 normalize(V3 a) {
    double n = std::sqrt(a[0]*a[0] + a[1]*a[1] + a[2]*a[2]);
    if (n < 1e-12) return {0,0,0};
    return {a[0]/n, a[1]/n, a[2]/n};
}
static V3 cross(V3 a, V3 b) {
    return { a[1]*b[2]-a[2]*b[1],
             a[2]*b[0]-a[0]*b[2],
             a[0]*b[1]-a[1]*b[0] };
}
static double dot(V3 a, V3 b) {
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

// Rotate a LVLH vector into inertial given chief r, v
static V3 lvlh_to_inertial(V3 lvlh, V3 r, V3 v) {
    V3 x_hat = normalize(r);
    V3 z_hat = normalize(cross(r, v));
    V3 y_hat = cross(z_hat, x_hat);
    // columns: x_hat, y_hat, z_hat
    return {
        x_hat[0]*lvlh[0] + y_hat[0]*lvlh[1] + z_hat[0]*lvlh[2],
        x_hat[1]*lvlh[0] + y_hat[1]*lvlh[1] + z_hat[1]*lvlh[2],
        x_hat[2]*lvlh[0] + y_hat[2]*lvlh[1] + z_hat[2]*lvlh[2],
    };
}

// Convert inertial acceleration vector to ThrustCmd gimbal angles
static void accel_to_gimbal(V3 a, double& throttle, double& pitch, double& yaw) {
    throttle = std::sqrt(a[0]*a[0] + a[1]*a[1] + a[2]*a[2]);
    if (throttle < 1e-15) { pitch = yaw = 0.0; return; }
    pitch = std::asin(a[2] / throttle);
    yaw   = std::atan2(a[1], a[0]);
}

// ── node ──────────────────────────────────────────────────────────────────────
class TeleopNode : public rclcpp::Node {
public:
    TeleopNode() : Node("teleop_node") {
        this->declare_parameter("target", "sat1");
        this->declare_parameter("thrust_mag", THRUST_MAG);

        target_    = this->get_parameter("target").as_string();
        thrust_mag_ = this->get_parameter("thrust_mag").as_double();

        // ThrustCmd publisher — namespace matches target
        thrust_pub_ = this->create_publisher<orbit_interfaces::msg::ThrustCmd>(
            "/" + target_ + "/cmd_thrust", 10);

        // /sim_pause publisher — latched so new subscribers see current state
        rclcpp::QoS latch_qos(1);
        latch_qos.transient_local();
        pause_pub_ = this->create_publisher<std_msgs::msg::Bool>(
            "/sim_pause", latch_qos);

        // /set_simulation_state service client
        sim_state_cli_ = this->create_client<
            simulation_interfaces::srv::SetSimulationState>(
            "/set_simulation_state");

        // OrbitState subscriber for LVLH mode
        orbit_sub_ = this->create_subscription<
            orbit_interfaces::msg::OrbitState>(
            "/" + target_ + "/orbit_state", 10,
            [this](orbit_interfaces::msg::OrbitState::SharedPtr msg) {
                r_chief_ = {msg->position.x, msg->position.y, msg->position.z};
                v_chief_ = {msg->velocity.x, msg->velocity.y, msg->velocity.z};
                has_state_ = true;
            });

        print_help();
    }

    void run() {
        TermRAII term;
        term.enable();

        rclcpp::Rate rate(30);
        char c = 0;

        while (rclcpp::ok()) {
            rclcpp::spin_some(shared_from_this());

            int n = read(STDIN_FILENO, &c, 1);
            if (n <= 0) { rate.sleep(); continue; }

            switch (c) {
            // ── frame toggle ──────────────────────────────────────────────
            case 'f': case 'F':
                lvlh_mode_ = !lvlh_mode_;
                printf("\r\033[K[teleop] Frame: %s\n",
                       lvlh_mode_ ? "LVLH (radial/along-track/cross-track)"
                                  : "INERTIAL (X/Y/Z)");
                fflush(stdout);
                break;

            // ── pause toggle ──────────────────────────────────────────────
            case 'p': case 'P':
                toggle_pause();
                break;

            // ── thrust keys ───────────────────────────────────────────────
            // LVLH:     radial=X, along-track=Y, cross-track=Z
            // INERTIAL: X, Y, Z
            case 'i': publish_thrust({ 0,  1,  0}); break;  // +along / +Y
            case 'k': publish_thrust({ 0, -1,  0}); break;  // -along / -Y
            case 'j': publish_thrust({-1,  0,  0}); break;  // -radial / -X
            case 'l': publish_thrust({ 1,  0,  0}); break;  // +radial / +X
            case 'u': publish_thrust({ 0,  0,  1}); break;  // +cross  / +Z
            case 'o': publish_thrust({ 0,  0, -1}); break;  // -cross  / -Z

            // ── zero thrust ───────────────────────────────────────────────
            case ' ':
                publish_zero_thrust();
                printf("\r\033[K[teleop] Thrust zeroed\n");
                fflush(stdout);
                break;

            // ── quit ──────────────────────────────────────────────────────
            case 'q': case 'Q': case 3:   // Ctrl-C
                publish_zero_thrust();
                printf("\r\033[K[teleop] Quit\n");
                return;

            default: break;
            }

            rate.sleep();
        }
    }

private:
    // ── state ─────────────────────────────────────────────────────────────
    std::string target_;
    double      thrust_mag_;
    bool        lvlh_mode_ = false;
    bool        paused_    = false;
    bool        has_state_ = false;
    V3          r_chief_   = {0, 0, 0};
    V3          v_chief_   = {0, 0, 0};

    // ── ROS handles ───────────────────────────────────────────────────────
    rclcpp::Publisher<orbit_interfaces::msg::ThrustCmd>::SharedPtr   thrust_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr                 pause_pub_;
    rclcpp::Client<simulation_interfaces::srv::SetSimulationState>::SharedPtr sim_state_cli_;
    rclcpp::Subscription<orbit_interfaces::msg::OrbitState>::SharedPtr orbit_sub_;

    // ── helpers ───────────────────────────────────────────────────────────
    void publish_thrust(V3 dir_unit) {
        // dir_unit is in LVLH frame when lvlh_mode_, inertial otherwise
        V3 accel_inertial;

        if (lvlh_mode_) {
            if (!has_state_) {
                printf("\r\033[K[teleop] LVLH mode: waiting for orbit_state...\n");
                fflush(stdout);
                return;
            }
            // Scale by thrust_mag, rotate LVLH → inertial
            V3 lvlh_accel = {
                dir_unit[0] * thrust_mag_,
                dir_unit[1] * thrust_mag_,
                dir_unit[2] * thrust_mag_,
            };
            accel_inertial = lvlh_to_inertial(lvlh_accel, r_chief_, v_chief_);
        } else {
            accel_inertial = {
                dir_unit[0] * thrust_mag_,
                dir_unit[1] * thrust_mag_,
                dir_unit[2] * thrust_mag_,
            };
        }

        double throttle, pitch, yaw;
        accel_to_gimbal(accel_inertial, throttle, pitch, yaw);

        auto msg = orbit_interfaces::msg::ThrustCmd();
        msg.header.stamp    = this->get_clock()->now();
        msg.header.frame_id = "world";
        msg.body_id         = "/" + target_;
        msg.throttle        = throttle;
        msg.gimbal_pitch    = pitch;
        msg.gimbal_yaw      = yaw;
        thrust_pub_->publish(msg);

        printf("\r\033[K[teleop/%s] %s thrust: throttle=%.2e pitch=%.3f yaw=%.3f\n",
               target_.c_str(),
               lvlh_mode_ ? "LVLH" : "INERTIAL",
               throttle, pitch, yaw);
        fflush(stdout);
    }

    void publish_zero_thrust() {
        auto msg = orbit_interfaces::msg::ThrustCmd();
        msg.header.stamp = this->get_clock()->now();
        msg.body_id      = "/" + target_;
        msg.throttle     = 0.0;
        msg.gimbal_pitch = 0.0;
        msg.gimbal_yaw   = 0.0;
        thrust_pub_->publish(msg);
    }

    void toggle_pause() {
        paused_ = !paused_;

        // 1. Publish to /sim_pause — gates all physics nodes
        auto bool_msg = std_msgs::msg::Bool();
        bool_msg.data = paused_;
        pause_pub_->publish(bool_msg);

        // 2. Call /set_simulation_state — freezes Isaac Sim renderer
        if (sim_state_cli_->service_is_ready()) {
            auto req = std::make_shared<
                simulation_interfaces::srv::SetSimulationState::Request>();
            req->state.state = paused_ ? STATE_PAUSED : STATE_PLAYING;
            // Fire-and-forget async — don't block the keyboard loop
            sim_state_cli_->async_send_request(req,
                [this](rclcpp::Client<simulation_interfaces::srv::SetSimulationState>
                       ::SharedFuture /*future*/) {
                    // response ignored — visual confirmation comes from Isaac Sim GUI
                });
        } else {
            printf("\r\033[K[teleop] WARNING: /set_simulation_state not ready "
                   "(Isaac Sim not running?)\n");
            fflush(stdout);
        }

        printf("\r\033[K[teleop] *** %s ***\n",
               paused_ ? "SIMULATION PAUSED" : "SIMULATION RESUMED");
        fflush(stdout);
    }

    void print_help() {
        printf("\n=== Orbit Teleop — target: %s ===\n", target_.c_str());
        printf("  i/k  → +/- along-track (LVLH) | +/- Y (INERTIAL)\n");
        printf("  j/l  → -/+ radial     (LVLH) | -/+ X (INERTIAL)\n");
        printf("  u/o  → +/- cross-track (LVLH) | +/- Z (INERTIAL)\n");
        printf("  SPACE → zero thrust\n");
        printf("  F    → toggle frame mode (current: INERTIAL)\n");
        printf("  P    → toggle full pause\n");
        printf("  Q    → quit\n\n");
        fflush(stdout);
    }
};

// ── main ──────────────────────────────────────────────────────────────────────
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TeleopNode>();
    node->run();
    rclcpp::shutdown();
    return 0;
}