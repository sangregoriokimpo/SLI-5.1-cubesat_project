/**
 * teleop_node.cpp
 *
 * Keyboard teleop for orbit + attitude control.
 *
 * Usage:
 *   ros2 run sli_gnc teleop_node --ros-args -p target:=sat1
 *
 * Translational (LVLH frame):
 *   i / k        +/- along-track  (fy)
 *   l / j        +/- radial       (fx)
 *   u / o        +/- cross-track  (fz)
 *
 * Attitude (body frame):
 *   UP / DOWN    +/- pitch  (wy)
 *   LEFT / RIGHT +/- yaw    (wz)
 *   , / .        +/- roll   (wx)
 *
 * Other:
 *   SPACE        zero all thrust and torque
 *   P            toggle full pause
 *   Q            quit
 *
 * Parameters:
 *   target      string  satellite namespace  (default: sat1)
 *   thrust_mag  double  translational accel km/s²  (default: 0.01)
 *   torque_mag  double  angular accel rad/s²        (default: 0.05)
 */

#include <csignal>
#include <cstdio>
#include <memory>
#include <string>
#include <termios.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "orbit_interfaces/msg/thrust_cmd.hpp"
#include "simulation_interfaces/srv/set_simulation_state.hpp"
#include "std_msgs/msg/bool.hpp"

static constexpr double THRUST_MAG  = 0.01;
static constexpr double TORQUE_MAG  = 0.05;
static constexpr int    STATE_PLAYING = 1;
static constexpr int    STATE_PAUSED  = 2;

// ── terminal raw-mode ─────────────────────────────────────────────────────────
struct TermRAII {
    termios old{};
    bool    active = false;

    void enable() {
        tcgetattr(STDIN_FILENO, &old);
        termios raw = old;
        raw.c_lflag &= ~(ICANON | ECHO);
        raw.c_cc[VMIN]  = 0;
        raw.c_cc[VTIME] = 1;   // 100ms read timeout
        tcsetattr(STDIN_FILENO, TCSANOW, &raw);
        active = true;
    }
    ~TermRAII() {
        if (active) tcsetattr(STDIN_FILENO, TCSANOW, &old);
    }
};

// ── node ──────────────────────────────────────────────────────────────────────
class TeleopNode : public rclcpp::Node {
public:
    TeleopNode() : Node("teleop_node") {
        this->declare_parameter("target",     "sat1");
        this->declare_parameter("thrust_mag", THRUST_MAG);
        this->declare_parameter("torque_mag", TORQUE_MAG);

        target_     = this->get_parameter("target").as_string();
        thrust_mag_ = this->get_parameter("thrust_mag").as_double();
        torque_mag_ = this->get_parameter("torque_mag").as_double();

        thrust_pub_ = this->create_publisher<orbit_interfaces::msg::ThrustCmd>(
            "/" + target_ + "/cmd_thrust", 10);

        rclcpp::QoS latch_qos(1);
        latch_qos.transient_local();
        pause_pub_ = this->create_publisher<std_msgs::msg::Bool>(
            "/sim_pause", latch_qos);

        sim_state_cli_ = this->create_client<
            simulation_interfaces::srv::SetSimulationState>(
            "/set_simulation_state");

        print_help();
    }

    void run() {
        TermRAII term;
        term.enable();

        rclcpp::Rate rate(30);

        while (rclcpp::ok()) {
            rclcpp::spin_some(shared_from_this());

            // Read up to 3 bytes -- needed to catch arrow key escape sequences
            char buf[3] = {0, 0, 0};
            int n = read(STDIN_FILENO, buf, 3);
            if (n <= 0) { rate.sleep(); continue; }

            // Arrow keys arrive as ESC [ A/B/C/D (3 bytes)
            if (n == 3 && buf[0] == 0x1b && buf[1] == '[') {
                switch (buf[2]) {
                case 'A': publish_cmd(0,0,0,  0, torque_mag_,  0); break; // UP    +pitch
                case 'B': publish_cmd(0,0,0,  0,-torque_mag_,  0); break; // DOWN  -pitch
                case 'C': publish_cmd(0,0,0,  0, 0,-torque_mag_); break;  // RIGHT -yaw
                case 'D': publish_cmd(0,0,0,  0, 0, torque_mag_); break;  // LEFT  +yaw
                default:  break;
                }
                rate.sleep();
                continue;
            }

            char c = buf[0];

            switch (c) {

            // ── pause ─────────────────────────────────────────────────────
            case 'p': case 'P':
                toggle_pause();
                break;

            // ── translational -- LVLH ─────────────────────────────────────
            case 'i': publish_cmd( 0,  thrust_mag_,  0,  0, 0, 0); break;
            case 'k': publish_cmd( 0, -thrust_mag_,  0,  0, 0, 0); break;
            case 'l': publish_cmd( thrust_mag_,  0,  0,  0, 0, 0); break;
            case 'j': publish_cmd(-thrust_mag_,  0,  0,  0, 0, 0); break;
            case 'u': publish_cmd( 0,  0,  thrust_mag_,  0, 0, 0); break;
            case 'o': publish_cmd( 0,  0, -thrust_mag_,  0, 0, 0); break;

            // ── roll -- body frame ────────────────────────────────────────
            case ',': publish_cmd(0,0,0,  torque_mag_, 0, 0); break;
            case '.': publish_cmd(0,0,0, -torque_mag_, 0, 0); break;

            // ── zero all ─────────────────────────────────────────────────
            case ' ':
                publish_cmd(0, 0, 0, 0, 0, 0);
                printf("\r\033[K[teleop/%s] Zeroed\n", target_.c_str());
                fflush(stdout);
                break;

            // ── quit ─────────────────────────────────────────────────────
            case 'q': case 'Q': case 3:
                publish_cmd(0, 0, 0, 0, 0, 0);
                printf("\r\033[K[teleop] Quit\n");
                return;

            default: break;
            }

            rate.sleep();
        }
    }

private:
    std::string target_;
    double      thrust_mag_;
    double      torque_mag_;
    bool        paused_ = false;

    rclcpp::Publisher<orbit_interfaces::msg::ThrustCmd>::SharedPtr thrust_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr              pause_pub_;
    rclcpp::Client<simulation_interfaces::srv::SetSimulationState>::SharedPtr
        sim_state_cli_;

    void publish_cmd(double fx, double fy, double fz,
                     double wx, double wy, double wz) {
        auto msg = orbit_interfaces::msg::ThrustCmd();
        msg.header.stamp    = this->get_clock()->now();
        msg.header.frame_id = "lvlh";
        msg.body_id         = "/" + target_;
        msg.fx = fx;  msg.fy = fy;  msg.fz = fz;
        msg.wx = wx;  msg.wy = wy;  msg.wz = wz;
        thrust_pub_->publish(msg);

        if (fx || fy || fz)
            printf("\r\033[K[teleop/%s] force:  fx=%.2e fy=%.2e fz=%.2e km/s2\n",
                   target_.c_str(), fx, fy, fz);
        if (wx || wy || wz)
            printf("\r\033[K[teleop/%s] torque: wx=%.2e wy=%.2e wz=%.2e rad/s2\n",
                   target_.c_str(), wx, wy, wz);
        fflush(stdout);
    }

    void toggle_pause() {
        paused_ = !paused_;

        auto bool_msg = std_msgs::msg::Bool();
        bool_msg.data = paused_;
        pause_pub_->publish(bool_msg);

        if (sim_state_cli_->service_is_ready()) {
            auto req = std::make_shared<
                simulation_interfaces::srv::SetSimulationState::Request>();
            req->state.state = paused_ ? STATE_PAUSED : STATE_PLAYING;
            sim_state_cli_->async_send_request(req);
        } else {
            printf("\r\033[K[teleop] WARNING: /set_simulation_state not ready\n");
            fflush(stdout);
        }

        printf("\r\033[K[teleop] *** %s ***\n",
               paused_ ? "PAUSED" : "RESUMED");
        fflush(stdout);
    }

    void print_help() {
        printf("\n=== Orbit Teleop === target: %s ===\n", target_.c_str());
        printf("  Translational (LVLH):\n");
        printf("    i / k        +/- along-track  (fy)\n");
        printf("    l / j        +/- radial        (fx)\n");
        printf("    u / o        +/- cross-track   (fz)\n");
        printf("  Attitude (body frame):\n");
        printf("    UP / DOWN    +/- pitch  (wy)\n");
        printf("    LEFT / RIGHT +/- yaw    (wz)\n");
        printf("    , / .        +/- roll   (wx)\n");
        printf("  SPACE          zero all\n");
        printf("  P              pause / resume\n");
        printf("  Q              quit\n\n");
        fflush(stdout);
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TeleopNode>();
    node->run();
    rclcpp::shutdown();
    return 0;
}