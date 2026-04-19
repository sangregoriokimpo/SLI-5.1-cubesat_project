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

static constexpr double THRUST_MAG  = 0.01;   // km/s²
static constexpr int    STATE_PLAYING = 1;
static constexpr int    STATE_PAUSED  = 2;

struct TermRAII {
    termios old{};
    bool    active = false;

    void enable() {
        tcgetattr(STDIN_FILENO, &old);
        termios raw = old;
        raw.c_lflag &= ~(ICANON | ECHO);
        raw.c_cc[VMIN]  = 0;
        raw.c_cc[VTIME] = 1;   // 100 ms read timeout
        tcsetattr(STDIN_FILENO, TCSANOW, &raw);
        active = true;
    }
    ~TermRAII() {
        if (active) tcsetattr(STDIN_FILENO, TCSANOW, &old);
    }
};

class TeleopNode : public rclcpp::Node {
public:
    TeleopNode() : Node("teleop_node") {
        this->declare_parameter("target",     "sat1");
        this->declare_parameter("thrust_mag", THRUST_MAG);

        target_     = this->get_parameter("target").as_string();
        thrust_mag_ = this->get_parameter("thrust_mag").as_double();

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
        char c = 0;

        while (rclcpp::ok()) {
            rclcpp::spin_some(shared_from_this());

            int n = read(STDIN_FILENO, &c, 1);
            if (n <= 0) { rate.sleep(); continue; }

            switch (c) {

            case 'p': case 'P':
                toggle_pause();
                break;

            // LVLH: fx=radial, fy=along-track, fz=cross-track
            case 'i': publish_thrust( 0.0,  1.0,  0.0); break;  // +along-track
            case 'k': publish_thrust( 0.0, -1.0,  0.0); break;  // -along-track
            case 'l': publish_thrust( 1.0,  0.0,  0.0); break;  // +radial
            case 'j': publish_thrust(-1.0,  0.0,  0.0); break;  // -radial
            case 'u': publish_thrust( 0.0,  0.0,  1.0); break;  // +cross-track
            case 'o': publish_thrust( 0.0,  0.0, -1.0); break;  // -cross-track

            case ' ':
                publish_thrust(0.0, 0.0, 0.0);
                printf("\r\033[K[teleop/%s] Thrust zeroed\n",
                       target_.c_str());
                fflush(stdout);
                break;

            case 'q': case 'Q': case 3:
                publish_thrust(0.0, 0.0, 0.0);
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
    bool        paused_ = false;

    rclcpp::Publisher<orbit_interfaces::msg::ThrustCmd>::SharedPtr thrust_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr              pause_pub_;
    rclcpp::Client<simulation_interfaces::srv::SetSimulationState>::SharedPtr
        sim_state_cli_;

    void publish_thrust(double fx_unit, double fy_unit, double fz_unit) {
        auto msg = orbit_interfaces::msg::ThrustCmd();
        msg.header.stamp    = this->get_clock()->now();
        msg.header.frame_id = "lvlh";
        msg.body_id         = "/" + target_;
        msg.fx              = fx_unit * thrust_mag_;
        msg.fy              = fy_unit * thrust_mag_;
        msg.fz              = fz_unit * thrust_mag_;
        thrust_pub_->publish(msg);

        if (fx_unit != 0.0 || fy_unit != 0.0 || fz_unit != 0.0) {
            printf("\r\033[K[teleop/%s] LVLH thrust: "
                   "fx=%.2e fy=%.2e fz=%.2e km/s²\n",
                   target_.c_str(), msg.fx, msg.fy, msg.fz);
            fflush(stdout);
        }
    }

    void toggle_pause() {
        paused_ = !paused_;

        // 1. Gate physics nodes
        auto bool_msg = std_msgs::msg::Bool();
        bool_msg.data = paused_;
        pause_pub_->publish(bool_msg);

        // 2. Isaac Sim renderer — fire and forget
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
        printf("\n=== Orbit Teleop — target: %s ===\n", target_.c_str());
        printf("  i / k  → +/- along-track  (fy)\n");
        printf("  l / j  → +/- radial        (fx)\n");
        printf("  u / o  → +/- cross-track   (fz)\n");
        printf("  SPACE  → zero thrust\n");
        printf("  P      → toggle pause\n");
        printf("  Q      → quit\n\n");
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