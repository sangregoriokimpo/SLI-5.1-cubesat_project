#include <string>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "orbit_interfaces/msg/orbit_state.hpp"
#include "simulation_interfaces/srv/set_entity_state.hpp"

static constexpr uint8_t RESULT_OK = 1;

class IsaacBridgeNode : public rclcpp::Node {
public:
    IsaacBridgeNode() : Node("isaac_bridge_node") {
        declare_parameter("prim_path",      "/World/Sat");
        declare_parameter("state_topic",    "orbit_state");
        declare_parameter("attractor_path", "/World/Earth");
        declare_parameter("scale",          1.0);

        prim_path_   = get_parameter("prim_path").as_string();
        state_topic_ = get_parameter("state_topic").as_string();
        scale_       = get_parameter("scale").as_double();

        cli_ = create_client<simulation_interfaces::srv::SetEntityState>(
            "/set_entity_state");

        sub_ = create_subscription<orbit_interfaces::msg::OrbitState>(
            state_topic_, 10,
            [this](orbit_interfaces::msg::OrbitState::SharedPtr msg) {
                on_orbit_state(msg);
            });

        check_timer_ = create_wall_timer(
            std::chrono::seconds(2),
            [this]() {
                if (cli_->service_is_ready()) {
                    if (!service_ready_) {
                        RCLCPP_INFO(get_logger(),
                            "/set_entity_state ready for %s",
                            prim_path_.c_str());
                        service_ready_ = true;
                        check_timer_->cancel();  
                    }
                } else {
                    RCLCPP_WARN(get_logger(),
                        "/set_entity_state not ready -- is Isaac Sim running?");
                }
            });

        RCLCPP_INFO(get_logger(),
            "isaac_bridge_node: %s -> %s  scale=%.1f",
            state_topic_.c_str(), prim_path_.c_str(), scale_);
    }

private:
    std::string prim_path_;
    std::string state_topic_;
    double      scale_;
    bool        service_ready_ = false;

    rclcpp::Client<simulation_interfaces::srv::SetEntityState>::SharedPtr cli_;
    rclcpp::Subscription<orbit_interfaces::msg::OrbitState>::SharedPtr    sub_;
    rclcpp::TimerBase::SharedPtr check_timer_;

    void on_orbit_state(orbit_interfaces::msg::OrbitState::SharedPtr msg) {
        if (!service_ready_) return;

        auto req = std::make_shared<
            simulation_interfaces::srv::SetEntityState::Request>();

        req->entity                      = prim_path_;
        req->state.header.stamp          = now();
        req->state.header.frame_id       = "world";
        req->state.pose.position.x       = msg->position.x * scale_;
        req->state.pose.position.y       = msg->position.y * scale_;
        req->state.pose.position.z       = msg->position.z * scale_;
        req->state.pose.orientation.w    = msg->attitude.w;
        req->state.pose.orientation.x    = msg->attitude.x;
        req->state.pose.orientation.y    = msg->attitude.y;
        req->state.pose.orientation.z    = msg->attitude.z;
        req->state.twist.linear.x        = msg->velocity.x * scale_;
        req->state.twist.linear.y        = msg->velocity.y * scale_;
        req->state.twist.linear.z        = msg->velocity.z * scale_;

        cli_->async_send_request(req,
            [this](rclcpp::Client<simulation_interfaces::srv::SetEntityState>
                   ::SharedFuture future) {
                try {
                    auto resp = future.get();
                    if (resp->result.result != RESULT_OK) {
                        RCLCPP_WARN(get_logger(),
                            "/set_entity_state failed for %s: code=%d msg=%s",
                            prim_path_.c_str(),
                            resp->result.result,
                            resp->result.error_message.c_str());
                    }
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(get_logger(),
                        "/set_entity_state exception for %s: %s",
                        prim_path_.c_str(), e.what());
                }
            });
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IsaacBridgeNode>());
    rclcpp::shutdown();
    return 0;
}