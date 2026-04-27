#include "rclcpp/rclcpp.hpp"
#include "orbit_interfaces/msg/orbit_state.hpp"
#include "orbit_interfaces/msg/thrust_cmd.hpp"
#include "std_msgs/msg/bool.hpp"
#include <Eigen/Dense>
#include <cmath>
#include <string>
#include <chrono>

using V3 = Eigen::Vector3d;
using Q4 = Eigen::Quaterniond;

/*

CW INTEGRATION

*/

static std::pair<V3,V3> cw_rk4_step(double n, V3 rho, V3 rho_dot,double dt, V3 a){
    auto f = [&](V3 r, V3 v) -> std::pair<V3,V3>{
        V3 acc;
        acc[0] =  2*n*v[1] + 3*n*n*r[0] + a[0];//RADIAL
        acc[1] = -2*n*v[0] + a[1];//ALONG TRACK
        acc[2] = -n*n*r[2] + a[2];//CROSS TRACK
        return std::make_pair(v,acc);
    };

    auto [k1r,k1v] = f(rho,rho_dot);
    V3 r2 = rho + 0.5*dt*k1r;
    V3 v2 = rho_dot + 0.5*dt*k1v;
    auto [k2r,k2v] = f(r2,v2);
    V3 r3 = rho + 0.5*dt*k2r;
    V3 v3 = rho_dot + 0.5*dt*k2v;
    auto[k3r,k3v] = f(r3,v3);
    V3 r4 = rho + dt*k3r;
    V3 v4 = rho_dot + dt*k3v;
    auto[k4r,k4v] = f(r4,v4);
    return std::make_pair(rho + (dt/6.0)*(k1r+2.0*k2r+2.0*k3r + k4r),rho_dot + (dt/6.0)*(k1v + 2.0*k2v + 2.0*k3v + k4v));
}

static V3 lvlh_to_inertial(V3 f, V3 r, V3 v){
    V3 x = r.normalized();
    V3 z = r.cross(v).normalized();
    V3 y = z.cross(x);
    return x*f[0]+y*f[1]+z*f[2];
}

static Q4 q_integrate(Q4 q, V3 omega, double dt){
    Q4 om;
    om.w() = 0.0;
    om.x() = omega[0];
    om.y() = omega[1];
    om.z() = omega[2];

    Q4 qd;
    qd.w() = q.w()*om.w()- q.x()*om.x() - q.y()*om.y() - q.z()*om.z();
    qd.x() = q.w()*om.x()+ q.x()*om.w() + q.y()*om.z() - q.z()*om.y();
    qd.y() = q.w()*om.y() - q.x()*om.z() + q.y()*om.w() +q.z()*om.x();
    qd.z() = q.w()*om.z() +q.x()*om.y() -q.y()*om.x() +q.z()*om.w();

    Q4 result;
    result.w() = q.w() + 0.5*qd.w()*dt;
    result.x() = q.x() + 0.5*qd.x()*dt;
    result.y() = q.y() + 0.5*qd.y()*dt;
    result.z() = q.z() + 0.5*qd.z()*dt;

    result.normalize();
    return result;


}

class CWNode : public rclcpp::Node{
    public: 
        CWNode() : Node("cw_node"){

            //DECLARE PARAMETERS
            declare_parameter("mu",398600.4418);
            declare_parameter("dt_sim",0.05);
            declare_parameter("publish_rate",30.0);
            declare_parameter("prim_path","/World/Sat3");
            declare_parameter("body_topic","/sat1/orbit_state");
            declare_parameter("dr_x",1.0);
            declare_parameter("dr_y",0.0);
            declare_parameter("dr_z",0.5);
            declare_parameter("dv_x",0.0);
            declare_parameter("dv_y",0.001);
            declare_parameter("dv_z",0.0);

            mu_ = get_parameter("mu").as_double();
            dt_sim_ = get_parameter("dt_sim").as_double();
            prim_path_ = get_parameter("prim_path").as_string();
            body_topic_ = get_parameter("body_topic").as_string();
            auto publish_rate = get_parameter("publish_rate").as_double();

            rho_ = {get_parameter("dr_x").as_double(),get_parameter("dr_y").as_double(),get_parameter("dr_z").as_double()};
            rho_dot_ = {get_parameter("dv_x").as_double(),get_parameter("dv_y").as_double(),get_parameter("dv_z").as_double()};
            pub_ = create_publisher<orbit_interfaces::msg::OrbitState>("orbit_state",10);

            sub_body_ = create_subscription<orbit_interfaces::msg::OrbitState>(
                body_topic_,10,[this](orbit_interfaces::msg::OrbitState::SharedPtr msg){
                    r_body_ref_={msg->position.x, msg->position.y, msg->position.z};
                    v_body_ref_ = {msg->velocity.x,msg->velocity.y,msg->velocity.z};
                    body_received_ = true;
                }
            );

            sub_thrust_ = create_subscription<orbit_interfaces::msg::ThrustCmd>("cmd_thrust",10,
            [this](orbit_interfaces::msg::ThrustCmd::SharedPtr msg){
                f_lvlh_={msg->fx, msg->fy, msg-> fz};
                omega_ = {msg->wx, msg->wy,msg->wz};
            });

            rclcpp::QoS latch(1);
            latch.transient_local().reliable();
            sub_pause_ = create_subscription<std_msgs::msg::Bool>(
                "/sim_pause",latch,
                [this](std_msgs::msg::Bool::SharedPtr msg){
                    if(msg->data != paused_){
                        paused_ = msg->data;
                        RCLCPP_INFO(get_logger(),"[%s] %s",prim_path_.c_str(),paused_ ? "PAUSED" : "RESUMED");
                    }
                });
            
            auto dt_ms = std::chrono::duration<double>(dt_sim_);
            auto pub_ms = std::chrono::duration<double>(1.0 / publish_rate);
            integrate_timer_ = create_wall_timer(dt_ms,  [this]() { integrate_step(); });
            publish_timer_   = create_wall_timer(pub_ms, [this]() { publish_state(); });

            RCLCPP_INFO(get_logger(), "cw_node: %s  body=%s  dt=%.5fs  pub=%.1fHz",prim_path_.c_str(), body_topic_.c_str(), dt_sim_, publish_rate);


        }
    private:
        double mu_;
        double dt_sim_;
        std::string prim_path_;
        std::string body_topic_;
        bool paused_ = false;
        bool body_received_ = false;

        V3 rho_ = V3::Zero();
        V3 rho_dot_ = V3::Zero();
        V3 r_body_ref_ = V3::Zero();
        V3 v_body_ref_ = V3::Zero();
        Q4 q_ = Q4::Identity();
        V3 f_lvlh_ = V3::Zero();
        V3 omega_ = V3::Zero();

        rclcpp::Publisher<orbit_interfaces::msg::OrbitState>::SharedPtr pub_;
        rclcpp::Subscription<orbit_interfaces::msg::OrbitState>::SharedPtr sub_body_;
        rclcpp::Subscription<orbit_interfaces::msg::ThrustCmd>::SharedPtr sub_thrust_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_pause_;
        rclcpp::TimerBase::SharedPtr integrate_timer_;
        rclcpp::TimerBase::SharedPtr publish_timer_;

        void integrate_step(){
            if(paused_ || !body_received_){
                return;
            }
            double r_mag = r_body_ref_.norm();
            if(r_mag < 1e-6){
                return;
            }
            double n = std::sqrt(mu_ / (r_mag*r_mag*r_mag));
            auto [rn,vn] = cw_rk4_step(n,rho_,rho_dot_,dt_sim_,f_lvlh_);
            rho_ = rn;
            rho_dot_ = vn;

            if(omega_.norm() > 1e-12){
                q_ = q_integrate(q_,omega_,dt_sim_);
            }
        }

        void publish_state(){
            if(paused_ || !body_received_){
                return;
            }
            V3 dr_inertial = lvlh_to_inertial(rho_,r_body_ref_,v_body_ref_);
            V3 dv_inertial = lvlh_to_inertial(rho_dot_,r_body_ref_,v_body_ref_);

            V3 r_world = r_body_ref_ + dr_inertial;
            V3 v_world = v_body_ref_ + dv_inertial;
            auto msg = orbit_interfaces::msg::OrbitState();
            msg.header.stamp = now();
            msg.header.frame_id = "world";
            msg.body_id = prim_path_;
            msg.position.x = r_world[0];
            msg.position.y= r_world[1];
            msg.position.z = r_world[2];
            msg.velocity.x = v_world[0];
            msg.velocity.y = v_world[1];
            msg.velocity.z= v_world[2];
            msg.attitude.w= q_.w();
            msg.attitude.x= q_.x();
            msg.attitude.y= q_.y();
            msg.attitude.z= q_.z();
            pub_->publish(msg);
        }
    


};

int main(int argc, char*argv[]){
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<CWNode>());
    rclcpp::shutdown();
    return 0;
}