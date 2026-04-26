#include "rclcpp/rclcpp.hpp"
#include "orbit_interfaces/msg/orbit_state.hpp"
#include "orbit_interfaces/msg/thrust_cmd.hpp"
#include "std_msgs/msg/bool.hpp"
#include <cmath>
#include <array>
#include <string>
#include <chrono>

using V3 = std::array<double, 3>;
using Q4 = std::array<double,4>;


/*

VECTOR MATH CODE

*/

static V3 v_add(V3 a, V3 b){
    return {a[0]+b[0], a[1]+b[1], a[2]+b[2]};
}

static V3 v_mul(double s, V3 a){
    return {s*a[0],s*a[1],s*a[2]};
}

static V3 v_cross(V3 a, V3 b){
    return{
        a[1]*b[2] - a[2]*b[1],
        a[2]*b[0] - a[0]*b[2],
        a[0]*b[1] - a[1]*b[0]
    };
}

static double v_norm(V3 a){
    return std::sqrt(a[0]*a[0] + a[1]*a[1] + a[2]*a[2]);
}

static V3 v_normalize(V3 a){
    double n = v_norm(a);
    if(n < 1e-12){
        return {0,0,0};
    }
    return {a[0]/n, a[1]/n, a[2]/n};
}

static V3 lvlh_to_inertial(V3 f, V3 r, V3 v){
    V3 x = v_normalize(r);
    V3 z = v_normalize(v_cross(r,v));
    V3 y = v_cross(z,x);
    return{
        x[0]*f[0] + y[0]*f[1] + z[0]*f[2],
        x[1]*f[0] + y[1]*f[1] + z[1]*f[2],
        x[2]*f[0] + y[2]*f[1] + z[2]*f[2]
    };
}

/*

ORBIT MECHANICS CODE UTILIZING RK4 INTEGRATION

*/

static std::pair<V3,V3> circular_ic(double mu, double radius, const std::string& plane){
    double spd = std::sqrt(mu/radius);
    if(plane == "xy"){
        return {{radius,0,0},{0,spd,0}};
    }
    if(plane == "xz"){
        return {{radius,0,0},{0,0,spd}};
    }
    if(plane == "yz"){
        return {{0,radius,0},{0,0,spd}};
    }
    throw std::invalid_argument("Unknown plane: " + plane);
}

static void rot_z(double angle, V3& v){
    double c = std::cos(angle), s = std::sin(angle);
    double x = c*v[0] - s*v[1];
    double y = s*v[0] + c*v[1];
    v[0] = x;
    v[1] = y;
}

static void rot_x(double angle, V3& v){
    double c = std::cos(angle), s = std::sin(angle);
    double y = c*v[1] - s*v[2];
    double z = s*v[1] + c*v[2];
    v[1] = y;
    v[2] = z;
}

static std::pair<V3,V3> elements_ic(double mu, double a, double e, double inc_deg, double raan_deg, double argp_deg, double nu_deg){
    double inc = inc_deg * M_PI / 180.0;
    double raan = raan_deg * M_PI / 180.0;
    double argp = argp_deg * M_PI / 180.0;
    double nu = nu_deg * M_PI / 180.0;

    double p =  a * (1.0 - e*e);
    double rm  = p / (1.0 + e * std::cos(nu));
    V3 r_pqw = {rm*std::cos(nu), rm*std::sin(nu),0.0};

    double fac = std::sqrt(mu / p);
    V3 v_pqw = {-fac*std::sin(nu), fac*(e + std::cos(nu)), 0.0};

    rot_z(argp, r_pqw);
    rot_x(inc, r_pqw); 
    rot_z(raan,r_pqw);
    
    rot_z(argp, v_pqw);
    rot_x(inc, v_pqw);
    rot_z(raan, v_pqw);

    return {r_pqw, v_pqw};

}

static std::pair<V3,V3>rk4_step(double mu, V3 r, V3 v, double dt, V3 a_ext){
    auto gravity = [&](V3 rr) -> V3{
        double d = v_norm(rr);
        if(d < 1e-12){
            return {0,0,0};
        }
        double s = -mu / (d*d*d);
        return {s*rr[0],s*rr[1],s*rr[2]};
    };
    auto accel = [&](V3 rr)-> V3{
        V3 g = gravity(rr);
        return {g[0]+a_ext[0],g[1]+a_ext[1],g[2]+a_ext[2]};
    };
    V3 k1r = v, k1v = accel(r);
    V3 r2 = v_add(r, v_mul(0.5*dt, k1r));
    V3 v2 = v_add(v, v_mul(0.5*dt, k1v));
    V3 k2r = v2, k2v = accel(r2);
    V3 r3 = v_add(r,v_mul(0.5*dt, k2r));
    V3 v3 = v_add(v,v_mul(0.5*dt, k2v));
    V3 k3r = v3, k3v = accel(r3); 
    V3 r4 = v_add(r,v_mul(dt,k3r));
    V3 v4 = v_add(v, v_mul(dt,k3v));
    V3 k4r = v4, k4v = accel(r4);

    auto comb = [&](V3 k1, V3 k2, V3 k3, V3 k4) -> V3{
        return v_mul(dt/6.0, v_add(v_add(k1,v_mul(2.0,k2)), v_add(v_mul(2.0,k3),k4)));
    };
    return {
        v_add(r,comb(k1r, k2r, k3r, k4r)), v_add(v,comb(k1v,k2v,k3v,k4v))
    };

}

/*

QUARTERNION FUNCTIONS

*/


static Q4 q_mul(Q4 p, Q4 q){
    return{
        p[0]*q[0] - p[1]*q[1] - p[2]*q[2] - p[3]*q[3],
        p[0]*q[1] + p[1]*q[0] + p[2]*q[3] - p[3]*q[2],
        p[0]*q[2] - p[1]*q[3] + p[2]*q[0] + p[3]*q[1],
        p[0]*q[3] + p[1]*q[2] - p[2]*q[1] + p[3]*q[0]
    };
}

static Q4 q_normalize(Q4 q){
    double n = std::sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    if(n < 1e-12){
        return {1,0,0,0};
    }
    return {q[0]/n , q[1]/n, q[2]/n , q[3]/n};
}

static Q4 q_integrate(Q4 q, V3 omega, double dt){
    Q4 om = {0.0, omega[0], omega[1], omega[2]};
    Q4 qd = q_mul(q, om);
    return q_normalize({
            q[0] + 0.5*qd[0]*dt,
            q[1] + 0.5*qd[1]*dt,
            q[2] + 0.5*qd[2]*dt,
            q[3] + 0.5*qd[3]*dt
        });  
}

class OrbitVelNode : public rclcpp::Node{
    public: 
    OrbitVelNode(): Node("orbit_vel_node"){
        declare_parameter("mu",398600.4418);
        declare_parameter("dt_sim",0.05);
        declare_parameter("publish_rate",30.0);
        declare_parameter("prim_path","/world/Sat");
        declare_parameter("orbit_type","circular");
        declare_parameter("radius",6778000 );
        declare_parameter("plane","xy");
        declare_parameter("a",6778000 );
        declare_parameter("e",0.0);
        declare_parameter("inc",0.0);
        declare_parameter("raan",0.0);
        declare_parameter("argp",0.0);
        declare_parameter("nu",0.0);
        
        mu_ = get_parameter("mu").as_double();
        dt_sim_ = get_parameter("dt_sim").as_double();
        prim_path_ = get_parameter("prim_path").as_string();
        auto orbit_type = get_parameter("orbit_type").as_string();
        auto publish_rate = get_parameter("publish_rate").as_double();

        if(orbit_type =="circular"){
            auto [r,v] = circular_ic(mu_, get_parameter("radius").as_double(), get_parameter("plane").as_string());
            r_ = r;
            v_ = v;
        }else if (orbit_type == "elements"){
            auto [r,v] = elements_ic(mu_,get_parameter("a").as_double(),get_parameter("e").as_double(),get_parameter("inc").as_double(),get_parameter("raan").as_double(),get_parameter("argp").as_double(),get_parameter("nu").as_double());
            r_ = r;
            v_ = v;
        }else{
            throw std::invalid_argument("Unknown orbit_type: " + orbit_type);
        }

        pub_ = create_publisher<orbit_interfaces::msg::OrbitState>("orbit_state",10);
        sub_thrust_ = create_subscription<orbit_interfaces::msg::ThrustCmd>("cmd_thrust",10, [this](orbit_interfaces::msg::ThrustCmd::SharedPtr msg){
            f_lvlh_ = {msg->fx, msg->fy, msg->fz};
            omega_ = {msg->wx, msg-> wy, msg->wz};
        }
        );
        rclcpp::QoS latch(1);
        latch.transient_local().reliable();
        sub_pause_ = create_subscription<std_msgs::msg::Bool>("/sim_pause",latch,[this](std_msgs::msg::Bool::SharedPtr msg){
            if(msg->data != paused_){
                paused_ = msg->data;
                RCLCPP_INFO(get_logger(),"[%s] %s",prim_path_.c_str(), paused_ ? "PAUSED" : "RESUMED");
            }
        });
        auto dt_ms = std::chrono::duration<double>(dt_sim_);
        auto pub_ms = std::chrono::duration<double>(1.0 / publish_rate);

        integrate_timer_ = create_wall_timer(dt_ms, [this](){integrate_step();});
        publish_timer_ = create_wall_timer(pub_ms,[this](){publish_state();});

        RCLCPP_INFO(get_logger(), "orbit_vel_node: %s dt=%.5fs pub=%.1fHz",prim_path_.c_str(), dt_sim_, publish_rate);

    }
    private:
    double mu_;
    double dt_sim_;
    std::string prim_path_;
    bool paused_ = false;
    V3 r_ ={0,0,0};
    V3 v_={0,0,0};
    Q4 q_ ={1,0,0,0};
    V3 f_lvlh_ ={0,0,0};
    V3 omega_ = {0,0,0};

    rclcpp::Publisher<orbit_interfaces::msg::OrbitState>::SharedPtr pub_;
    rclcpp::Subscription<orbit_interfaces::msg::ThrustCmd>::SharedPtr sub_thrust_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_pause_;
    rclcpp::TimerBase::SharedPtr integrate_timer_;
    rclcpp::TimerBase::SharedPtr publish_timer_;



    void integrate_step(){
        if(paused_){
            return;
        }
        V3 a_inertial = {0,0,0};
        if (f_lvlh_[0] || f_lvlh_[1] || f_lvlh_[2]){
            a_inertial = lvlh_to_inertial(f_lvlh_,r_,v_);
        }

        auto [rn,vn] = rk4_step(mu_, r_, dt_sim_, a_inertial);
        r_ = rn;
        v_ = vn;
        if(omega_[0] || omega_[1] || omega_[2]){
            q_ = q_integrate(q_,omega_,dt_sim_);
        }
    }

    void publish_state(){
        if(paused_){
            return;
        }
        auto msg = orbit_interfaces::msg::OrbitState():
        msg.header.stamp = now();
        msg.header.frame_id ="world";
        msg.body_id = prim_path_;
        msg.position.x = r_[0];
        msg.position.y = r_[1];
        msg.position.z= r_[2];
        msg.velocity.x = v_[0];
        msg.velocity.y = v_[1];
        msg.velocity.z = v_[2];
        msg.attitude.w = q_[0];
        msg.attitude.x = q_[1];
        msg.attitude.y = q_[2];
        msg.attitude.z= q_[3];
        pub_->publish(msg);
    }
};

