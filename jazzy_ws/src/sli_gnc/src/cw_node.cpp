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