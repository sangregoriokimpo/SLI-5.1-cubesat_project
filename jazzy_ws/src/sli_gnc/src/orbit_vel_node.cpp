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