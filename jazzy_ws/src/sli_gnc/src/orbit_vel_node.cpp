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


