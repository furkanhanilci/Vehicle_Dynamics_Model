//
// Created by furkanhanilci on 29.04.2024.
//

#include <boost/python/def.hpp>
#include <boost/python/extract.hpp>
#include <boost/python/module.hpp>
#include <boost/python/numpy.hpp>
#include <cmath>
#include <ctime>
#include <iostream>
#include <iostream>
#include <sstream>
#include <stdlib.h>
#include <string>
#include <vector>

#define EPS 0.01    // epsilon
#define STATE_DIM 16    // state dimension
#define HELPER_T_DIM 10 // helper_t dimension
#define CTRL_DIM 5  // control dimension
#define mathpi 3.141592653589793238462643383279502884197169399375105820974944592307816406286    // pi

using namespace std;
using namespace boost::python;

using std::sin;
using std::cos;
using std::tan;
using std::atan;
using std::asin;
using std::acos;

/**
*   \namespace params
*   \brief     Every constant parameter of the vehicle
*/
namespace params
{
    double gravity = 9.81;  // gravity
    double rho_air = 1.2;   // air density

    double lf = 1.304;  // distance from the center of mass to the front axle
    double lr = 1.788;  // distance from the center of mass to the rear axle

    double lw = 1.608;  // distance between the two front wheels

    double r_eff = 0.34;    // effective radius of the wheels
    double h_CoG = 0.715;   // height of the center of gravity
    double r0 = 0.35;   // radius of the wheels

    double mass_susp = 1738;    // mass of the suspension
    double mass = 1878; // mass of the vehicle

    double Iz = 4045.3; // moment of inertia around the z-axis
    double Ir = 1.3;    // moment of inertia of the wheels
    double Ix = 692.7;  // moment of inertia around the x-axis
    double Iy = 3961.4; // moment of inertia around the y-axis

    double k_susp_f = 42000;    // suspension stiffness
    double d_susp_f = 4000; // suspension damping

    double k_susp_r = (lf / lr) *(lf / lr) * k_susp_f;  // rear suspension stiffness
    double d_susp_r = (lf / lr) *(lf / lr) * d_susp_f;  // rear suspension damping

    double front_surf = 2.675;  // front surface of the vehicle
    double drag_coef = 0.35;    // drag coefficient

    double mass_wheel = (mass - mass_susp) / 4;   // mass of each wheel

    double f0 = mass_susp*gravity / (2 * (lf + lr));    // force at each suspension
    double Fsusp0[4] = { f0*lr, f0*lr, f0*lf, f0*lf };  // force at each suspension
    double fz0[4] = { Fsusp0[0] + mass_wheel*gravity,   // force at each wheel
                      Fsusp0[1] + mass_wheel*gravity,
                      Fsusp0[2] + mass_wheel*gravity,
                      Fsusp0[3] + mass_wheel*gravity };

    double aero_coef = 0.5 * rho_air*front_surf*drag_coef;  // aerodynamic coefficient

    double V_max = 50;  // maximum velocity
    double OmegaMax = V_max / r_eff;    // maximum angular velocity
    double rollMax = 10 * mathpi / 180; // maximum roll angle
    double pitchMax = 10 * mathpi / 180;    // maximum pitch angle

    double C_mot_max = 2 * 750; // maximum motor couple
    double C_fr_min = 4 * (-1250);  // minimum friction couple
    double A_max = 2.5; // maximum acceleration
    double D_max = -9.8;    // maximum deceleration

    double ColDirection = 18.24;    // collision direction
    double delta_max = 500 * (mathpi / (180 * ColDirection));   // maximum steering angle
    double delta_min = -500 * (mathpi / (180 * ColDirection));  // minimum steering angle
    double delta_braq = 450 * (mathpi / (180 * ColDirection));  // steering angle for braking

    /**
    *   \namespace params::tires
    *   \brief     Every constant parameter of the vehicle tires
    */
    namespace tires
    {
        // pTire
        double p_Cx1 = 1.65;    // Shape factor C for longitudinal force; influences the stiffness of the force curve
        double p_Dx1 = 1;       // Peak value D for longitudinal force; influences the maximum value of the longitudinal force
        double p_Dx2 = 0;    // Variation of peak value D with load; influences the variation of the maximum value of the longitudinal force with the load
        double p_Ex1 = -0.5;    // Shape factor E for longitudinal force; influences the curvature of the force curve near and beyond the peak force
        double p_Ex2 = 0;   // Variation of shape factor E with load; influences the variation of the curvature of the force curve with the load
        double p_Ex3 = 0;   // Variation of shape factor E with slip; influences the variation of the curvature of the force curve with slip
        double p_Ex4 = 0;   // Variation of shape factor E with camber; influences the variation of the curvature of the force curve with camber
        double p_Kx1 = 12;  // Longitudinal slip stiffness K; define how the force builds up with increasing slip
        double p_Kx2 = 10;  // Variation of longitudinal slip stiffness K with load; influences the variation of the force build-up with slip with the load
        double p_Kx3 = -0.6;    // Influence of camber on the longitudinal slip stiffness K; influences the variation of the force build-up with slip with camber
        double p_Hx1 = 0;   // Horizontal shift of the longitudinal force peak value point B; influences the horizontal position of the peak force
        double p_Hx2 = 0;   // Horizontal shift of the longitudinal force peak value point B with load; influences the variation of the horizontal position of the peak force with the load
        double p_Vx1 = 0;   // Vertical shift of the longitudinal force peak value point B; influences the vertical position of the peak force
        double p_Vx2 = 0;   // Vertical shift of the longitudinal force peak value point B with load; influences the variation of the vertical position of the peak force with the load

        double p_Cy1 = 1.3; // Shape factor C for lateral force; similar role as p_Cx1, but for lateral dynamics
        double p_Dy1 = 1;   // Peak value D for lateral force; similar role as p_Dx1, but for lateral dynamics
        double p_Dy2 = 0;   // Variation of peak value D with load; similar role as p_Dx2, but for lateral dynamics
        double p_Dy3 = 0;   // Variation of peak value D with camber; similar role as p_Dx2, but for lateral dynamics
        double p_Ey1 = -1;  // Shape factor E for lateral force; influences the curvature of the force curve near and beyond the peak force
        double p_Ey2 = 0;   // Variation of shape factor E with load; influences the variation of the curvature of the force curve with the load
        double p_Ey3 = 0;   // Variation of shape factor E with camber; influences the variation of the curvature of the force curve with camber
        double p_Ey4 = 0;   // Variation of shape factor E with inclination; influences the variation of the curvature of the force curve with inclination
        double p_Ky1 = 10;  // Slip stiffness K for lateral force; similar role as p_Kx1, but for lateral dynamics
        double p_Ky2 = 1.5; // Variation of slip stiffness K with load; similar role as p_Kx2, but for lateral dynamics
        double p_Ky3 = 0;   // Variation of slip stiffness K with camber; similar role as p_Kx3, but for lateral dynamics
        double p_Hy1 = 0;   // Horizontal shift of the lateral force peak value point B; similar role as p_Hx1, but for lateral dynamics
        double p_Hy2 = 0;   // Horizontal shift of the lateral force peak value point B with load; similar role as p_Hx2, but for lateral dynamics
        double p_Hy3 = 0.25;    // Horizontal shift of the lateral force peak value point B with inclination; similar role as p_Hx2, but for lateral dynamics
        double p_Vy1 = 0;   // Vertical shift of the lateral force peak value point B; similar role as p_Vx1, but for lateral dynamics
        double p_Vy2 = 0;   // Vertical shift of the lateral force peak value point B with load; similar role as p_Vx2, but for lateral dynamics
        double p_Vy3 = 0.15;    // Vertical shift of the lateral force peak value point B with inclination; similar role as p_Vx2, but for lateral dynamics
        double p_Vy4 = 0;   // Influence of inclination on the lateral force peak value point B; influences the variation of the vertical position of the peak force with inclination

        double q_Bz1 = 6;   // Shape factor B for aligning moment; influences the stiffness and shape of thr torque curve as a function of the slip angle, load and other conditions
        double q_Bz2 = -4;  // Variation of shape factor B with load; influences the variation of the stiffness and shape of the torque curve with the load
        double q_Bz3 = 0.6; // Variation of shape factor B with camber; influences the variation of the stiffness and shape of the torque curve with camber
        double q_Bz4 = 0;   // Variation of shape factor B with inclination; influences the variation of the stiffness and shape of the torque curve with inclination
        double q_Bz5 = 0;   // Variation of shape factor B with inclination squared; influences the variation of the stiffness and shape of the torque curve with inclination squared
        double q_Bz10 = 0.7;    // Shape factor B for aligning moment; influences the stiffness and shape of the torque curve as a function of the slip angle, load and other conditions
        double q_Cz1 = 1.05;    // Shape factor C for aligning moment; influences the stiffness and shape of the torque curve as a function of the slip angle, load and other conditions
        double q_Dz1 = 0.12;    // Peak value D for aligning moment; influences the maximum value of the aligning moment
        double q_Dz2 = -0.03;   // Variation of peak value D with load; influences the variation of the maximum value of the aligning moment with the load
        double q_Dz3 = 0;   // Variation of peak value D with camber; influences the variation of the maximum value of the aligning moment with camber
        double q_Dz4 = -1;  // Shape factor D for aligning moment; influences the curvature of the aligning moment curve near and beyond the peak aligning moment
        double q_Dz6 = 0;   // Variation of shape factor D with inclination; influences the variation of the curvature of the aligning moment curve with inclination
        double q_Dz7 = 0;   // Variation of shape factor D with inclination squared; influences the variation of the curvature of the aligning moment curve with inclination squared
        double q_Dz8 = 0.6; // Variation of shape factor D with camber; influences the variation of the curvature of the aligning moment curve with camber
        double q_Dz9 = 0.2; // Variation of shape factor D with camber squared; influences the variation of the curvature of the aligning moment curve with camber squared
        double q_Ez1 = -10; // Shape factor E for aligning moment; influences the curvature of the aligning moment curve near and beyond the peak aligning moment
        double q_Ez2 = 0;   // Variation of shape factor E with load; influences the variation of the curvature of the aligning moment curve with the load
        double q_Ez3 = 0;   // Variation of shape factor E with camber; influences the variation of the curvature of the aligning moment curve with camber
        double q_Ez4 = 0;   // Variation of shape factor E with inclination; influences the variation of the curvature of the aligning moment curve with inclination
        double q_Ez5 = 0;   // Variation of shape factor E with inclination squared; influences the variation of the curvature of the aligning moment curve with inclination squared
        double q_Hz1 = 0;   // Horizontal shift of the aligning moment peak value point B; influences the horizontal position of the peak aligning moment
        double q_Hz2 = 0;   // Horizontal shift of the aligning moment peak value point B with load; influences the variation of the horizontal position of the peak aligning moment with the load
        double q_Hz3 = 0;   // Horizontal shift of the aligning moment peak value point B with camber; influences the variation of the horizontal position of the peak aligning moment with camber
        double q_Hz4 = 0;   // Horizontal shift of the aligning moment peak value point B with inclination; influences the variation of the horizontal position of the peak aligning moment with inclination

        double r_Bx1 = 5;   // Stiffness factor B for longitudinal force; influences the stiffness factor for combined slip conditions
        double r_Bx2 = 8;   // Stiffness factor B for longitudinal force; influences the stiffness factor for combined slip conditions
        double r_Cx1 = 1;   // Shape factor C for longitudinal force; influences the stiffness of the force curve for combined slip conditions
        double r_Ex1 = 0;   // Curvature factor E for longitudinal force; influences the curvature factor for combined slip conditions
        double r_Ex2 = 0;   // Curvature factor E for longitudinal force; influences the curvature factor for combined slip conditions
        double r_Hx1 = 0;   // Horizontal shift of the longitudinal force peak value point B; influences the horizontal position of the peak force for combined slip conditions

        double r_By1 = 7;   // Stiffness factor B for lateral force; influences the stiffness factor for combined slip conditions
        double r_By2 = 2.5; // Stiffness factor B for lateral force; influences the stiffness factor for combined slip conditions
        double r_By3 = 0;   // Shape factor B for lateral force; influences the shape factor for combined slip conditions
        double r_Cy1 = 1;   // Shape factor C for lateral force; influences the stiffness of the force curve for combined slip conditions
        double r_Ey1 = 0;   // Curvature factor E for lateral force; influences the curvature factor for combined slip conditions
        double r_Ey2 = 0;   // Curvature factor E for lateral force; influences the curvature factor for combined slip conditions
        double r_Hy1 = 0;   // Horizontal shift of the lateral force peak value point B; influences the horizontal position of the peak force for combined slip conditions
        double r_Hy2 = 0;   // Horizontal shift of the lateral force peak value point B with load; influences the variation of the horizontal position of the peak force with the load for combined slip conditions
        double r_Vy1 = 0;   // Vertical shift of the lateral force peak value point B; influences the vertical position of the peak force for combined slip conditions
        double r_Vy2 = 0;   // Vertical shift of the lateral force peak value point B with load; influences the variation of the vertical position of the peak force with the load for combined slip conditions
        double r_Vy3 = -0.2;    // Vertical shift of the lateral force peak value point B with inclination; influences the variation of the vertical position of the peak force with inclination for combined slip conditions
        double r_Vy4 = 14;  // Influence of inclination on the lateral force peak value point B; influences the variation of the vertical position of the peak force with inclination for combined slip conditions
        double r_Vy5 = 1.9; // Influence of inclination on the lateral force peak value point B; influences the variation of the vertical position of the peak force with inclination for combined slip conditions
        double r_Vy6 = 10;  // Influence of inclination on the lateral force peak value point B; influences the variation of the vertical position of the peak force with inclination for combined slip conditions

        double s_sz1 = 0;   // Shape factor S for longitudinal slip stiffness; influences the stiffness of the slip stiffness curve
        double s_sz2 = -0.1;    // Variation of shape factor S with load; influences the variation of the stiffness of the slip stiffness curve with the load
        double s_sz3 = -1.0;    // Shape factor S for longitudinal slip stiffness; influences the stiffness of the slip stiffness curve
        double s_sz4 = 0;   // Shape factor S for longitudinal slip stiffness; influences the stiffness of the slip stiffness curve
    }
}

/**
*   \struct    state
*   \brief     The vehicle internal state
    The state consist of position, rotation, position derivative, rotation derivative, and the wheels angular rotation speed
*/
struct state
{
    double* __data = new double[STATE_DIM]; // state data
    double* x = &(this->__data[0]); // x position; represents the vehicle position on a 2D plane in the x-axis
    double* vx = &(this->__data[1]);    // x velocity;  represents the velocity of the vehicle in the x-axis
    double* y = &(this->__data[2]); // y position;  represents the position of the vehicle in the y-axis
    double* vy = &(this->__data[3]);    // y velocity;  represents the velocity of the vehicle in the y-axis
    double* z = &(this->__data[4]); // z position;  represents the position of the vehicle in the z-axis
    double* vz = &(this->__data[5]);    // z velocity;  represents the velocity of the vehicle in the z-axis

    double* r = &(this->__data[6]); // roll;    represents the roll of the vehicle, which is the rotation around the longitudinal axis
    double* vr = &(this->__data[7]);    // roll velocity;   indicating how fast the vehicle is rotating around the longitudinal axis
    double* p = &(this->__data[8]); // pitch;   which is the rotation around the lateral axis
    double* vp = &(this->__data[9]);    // pitch velocity; showing how fast the pitch is changing
    double* yaw = &(this->__data[10]);  // yaw; represents the rotation around the vertical axis
    double* vyaw = &(this->__data[11]); // yaw velocity; indicates how fast the yaw is changing

    double* om_fl = &(this->__data[12]);    // front left wheel angular velocity
    double* om_fr = &(this->__data[13]);    // front right wheel angular velocity
    double* om_rl = &(this->__data[14]);    // rear left wheel angular velocity
    double* om_rr = &(this->__data[15]);    // rear right wheel angular velocity

    void init(double* dat)
    {
        for (int i = 0; i < STATE_DIM; ++i)
            this->__data[i] = dat[i];
    }
};

/**
*   \struct    ctrl
*   \brief     The control to be applied to the vehicle at a given timestep
    The control to apply consist of the couple T on each of the 4 wheels (each wheel can have a specific couple) and the steering \delta of the vehicle
*/
struct ctrl
{
    double* __data = new double[CTRL_DIM];  // control data
    double* T = &(this->__data[0]); // couple on each wheel
    double* steer = &(this->__data[4]); // steering angle

    void init(double* dat)
    {
        for (int i = 0; i < CTRL_DIM; ++i)
            this->__data[i] = dat[i];
    }
};

/**
*   \struct    helper_t
*   \brief     Used to keep track of other state values: ax ay sr1 sr2 sr3 sr4 sa1 sa2 sa3 sa4
*/
struct helper_t {
    double* __data = new double[HELPER_T_DIM];  // helper_t data
    double* ax = &(this->__data[0]);    // acceleration in the x-axis
    double* ay = &(this->__data[1]);    // acceleration in the y-axis
    double* sr1 = &(this->__data[2]);   // slip ratio of the front left wheel
    double* sr2 = &(this->__data[3]);   // slip ratio of the front right wheel
    double* sr3 = &(this->__data[4]);   // slip ratio of the rear left wheel
    double* sr4 = &(this->__data[5]);   // slip ratio of the rear right wheel
    double* sa1 = &(this->__data[6]);   // slip angle of the front left wheel
    double* sa2 = &(this->__data[7]);   // slip angle of the front right wheel
    double* sa3 = &(this->__data[8]);   // slip angle of the rear left wheel
    double* sa4 = &(this->__data[9]);   // slip angle of the rear right wheel
    void init(double* dat)
    {
        for (int i = 0; i < HELPER_T_DIM; ++i)
            this->__data[i] = dat[i];
    }
};

/**
*   \fn        sign
*   \brief     Returns the sign of the argument. (i.e. +1 if the value provided is positive, and -1 otherwise.)
*/
double sign(double x)   // sign function
{
    return (x >= 0) - (x < 0);
}

/**
*   \fn        compute_slip
*   \brief     TODO
*/
void compute_slip(const double* X, double steer, double* sr, double* sa)
/* compute slip ratio and slip angle (X is the state, steer is the steering angle
    sr is the slip ratio, sa is the slip angle) */
{
    double xxx = params::lw / 2 * X[11];    // Half of the distance between the two front wheels times the yaw rate
    double yyyf = params::lf * X[11];   // Distance from the  front axle to the tire times the yaw rate
    double yyyr = params::lr * X[11];   // Distance from the rear axle to the tire times the yaw rate

    double V_pneu_x[4] = { X[1] - xxx, X[1] + xxx, X[1] - xxx, X[1] + xxx };    // x component of the velocity of the wheels
    double V_pneu_y[4] = { X[3] + yyyf, X[3] + yyyf, X[3] - yyyr, X[3] - yyyr };    // y component of the velocity of the wheels

    double c = std::cos(steer); // cosine of the steering angle
    double s = std::sin(steer); // sine of the steering angle

    double V_tire_t[4] = { c*V_pneu_x[0] - s*V_pneu_y[0], c*V_pneu_x[1] - s*V_pneu_y[1], V_pneu_x[2], V_pneu_x[3] };    // tangential velocity of the wheels

    for (int k = 0; k < 4; ++k) // for each wheel
    {
        if (X[k + 12] * V_tire_t[k] >= 0)   // if the product of the angular velocity of the wheel and the tangential velocity of the wheel is positive
        {
            if (std::abs(params::r_eff*X[k + 12]) >= std::abs(V_tire_t[k])) // if the absolute value of the product of the effective radius of the wheel and the angular velocity of the wheel is greater than the absolute value of the tangential velocity of the wheel
            {
                sr[k] = (params::r_eff*X[k + 12] - V_tire_t[k]) / (params::r_eff*X[k + 12] + sign(X[k + 12])*EPS);  // calculate the slip ratio
            }
            else {
                sr[k] = (params::r_eff*X[k + 12] - V_tire_t[k]) / (V_tire_t[k] + sign(V_tire_t[k])*EPS);    // calculate the slip ratio
            }
        }
        else if (V_tire_t[k] < 0)   // if the tangential velocity of the wheel is negative
        {
            sr[k] = 1;  // set the slip ratio to 1
        }
        else {
            sr[k] = -1; // set the slip ratio to -1
        }

        sa[k] = -std::atan2(V_pneu_y[k], V_pneu_x[k] + EPS*sign(V_pneu_x[k]));  // calculate the slip angle
        if (k < 2)  // if the wheel is a front wheel
            sa[k] += steer; // add the steering angle to the slip angle
    }
}

/**
*   \fn        tire_forces
*   \brief     TODO
*   \brief    TODO : 29.04.24
*/
void tire_forces(const double* tau_x, const double* slip_angle, const double* fz, const double* gamma, const double* mu, double* Fxp, double* Fyp, double* dFz)
/* compute the tire forces (tau_x is the longitudinal slip, slip_angle is the slip angle, fz is the vertical tire load, gamma is the inclination angle, mu is the friction coefficient,
    Fxp is the longitudinal tire force, Fyp is the lateral tire force, dFz is the vertical tire load changes) */
{
    double tau_shift[4];    // tire longitudinal slip
    double slip_angle_shift[4]; // tire slip angle
    double Fxp0[4]; // longitudinal tire force
    double Fyp0[4]; // lateral tire force
    double G_xa[4]; // longitudinal force scaling factor
    double G_yk[4]; // lateral force scaling factor

    for (int k = 0; k < 4; ++k) // for each wheel
    {
        double dfz = (fz[k] - params::fz0[k]) / params::fz0[k];   // compute normalized vertical tire load changes
        dFz[k] = dfz;   // vertical tire load on the current wheel 'k'; it represents the force exerted by the vehicle's mass on the wheel

        double S_Hx = params::tires::p_Hx1; // 'S_Hx' is the longitudinal shift of the longitudinal force peak value point B
        tau_shift[k] = tau_x[k] + S_Hx; // compute the longitudinal tire slip
        double C_x = params::tires::p_Cx1;  // 'C_x' is the longitudinal stiffness factor for the tire
        double mu_x = mu[k] * (params::tires::p_Dx1 + params::tires::p_Dx2*dfz);    // 'mu_x' is the longitudinal friction coefficient considering load changes
        double D_x = mu_x*fz[k];    // 'D_x' is the longitudinal friction force
        double E_x = (params::tires::p_Ex1 + params::tires::p_Ex2*dfz + params::tires::p_Ex3*dfz*dfz)*(1 - params::tires::p_Ex4*sign(tau_shift[k]));    // 'E_x' is the longitudinal curvature factor
        double K_xk = fz[k] * (params::tires::p_Kx1 + params::tires::p_Kx2*dfz)*exp(params::tires::p_Kx3*dfz);  // 'K_xk' is the longitudinal slip stiffness
        double B_x = K_xk / (C_x*D_x);  // 'B_x' is the longitudinal stiffness factor
        double S_Vx = fz[k] * (params::tires::p_Vx1 + params::tires::p_Vx2*dfz);    // 'S_Vx' is the vertical shift of the longitudinal force peak value point B

        Fxp0[k] = D_x*sin(C_x*atan(B_x*tau_shift[k] - E_x*(B_x*tau_shift[k] - atan(B_x*tau_shift[k])))) + S_Vx;   // compute the longitudinal tire force

        double S_Hy = (params::tires::p_Hy1 + params::tires::p_Hy2*dfz) + params::tires::p_Hy3*gamma[k];    // 'S_Hy' is the lateral shift of the lateral force peak value point B
        slip_angle_shift[k] = slip_angle[k] + S_Hy; // compute the lateral tire slip

        double C_y = params::tires::p_Cy1;  // 'C_y' is the lateral stiffness factor for the tire
        double mu_y = mu[k] * (params::tires::p_Dy1 + params::tires::p_Dy2*dfz)*(1 - params::tires::p_Dy3*gamma[k] * gamma[k]);   // 'mu_y' is the lateral friction coefficient considering load and inclination changes
        double D_y = mu_y*fz[k];    // 'D_y' is the lateral friction force
        double E_y = (params::tires::p_Ey1 + params::tires::p_Ey2*dfz)*(1 - (params::tires::p_Ey3 + params::tires::p_Ey4*gamma[k] * sign(slip_angle_shift[k])));    // 'E_y' is the lateral curvature factor
        double K_ya0 = params::tires::p_Ky1*params::fz0[k] * sin(2 * atan(fz[k] / (params::tires::p_Ky2*params::fz0[k])));  // 'K_ya0' is the lateral stiffness factor
        double K_ya = K_ya0*(1 - params::tires::p_Ky3*gamma[k] * gamma[k]);   // 'K_ya' is the lateral stiffness factor considering inclination changes
        double B_y = K_ya / (C_y*D_y);  // 'B_y' is the lateral stiffness factor
        double S_Vy = fz[k] * ((params::tires::p_Vy1 + params::tires::p_Vy2*dfz) + (params::tires::p_Vy3 + params::tires::p_Vy4*dfz)*gamma[k]);   // 'S_Vy' is the vertical shift of the lateral force peak value point B

        Fyp0[k] = D_y*sin(C_y*atan(B_y*slip_angle_shift[k] - E_y*(B_y*slip_angle_shift[k] - atan(B_y*slip_angle_shift[k])))) + S_Vy;    // compute the lateral tire force

        double B_xa = params::tires::r_Bx1*cos(atan(params::tires::r_Bx2*tau_x[k]));    // compute the longitudinal stiffness factor
        double C_xa = params::tires::r_Cx1; // compute the longitudinal stiffness factor
        double E_xa = params::tires::r_Ex1 + params::tires::r_Ex2*dfz;  // compute the longitudinal curvature factor
        double S_Hxa = params::tires::r_Hx1;    // compute the longitudinal shift of the longitudinal force peak value point B

        slip_angle_shift[k] = slip_angle[k] + S_Hxa;    // compute the longitudinal tire slip

        G_xa[k] = (cos(C_xa*atan(B_xa*slip_angle_shift[k] - E_xa*(B_xa*slip_angle_shift[k] - atan(B_xa*slip_angle_shift[k]))))) / (cos(C_xa*atan(B_xa*S_Hxa - E_xa*(B_xa*S_Hxa - atan(B_xa*S_Hxa)))));  // compute the longitudinal force scaling factor

        Fxp[k] = G_xa[k] * Fxp0[k];   // compute the longitudinal tire force

        double B_yk = params::tires::r_By1*cos(atan(params::tires::r_By2*(slip_angle[k] - params::tires::r_By3)));  // compute the lateral stiffness factor
        double C_yk = params::tires::r_Cy1; // compute the lateral stiffness factor
        double E_yk = params::tires::r_Ey1 + params::tires::r_Ey2*dfz;  // compute the lateral curvature factor
        double S_Hyk = params::tires::r_Hy1 + params::tires::r_Hy2*dfz;   // compute the lateral shift of the lateral force peak value point B
        double D_Vyk = mu_y*fz[k] * (params::tires::r_Vy1 + params::tires::r_Vy2*dfz + params::tires::r_Vy3*gamma[k])*cos(atan(params::tires::r_Vy4*slip_angle[k]));    // compute the lateral friction force
        double S_Vyk = D_Vyk*sin(params::tires::r_Vy5*atan(params::tires::r_Vy6*tau_x[k]));   // compute the lateral shift of the lateral force peak value point B

        tau_shift[k] = tau_x[k] + S_Hyk;    // compute the lateral tire slip
        G_yk[k] = (cos(C_yk*atan(B_yk*tau_shift[k] - E_yk*(B_yk*tau_shift[k] - atan(B_yk*tau_shift[k]))))) / (cos(C_yk*atan(B_yk*S_Hyk - E_yk*(B_yk*S_Hyk - atan(B_yk*S_Hyk)))));   // compute the lateral force scaling factor
        Fyp[k] = G_yk[k] * Fyp0[k] + S_Vyk;   // compute the lateral tire force
    }
}

/**
*   \fn        compute_susp_forces
*   \brief     TODO
*/
void compute_susp_forces(const double* X, double* F_susp, double* Delta_z_susp, double* d_z_susp)   // compute the suspension forces
{
    for (int k = 0; k < 2; ++k) // for each front wheel
    {
        double sgn = (k % 2) ? -1. : 1.;    // compute the sign of the wheel
        Delta_z_susp[k] = sgn*params::lw / 2 * sin(X[6]) - params::lf*cos(X[6])*sin(X[8]);  // compute the vertical displacement of the suspension
        d_z_susp[k] = sgn*params::lw / 2 * X[7] * cos(X[6]) + params::lf*X[7] * sin(X[6])*sin(X[8]) - params::lf*X[9] * cos(X[6]) * cos(X[8]);  // compute the vertical velocity of the suspension
        F_susp[k] = -params::k_susp_f*Delta_z_susp[k] - params::d_susp_f*d_z_susp[k];   // compute the suspension force
    }

    for (int k = 2; k < 4; ++k)
    {
        double sgn = (k % 2) ? -1. : 1.;    // compute the sign of the wheel
        Delta_z_susp[k] = sgn*params::lw / 2 * sin(X[6]) + params::lr*cos(X[6])*sin(X[8]);  // compute the vertical displacement of the suspension
        d_z_susp[k] = sgn*params::lw / 2 * X[7] * cos(X[6]) - params::lr*X[7] * sin(X[6])*sin(X[8]) + params::lr*X[9] * cos(X[6]) * cos(X[8]);  // compute the vertical velocity of the suspension
        F_susp[k] = -params::k_susp_f*Delta_z_susp[k] - params::d_susp_f*d_z_susp[k];   // compute the suspension force
    }
}

/**
*   \fn        force_fame_change
*   \brief     TODO
*/
void force_fame_change(const double* Fxp, const double* Fyp, const double* Fz, const double* X, double steer, double* Fx, double* Fy)   // compute the forces in the vehicle frame
{
    double delta_steer[4] = { steer, steer, 0, 0 };   // compute the steering angle of the wheels
    for (int k = 0; k < 4; ++k) // for each wheel
    {
        Fx[k] = (Fxp[k] * cos(delta_steer[k]) - Fyp[k] * sin(delta_steer[k]))*cos(X[8]) - Fz[k] * sin(X[8]);    // compute the longitudinal force
        Fy[k] = (Fxp[k] * cos(delta_steer[k]) - Fyp[k] * sin(delta_steer[k]))*sin(X[6])*sin(X[8]) + (Fyp[k] * cos(delta_steer[k]) + Fxp[k] * sin(delta_steer[k]))*cos(X[6]) - Fz[k] * sin(X[6])*cos(X[8]);  // compute the lateral force
    }
}

/**
*   \fn        comp_derivative
*   \brief     TODO
*/
void comp_derivative(const double* X, const double* u, const double* mu, double* dX)    // compute the derivative of the state
{
    double tau_x[4];    // longitudinal slip
    double slip_angle[4];   // slip angle

    double steer = u[4];    // steering angle

    compute_slip(X, steer, tau_x, slip_angle);  // compute the slip ratio and slip angle

    double Delta_F_susp[4]; // suspension forces
    double Delta_z_susp[4]; // vertical displacement of the suspension
    double d_z_susp[4]; // vertical velocity of the suspension
    compute_susp_forces(X, Delta_F_susp, Delta_z_susp, d_z_susp);   // compute the suspension forces

    double Fz[4];   // vertical tire load
    for (int k = 0; k < 4; ++k) // for each wheel
        Fz[k] = params::fz0[k] + Delta_F_susp[k];   // compute the vertical tire load

    double gamma[4] = { 0,0,0,0 };  // inclination angle
    double Fxp[4];  // longitudinal tire force
    double Fyp[4];  // lateral tire force
    double dFz[4];  // vertical tire load changes
    tire_forces(tau_x, slip_angle, Fz, gamma, mu, Fxp, Fyp, dFz);   // compute the tire forces


    double Fx[4];   // longitudinal force
    double Fy[4];   // lateral force
    force_fame_change(Fxp, Fyp, Fz, X, steer, Fx, Fy);  // compute the forces in the vehicle frame

    double F_aero = params::aero_coef * X[1] * X[1];    // compute the aerodynamic force

    double sFx = 0; // sum of the longitudinal forces
    double sFy = 0; // sum of the lateral forces
    for (int k = 0; k < 4; ++k) // for each wheel
    {
        sFx += Fx[k];   // sum the longitudinal forces
        sFy += Fy[k];   // sum the lateral forces
    }

    dX[0] = X[1] * cos(X[10]) - X[3] * sin(X[10]);  // compute the x position derivative
    dX[1] = X[11] * X[3] - X[9] * X[5] + (Fx[0] + Fx[1] + Fx[2] + Fx[3] - F_aero) / params::mass;   // compute the x velocity derivative
    dX[2] = X[1] * sin(X[10]) + X[3] * cos(X[10]);  // compute the y position derivative
    dX[3] = -X[11] * X[1] + X[7] * X[5] + (Fy[0] + Fy[1] + Fy[2] + Fy[3]) / params::mass;   // compute the y velocity derivative
    dX[4] = X[5];   // compute the z position derivative
    dX[5] = 0;  // compute the z velocity derivative
    dX[6] = X[7];   // compute the roll derivative
    dX[7] = 1. / (params::Ix)*(params::lw / 2 * (Delta_F_susp[0] + Delta_F_susp[2] - Delta_F_susp[1] - Delta_F_susp[3]) + X[4] * sFy);  // compute the roll velocity derivative
    dX[8] = X[9];   // compute the pitch derivative
    dX[9] = 1 / params::Iy *(-params::lf*(Delta_F_susp[0] + Delta_F_susp[1]) + params::lr*(Delta_F_susp[2] + Delta_F_susp[3]) - X[4] * sFx);    // compute the pitch velocity derivative
    dX[10] = X[11]; // compute the yaw derivative
    dX[11] = 1. / params::Iz*(params::lf*(Fy[0] + Fy[1]) - params::lr*(Fy[2] + Fy[3]) + params::lw / 2.*(Fx[1] + Fx[3] - Fx[0] - Fx[2]));   // compute the yaw velocity derivative
    for (int k = 0; k < 4; ++k) // for each wheel
        dX[k + 12] = (u[k] - params::r_eff*Fxp[k]) / params::Ir;    // compute the angular velocity derivative
}

/**
*   \fn        propagate
*   \brief     TODO
*/
void propagate(const double* X, const double* dX, double h, double* Xnew)   // propagate the state
{
    for (int i = 0; i < STATE_DIM; ++i) // for each state variable
        Xnew[i] = X[i] + dX[i] * h; // compute the new state variable

    for (int k = 0; k < 4; ++k) // for each wheel
        Xnew[k + 12] = std::fmax(std::fmin(Xnew[k + 12], params::V_max / params::r_eff), 0.);   // compute the new angular velocity
    Xnew[1] = std::fmax(std::fmin(Xnew[1], params::V_max), 0.);   // compute the new velocity
}

/**
*   \fn        rk4
*   \brief     Applies a fourth-order Runge-Kutta approach to approximate the solution of the differential equations.
*/
state rk4(const state &X, const ctrl &u, const double* mu, double h, helper_t& helper)  // apply the fourth-order Runge-Kutta method
{
    state Xnew; // new state
    double* Xtmp = (double*)malloc(sizeof(double)*STATE_DIM);   // temporary state
    double* dX1 = (double*)malloc(sizeof(double)*STATE_DIM);    // derivative 1; it represents the slope at the beginning of the interval
    double* dX2 = (double*)malloc(sizeof(double)*STATE_DIM);    // derivative 2; it represents the slope at the midpoint of the interval
    double* dX3 = (double*)malloc(sizeof(double)*STATE_DIM);    // derivative 3; it represents the slope at the midpoint of the interval
    double* dX4 = (double*)malloc(sizeof(double)*STATE_DIM);    // derivative 4; it represents the slope at the end of the interval

    comp_derivative(X.__data, u.__data, mu, dX1); // k1; compute the derivative at the beginning of the interval

    propagate(X.__data, dX1, h / 2, Xtmp); // Xtmp = X + h/2 dX1; compute the state at the midpoint of the interval

    comp_derivative(Xtmp, u.__data, mu, dX2); // k2; compute the derivative at the midpoint of the interval

    propagate(X.__data, dX2, h / 2, Xtmp); // Xtmp = X + h/2 dX2; compute the state at the midpoint of the interval
    comp_derivative(Xtmp, u.__data, mu, dX3); // k3; compute the derivative at the midpoint of the interval

    propagate(X.__data, dX3, h, Xtmp);  // Xtmp = X + h dX3; compute the state at the end of the interval
    comp_derivative(Xtmp, u.__data, mu, dX4); // k4; compute the derivative at the end of the interval

    double tau_x[4];    // longitudinal slip
    double slip_angle[4];   // slip angle

    for (int i = 0; i < STATE_DIM; ++i) // for each state variable
    {
        Xnew.__data[i] = X.__data[i] + h / 6. * (dX1[i] + 2 * dX2[i] + 2 * dX3[i] + dX4[i]);    // compute the new state variable
        if (i == 1 || i == 3)
        {
            helper.__data[(int)(i-1)/2] = 1./6.*(dX1[i] + 2 * dX2[i] + 2 * dX3[i] + dX4[i]);    // compute the acceleration in the x-axis and y-axis
        }
    }

    compute_slip(Xnew.__data, *u.steer, tau_x, slip_angle); // compute the slip ratio and slip angle

    for (int i = 0; i < 4; ++i) // for each wheel
    {
        helper.__data[2+i] = tau_x[i];  // compute the longitudinal slip
        helper.__data[2+4+i] = slip_angle[i];   // compute the slip angle
    }

    free(Xtmp); // free the memory allocated for the temporary state
    free(dX1);  // free the memory allocated for the derivative 1
    free(dX2);  // free the memory allocated for the derivative 2
    free(dX3);  // free the memory allocated for the derivative 3
    free(dX4);  // free the memory allocated for the derivative 4

    return Xnew;    // return the new state
}

/**
*   \fn        simulate
*   \brief     Compute the state of the vehicle at each timestep under the controls provided.
*/
std::vector<state> simulate(state X0, std::vector<ctrl> controls, double* mu, double dt, std::vector<helper_t>& helper)   // simulate the vehicle
{
    helper_t _helper;   // helper_t data
    std::vector<state> out; // output data
    out.reserve(controls.size());   // reserve space for the output data
    out.push_back(X0);  // add the initial state to the output data
    state curState = X0;    // current state
    for (std::vector<ctrl>::size_type i = 0; i < controls.size(); ++i)  // for each control
    {
        curState = rk4(curState, controls[i], mu, dt, _helper);   // compute the new state
        out.push_back(curState);    // add the new state to the output data
        helper.push_back(_helper);  // add the helper data to the output data
    }
    return out; // return the output data
}

/**
*   \fn        run
*   \brief     Run a simulation from python and return the vehicle simulated states back to python
*/
boost::python::numpy::ndarray run(int duration, double dt, boost::python::numpy::ndarray initial_conditions_ndarray,
                                  boost::python::numpy::ndarray controls_ndarray, boost::python::numpy::ndarray mu_ndarray) // run the simulation
{
    // mu = 1 (for each wheel) when it's sunny
    double mu[4] = {
            boost::python::extract<double>(mu_ndarray[0]),  // friction coefficient of the front left wheel
            boost::python::extract<double>(mu_ndarray[1]),  // friction coefficient of the front right wheel
            boost::python::extract<double>(mu_ndarray[2]),  // friction coefficient of the rear left wheel
            boost::python::extract<double>(mu_ndarray[3]),  // friction coefficient of the rear right wheel
    };

    // Initial state of the vehicle
    state state_X_init;
    // Xg Vx Yg Vy Zg Vz roll droll pitch dpitch yaw dyaw omega1 omega2 omega3 omega4 ax ay sr1 sr2 sr3 sr4 sa1 sa2 sa3 sa4 u1 u2 u3 u4 delta
    double x = boost::python::extract<double>(initial_conditions_ndarray[0]);   // x position
    double vx = boost::python::extract<double>(initial_conditions_ndarray[1]);  // x velocity
    double y = boost::python::extract<double>(initial_conditions_ndarray[2]);   // y position
    double vy = boost::python::extract<double>(initial_conditions_ndarray[3]);  // y velocity
    double z = boost::python::extract<double>(initial_conditions_ndarray[4]);   // z position
    double vz = boost::python::extract<double>(initial_conditions_ndarray[5]);  // z velocity
    double r = boost::python::extract<double>(initial_conditions_ndarray[6]);   // roll
    double vr = boost::python::extract<double>(initial_conditions_ndarray[7]);  // roll velocity
    double p = boost::python::extract<double>(initial_conditions_ndarray[8]);   // pitch
    double vp = boost::python::extract<double>(initial_conditions_ndarray[9]);  // pitch velocity
    double yaw = boost::python::extract<double>(initial_conditions_ndarray[10]);    // yaw
    double vyaw = boost::python::extract<double>(initial_conditions_ndarray[11]);   // yaw velocity
    *state_X_init.x = x;    // x position
    *state_X_init.vx = vx;  // x velocity
    *state_X_init.y = y;    // y position
    *state_X_init.vy = vy;  // y velocity
    *state_X_init.z = z;    // z position
    *state_X_init.vz = vz;  // z velocity
    *state_X_init.r = r;    // roll
    *state_X_init.vr = vr;  // roll velocity
    *state_X_init.p = p;    // pitch
    *state_X_init.vp = vp;  // pitch velocity
    *state_X_init.yaw = yaw;    // yaw
    *state_X_init.vyaw = vyaw;  // yaw velocity
    *state_X_init.om_fl = *state_X_init.vx / params::r_eff;   // angular velocity of the front left wheel
    *state_X_init.om_fr = *state_X_init.vx / params::r_eff;  // angular velocity of the front right wheel
    *state_X_init.om_rl = *state_X_init.vx / params::r_eff; // angular velocity of the rear left wheel
    *state_X_init.om_rr = *state_X_init.vx / params::r_eff; // angular velocity of the rear right wheel

    // Controls to apply at each timestep
    std::vector<ctrl> my_ctrls;
    std::vector<helper_t> my_helpers;   // helper data
    for (int i = 0; i < duration; ++i)  // for each timestep
    {
        if (controls_ndarray.get_nd() == 1 && controls_ndarray.shape(0) == 5)   // if the controls are constant
        {
            // constant control to apply
            ctrl m_ctrl;
            double T_front_left = boost::python::extract<double>(controls_ndarray[0]);  // torque of the front left wheel
            double T_front_right = boost::python::extract<double>(controls_ndarray[1]); // torque of the front right wheel
            double T_back_left = boost::python::extract<double>(controls_ndarray[2]);   // torque of the rear left wheel
            double T_back_right = boost::python::extract<double>(controls_ndarray[3]);  // torque of the rear right wheel
            double steer = boost::python::extract<double>(controls_ndarray[4]); // steering angle
            m_ctrl.T[0] = T_front_left; // torque of the front left wheel
            m_ctrl.T[1] = T_front_right;    // torque of the front right wheel
            m_ctrl.T[2] = T_back_left;  // torque of the rear left wheel
            m_ctrl.T[3] = T_back_right; // torque of the rear right wheel
            *m_ctrl.steer = steer;  // steering angle
            my_ctrls.push_back(m_ctrl); // add the control to the controls to apply
        }
        else if (controls_ndarray.get_nd() == 2 && controls_ndarray.shape(0) == duration && controls_ndarray.shape(1) == 5)   // if the controls are time-varying
        {
            // this timestep's control to apply
            double T_front_left = boost::python::extract<double>(controls_ndarray[i][0]);   // torque of the front left wheel
            double T_front_right = boost::python::extract<double>(controls_ndarray[i][1]);  // torque of the front right wheel
            double T_back_left = boost::python::extract<double>(controls_ndarray[i][2]);    // torque of the rear left wheel
            double T_back_right = boost::python::extract<double>(controls_ndarray[i][3]);   // torque of the rear right wheel
            double steer = boost::python::extract<double>(controls_ndarray[i][4]);  // steering angle
            ctrl m_ctrl;    // control to apply
            m_ctrl.T[0] = T_front_left; // torque of the front left wheel
            m_ctrl.T[1] = T_front_right;    // torque of the front right wheel
            m_ctrl.T[2] = T_back_left;  // torque of the rear left wheel
            m_ctrl.T[3] = T_back_right; // torque of the rear right wheel
            *m_ctrl.steer = steer;  // steering angle
            my_ctrls.push_back(m_ctrl); // add the control to the controls to apply
        }
        else {
            cerr << "The controls you provide are not formatted as they should." << endl;   // print an error message
            exit(EXIT_FAILURE); // exit the program
        }
    }

    // Run the vehicle simulation
    std::vector<state> res_states = simulate(state_X_init, my_ctrls, mu, dt, my_helpers);

    // Return values in a python numpy array
    boost::python::tuple shape = boost::python::make_tuple(duration, 1 + STATE_DIM + HELPER_T_DIM);   // shape of the numpy array
    boost::python::numpy::dtype dtype = boost::python::numpy::dtype::get_builtin<float>();  // data type of the numpy array
    boost::python::numpy::ndarray res_numpy_array = boost::python::numpy::zeros(shape, dtype);  // numpy array to return
    for (int i = 0; i < duration; i += 1)   // for each timestep
    {
        // elapsed time
        res_numpy_array[i][0] = i * dt; // elapsed time
        // state
        res_numpy_array[i][1] = *res_states[i].x;   // x position
        res_numpy_array[i][2] = *res_states[i].vx;  // x velocity
        res_numpy_array[i][3] = *res_states[i].y;   // y position
        res_numpy_array[i][4] = *res_states[i].vy;  // y velocity
        res_numpy_array[i][5] = *res_states[i].z;   // z position
        res_numpy_array[i][6] = *res_states[i].vz;  // z velocity
        res_numpy_array[i][7] = *res_states[i].r;   // roll
        res_numpy_array[i][8] = *res_states[i].vr;  // roll velocity
        res_numpy_array[i][9] = *res_states[i].p;   // pitch
        res_numpy_array[i][10] = *res_states[i].vp; // pitch velocity
        res_numpy_array[i][11] = *res_states[i].yaw;    // yaw
        res_numpy_array[i][12] = *res_states[i].vyaw;   // yaw velocity
        res_numpy_array[i][13] = *res_states[i].om_fl;  // angular velocity of the front left wheel
        res_numpy_array[i][14] = *res_states[i].om_fr;  // angular velocity of the front right wheel
        res_numpy_array[i][15] = *res_states[i].om_rl;  // angular velocity of the rear left wheel
        res_numpy_array[i][16] = *res_states[i].om_rr;  // angular velocity of the rear right wheel
        // additional state
        res_numpy_array[i][17] = *my_helpers[i].ax; // acceleration in the x-axis
        res_numpy_array[i][18] = *my_helpers[i].ay; // acceleration in the y-axis
        res_numpy_array[i][19] = *my_helpers[i].sr1;    // slip ratio of the front left wheel
        res_numpy_array[i][20] = *my_helpers[i].sr2;    // slip ratio of the front right wheel
        res_numpy_array[i][21] = *my_helpers[i].sr3;    // slip ratio of the rear left wheel
        res_numpy_array[i][22] = *my_helpers[i].sr4;    // slip ratio of the rear right wheel
        res_numpy_array[i][23] = *my_helpers[i].sa1;    // slip angle of the front left wheel
        res_numpy_array[i][24] = *my_helpers[i].sa2;    // slip angle of the front right wheel
        res_numpy_array[i][25] = *my_helpers[i].sa3;    // slip angle of the rear left wheel
        res_numpy_array[i][26] = *my_helpers[i].sa4;    // slip angle of the rear right wheel
    }
    return res_numpy_array; // return the numpy array
}

/**
*   \fn        cpp_version
*   \brief     Get the C++ code compilation date and time as a python string
*/
boost::python::object cpp_version() // get the C++ code compilation date and time
{
    time_t rawtime; // raw time
    struct tm * timeinfo;   // time information
    char buffer[80];    // buffer

    time (&rawtime);    // get the raw time
    timeinfo = localtime(&rawtime); // get the time information

    strftime(buffer,sizeof(buffer),"%d-%m-%Y %H:%M:%S",timeinfo);   // format the time
    std::string str(buffer);    // convert the time to a string

    return boost::python::str("Compiled on " + str);    // return the time as a python string
}

/**
*   BOOST_PYTHON_MODULE
*   \brief     Expose the c++ functions to python
*   Important: the name provided to the macro (i.e. vehiclemodel) must match the name of the .so generated file
*   Important: initialize boost before defining the functions exposed
*/
BOOST_PYTHON_MODULE(vehiclemodel)   // expose the c++ functions to python
        {
                Py_Initialize();    // initialize python
        boost::python::numpy::initialize(); // initialize numpy
        def("run", run);    // run the simulation
        def("cpp_version", cpp_version);    // get the C++ code compilation date and time
        }