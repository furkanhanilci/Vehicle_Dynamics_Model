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
        double D_x = mu_x*fz[k];
        double E_x = (params::tires::p_Ex1 + params::tires::p_Ex2*dfz + params::tires::p_Ex3*dfz*dfz)*(1 - params::tires::p_Ex4*sign(tau_shift[k]));
        double K_xk = fz[k] * (params::tires::p_Kx1 + params::tires::p_Kx2*dfz)*exp(params::tires::p_Kx3*dfz);
        double B_x = K_xk / (C_x*D_x);
        double S_Vx = fz[k] * (params::tires::p_Vx1 + params::tires::p_Vx2*dfz);

        Fxp0[k] = D_x*sin(C_x*atan(B_x*tau_shift[k] - E_x*(B_x*tau_shift[k] - atan(B_x*tau_shift[k])))) + S_Vx;

        double S_Hy = (params::tires::p_Hy1 + params::tires::p_Hy2*dfz) + params::tires::p_Hy3*gamma[k];
        slip_angle_shift[k] = slip_angle[k] + S_Hy;

        double C_y = params::tires::p_Cy1;
        double mu_y = mu[k] * (params::tires::p_Dy1 + params::tires::p_Dy2*dfz)*(1 - params::tires::p_Dy3*gamma[k] * gamma[k]);
        double D_y = mu_y*fz[k];
        double E_y = (params::tires::p_Ey1 + params::tires::p_Ey2*dfz)*(1 - (params::tires::p_Ey3 + params::tires::p_Ey4*gamma[k] * sign(slip_angle_shift[k])));
        double K_ya0 = params::tires::p_Ky1*params::fz0[k] * sin(2 * atan(fz[k] / (params::tires::p_Ky2*params::fz0[k])));
        double K_ya = K_ya0*(1 - params::tires::p_Ky3*gamma[k] * gamma[k]);
        double B_y = K_ya / (C_y*D_y);
        double S_Vy = fz[k] * ((params::tires::p_Vy1 + params::tires::p_Vy2*dfz) + (params::tires::p_Vy3 + params::tires::p_Vy4*dfz)*gamma[k]);

        Fyp0[k] = D_y*sin(C_y*atan(B_y*slip_angle_shift[k] - E_y*(B_y*slip_angle_shift[k] - atan(B_y*slip_angle_shift[k])))) + S_Vy;

        double B_xa = params::tires::r_Bx1*cos(atan(params::tires::r_Bx2*tau_x[k]));
        double C_xa = params::tires::r_Cx1;
        double E_xa = params::tires::r_Ex1 + params::tires::r_Ex2*dfz;
        double S_Hxa = params::tires::r_Hx1;

        slip_angle_shift[k] = slip_angle[k] + S_Hxa;

        G_xa[k] = (cos(C_xa*atan(B_xa*slip_angle_shift[k] - E_xa*(B_xa*slip_angle_shift[k] - atan(B_xa*slip_angle_shift[k]))))) / (cos(C_xa*atan(B_xa*S_Hxa - E_xa*(B_xa*S_Hxa - atan(B_xa*S_Hxa)))));

        Fxp[k] = G_xa[k] * Fxp0[k];

        double B_yk = params::tires::r_By1*cos(atan(params::tires::r_By2*(slip_angle[k] - params::tires::r_By3)));
        double C_yk = params::tires::r_Cy1;
        double E_yk = params::tires::r_Ey1 + params::tires::r_Ey2*dfz;
        double S_Hyk = params::tires::r_Hy1 + params::tires::r_Hy2*dfz;
        double D_Vyk = mu_y*fz[k] * (params::tires::r_Vy1 + params::tires::r_Vy2*dfz + params::tires::r_Vy3*gamma[k])*cos(atan(params::tires::r_Vy4*slip_angle[k]));
        double S_Vyk = D_Vyk*sin(params::tires::r_Vy5*atan(params::tires::r_Vy6*tau_x[k]));

        tau_shift[k] = tau_x[k] + S_Hyk;
        G_yk[k] = (cos(C_yk*atan(B_yk*tau_shift[k] - E_yk*(B_yk*tau_shift[k] - atan(B_yk*tau_shift[k]))))) / (cos(C_yk*atan(B_yk*S_Hyk - E_yk*(B_yk*S_Hyk - atan(B_yk*S_Hyk)))));
        Fyp[k] = G_yk[k] * Fyp0[k] + S_Vyk;
    }
}

/**
*   \fn        compute_susp_forces
*   \brief     TODO
*/
void compute_susp_forces(const double* X, double* F_susp, double* Delta_z_susp, double* d_z_susp)
{
    for (int k = 0; k < 2; ++k)
    {
        double sgn = (k % 2) ? -1. : 1.;
        Delta_z_susp[k] = sgn*params::lw / 2 * sin(X[6]) - params::lf*cos(X[6])*sin(X[8]);
        d_z_susp[k] = sgn*params::lw / 2 * X[7] * cos(X[6]) + params::lf*X[7] * sin(X[6])*sin(X[8]) - params::lf*X[9] * cos(X[6]) * cos(X[8]);
        F_susp[k] = -params::k_susp_f*Delta_z_susp[k] - params::d_susp_f*d_z_susp[k];
    }

    for (int k = 2; k < 4; ++k)
    {
        double sgn = (k % 2) ? -1. : 1.;
        Delta_z_susp[k] = sgn*params::lw / 2 * sin(X[6]) + params::lr*cos(X[6])*sin(X[8]);
        d_z_susp[k] = sgn*params::lw / 2 * X[7] * cos(X[6]) - params::lr*X[7] * sin(X[6])*sin(X[8]) + params::lr*X[9] * cos(X[6]) * cos(X[8]);
        F_susp[k] = -params::k_susp_f*Delta_z_susp[k] - params::d_susp_f*d_z_susp[k];
    }
}

/**
*   \fn        force_fame_change
*   \brief     TODO
*/
void force_fame_change(const double* Fxp, const double* Fyp, const double* Fz, const double* X, double steer, double* Fx, double* Fy)
{
    double delta_steer[4] = { steer, steer, 0, 0 };
    for (int k = 0; k < 4; ++k)
    {
        Fx[k] = (Fxp[k] * cos(delta_steer[k]) - Fyp[k] * sin(delta_steer[k]))*cos(X[8]) - Fz[k] * sin(X[8]);
        Fy[k] = (Fxp[k] * cos(delta_steer[k]) - Fyp[k] * sin(delta_steer[k]))*sin(X[6])*sin(X[8]) + (Fyp[k] * cos(delta_steer[k]) + Fxp[k] * sin(delta_steer[k]))*cos(X[6]) - Fz[k] * sin(X[6])*cos(X[8]);
    }
}

/**
*   \fn        comp_derivative
*   \brief     TODO
*/
void comp_derivative(const double* X, const double* u, const double* mu, double* dX)
{
    double tau_x[4];
    double slip_angle[4];

    double steer = u[4];

    compute_slip(X, steer, tau_x, slip_angle);

    double Delta_F_susp[4];
    double Delta_z_susp[4];
    double d_z_susp[4];
    compute_susp_forces(X, Delta_F_susp, Delta_z_susp, d_z_susp);

    double Fz[4];
    for (int k = 0; k < 4; ++k)
        Fz[k] = params::fz0[k] + Delta_F_susp[k];

    double gamma[4] = { 0,0,0,0 };
    double Fxp[4];
    double Fyp[4];
    double dFz[4];
    tire_forces(tau_x, slip_angle, Fz, gamma, mu, Fxp, Fyp, dFz);


    double Fx[4];
    double Fy[4];
    force_fame_change(Fxp, Fyp, Fz, X, steer, Fx, Fy);

    double F_aero = params::aero_coef * X[1] * X[1];

    double sFx = 0;
    double sFy = 0;
    for (int k = 0; k < 4; ++k)
    {
        sFx += Fx[k];
        sFy += Fy[k];
    }

    dX[0] = X[1] * cos(X[10]) - X[3] * sin(X[10]);
    dX[1] = X[11] * X[3] - X[9] * X[5] + (Fx[0] + Fx[1] + Fx[2] + Fx[3] - F_aero) / params::mass;
    dX[2] = X[1] * sin(X[10]) + X[3] * cos(X[10]);
    dX[3] = -X[11] * X[1] + X[7] * X[5] + (Fy[0] + Fy[1] + Fy[2] + Fy[3]) / params::mass;
    dX[4] = X[5];
    dX[5] = 0;
    dX[6] = X[7];
    dX[7] = 1. / (params::Ix)*(params::lw / 2 * (Delta_F_susp[0] + Delta_F_susp[2] - Delta_F_susp[1] - Delta_F_susp[3]) + X[4] * sFy);
    dX[8] = X[9];
    dX[9] = 1 / params::Iy *(-params::lf*(Delta_F_susp[0] + Delta_F_susp[1]) + params::lr*(Delta_F_susp[2] + Delta_F_susp[3]) - X[4] * sFx);
    dX[10] = X[11];
    dX[11] = 1. / params::Iz*(params::lf*(Fy[0] + Fy[1]) - params::lr*(Fy[2] + Fy[3]) + params::lw / 2.*(Fx[1] + Fx[3] - Fx[0] - Fx[2]));
    for (int k = 0; k < 4; ++k)
        dX[k + 12] = (u[k] - params::r_eff*Fxp[k]) / params::Ir;
}

/**
*   \fn        propagate
*   \brief     TODO
*/
void propagate(const double* X, const double* dX, double h, double* Xnew)
{
    for (int i = 0; i < STATE_DIM; ++i)
        Xnew[i] = X[i] + dX[i] * h;

    for (int k = 0; k < 4; ++k)
        Xnew[k + 12] = std::fmax(std::fmin(Xnew[k + 12], params::V_max / params::r_eff), 0.);
    Xnew[1] = std::fmax(std::fmin(Xnew[1], params::V_max), 0.);
}

/**
*   \fn        rk4
*   \brief     Applies a fourth-order Runge-Kutta approach to approximate the solution of the differential equations.
*/
state rk4(const state &X, const ctrl &u, const double* mu, double h, helper_t& helper)
{
    state Xnew;
    double* Xtmp = (double*)malloc(sizeof(double)*STATE_DIM);
    double* dX1 = (double*)malloc(sizeof(double)*STATE_DIM);
    double* dX2 = (double*)malloc(sizeof(double)*STATE_DIM);
    double* dX3 = (double*)malloc(sizeof(double)*STATE_DIM);
    double* dX4 = (double*)malloc(sizeof(double)*STATE_DIM);

    comp_derivative(X.__data, u.__data, mu, dX1); // k1

    propagate(X.__data, dX1, h / 2, Xtmp); // Xtmp = X + h/2 dX1

    comp_derivative(Xtmp, u.__data, mu, dX2); // k2

    propagate(X.__data, dX2, h / 2, Xtmp); // Xtmp = X + h/2 dX2
    comp_derivative(Xtmp, u.__data, mu, dX3); // k3

    propagate(X.__data, dX3, h, Xtmp);
    comp_derivative(Xtmp, u.__data, mu, dX4); // k4

    double tau_x[4];
    double slip_angle[4];

    for (int i = 0; i < STATE_DIM; ++i)
    {
        Xnew.__data[i] = X.__data[i] + h / 6. * (dX1[i] + 2 * dX2[i] + 2 * dX3[i] + dX4[i]);
        if (i == 1 || i == 3)
        {
            helper.__data[(int)(i-1)/2] = 1./6.*(dX1[i] + 2 * dX2[i] + 2 * dX3[i] + dX4[i]);
        }
    }

    compute_slip(Xnew.__data, *u.steer, tau_x, slip_angle);

    for (int i = 0; i < 4; ++i)
    {
        helper.__data[2+i] = tau_x[i];
        helper.__data[2+4+i] = slip_angle[i];
    }

    free(Xtmp);
    free(dX1);
    free(dX2);
    free(dX3);
    free(dX4);

    return Xnew;
}

/**
*   \fn        simulate
*   \brief     Compute the state of the vehicle at each timestep under the controls provided.
*/
std::vector<state> simulate(state X0, std::vector<ctrl> controls, double* mu, double dt, std::vector<helper_t>& helper)
{
    helper_t _helper;
    std::vector<state> out;
    out.reserve(controls.size());
    out.push_back(X0);
    state curState = X0;
    for (std::vector<ctrl>::size_type i = 0; i < controls.size(); ++i)
    {
        curState = rk4(curState, controls[i], mu, dt, _helper);
        out.push_back(curState);
        helper.push_back(_helper);
    }
    return out;
}

/**
*   \fn        run
*   \brief     Run a simulation from python and return the vehicle simulated states back to python
*/
boost::python::numpy::ndarray run(int duration, double dt, boost::python::numpy::ndarray initial_conditions_ndarray,
                                  boost::python::numpy::ndarray controls_ndarray, boost::python::numpy::ndarray mu_ndarray)
{
    // mu = 1 (for each wheel) when it's sunny
    double mu[4] = {
            boost::python::extract<double>(mu_ndarray[0]),
            boost::python::extract<double>(mu_ndarray[1]),
            boost::python::extract<double>(mu_ndarray[2]),
            boost::python::extract<double>(mu_ndarray[3]),
    };

    // Initial state of the vehicle
    state state_X_init;
    // Xg Vx Yg Vy Zg Vz roll droll pitch dpitch yaw dyaw omega1 omega2 omega3 omega4 ax ay sr1 sr2 sr3 sr4 sa1 sa2 sa3 sa4 u1 u2 u3 u4 delta
    double x = boost::python::extract<double>(initial_conditions_ndarray[0]);
    double vx = boost::python::extract<double>(initial_conditions_ndarray[1]);
    double y = boost::python::extract<double>(initial_conditions_ndarray[2]);
    double vy = boost::python::extract<double>(initial_conditions_ndarray[3]);
    double z = boost::python::extract<double>(initial_conditions_ndarray[4]);
    double vz = boost::python::extract<double>(initial_conditions_ndarray[5]);
    double r = boost::python::extract<double>(initial_conditions_ndarray[6]);
    double vr = boost::python::extract<double>(initial_conditions_ndarray[7]);
    double p = boost::python::extract<double>(initial_conditions_ndarray[8]);
    double vp = boost::python::extract<double>(initial_conditions_ndarray[9]);
    double yaw = boost::python::extract<double>(initial_conditions_ndarray[10]);
    double vyaw = boost::python::extract<double>(initial_conditions_ndarray[11]);
    *state_X_init.x = x;
    *state_X_init.vx = vx;
    *state_X_init.y = y;
    *state_X_init.vy = vy;
    *state_X_init.z = z;
    *state_X_init.vz = vz;
    *state_X_init.r = r;
    *state_X_init.vr = vr;
    *state_X_init.p = p;
    *state_X_init.vp = vp;
    *state_X_init.yaw = yaw;
    *state_X_init.vyaw = vyaw;
    *state_X_init.om_fl = *state_X_init.vx / params::r_eff;
    *state_X_init.om_fr = *state_X_init.vx / params::r_eff;
    *state_X_init.om_rl = *state_X_init.vx / params::r_eff;
    *state_X_init.om_rr = *state_X_init.vx / params::r_eff;

    // Controls to apply at each timestep
    std::vector<ctrl> my_ctrls;
    std::vector<helper_t> my_helpers;
    for (int i = 0; i < duration; ++i)
    {
        if (controls_ndarray.get_nd() == 1 && controls_ndarray.shape(0) == 5)
        {
            // constant control to apply
            ctrl m_ctrl;
            double T_front_left = boost::python::extract<double>(controls_ndarray[0]);
            double T_front_right = boost::python::extract<double>(controls_ndarray[1]);
            double T_back_left = boost::python::extract<double>(controls_ndarray[2]);
            double T_back_right = boost::python::extract<double>(controls_ndarray[3]);
            double steer = boost::python::extract<double>(controls_ndarray[4]);
            m_ctrl.T[0] = T_front_left;
            m_ctrl.T[1] = T_front_right;
            m_ctrl.T[2] = T_back_left;
            m_ctrl.T[3] = T_back_right;
            *m_ctrl.steer = steer;
            my_ctrls.push_back(m_ctrl);
        }
        else if (controls_ndarray.get_nd() == 2 && controls_ndarray.shape(0) == duration && controls_ndarray.shape(1) == 5)
        {
            // this timestep's control to apply
            double T_front_left = boost::python::extract<double>(controls_ndarray[i][0]);
            double T_front_right = boost::python::extract<double>(controls_ndarray[i][1]);
            double T_back_left = boost::python::extract<double>(controls_ndarray[i][2]);
            double T_back_right = boost::python::extract<double>(controls_ndarray[i][3]);
            double steer = boost::python::extract<double>(controls_ndarray[i][4]);
            ctrl m_ctrl;
            m_ctrl.T[0] = T_front_left;
            m_ctrl.T[1] = T_front_right;
            m_ctrl.T[2] = T_back_left;
            m_ctrl.T[3] = T_back_right;
            *m_ctrl.steer = steer;
            my_ctrls.push_back(m_ctrl);
        }
        else {
            cerr << "The controls you provide are not formatted as they should." << endl;
            exit(EXIT_FAILURE);
        }
    }

    // Run the vehicle simulation
    std::vector<state> res_states = simulate(state_X_init, my_ctrls, mu, dt, my_helpers);

    // Return values in a python numpy array
    boost::python::tuple shape = boost::python::make_tuple(duration, 1 + STATE_DIM + HELPER_T_DIM);
    boost::python::numpy::dtype dtype = boost::python::numpy::dtype::get_builtin<float>();
    boost::python::numpy::ndarray res_numpy_array = boost::python::numpy::zeros(shape, dtype);
    for (int i = 0; i < duration; i += 1)
    {
        // elapsed time
        res_numpy_array[i][0] = i * dt;
        // state
        res_numpy_array[i][1] = *res_states[i].x;
        res_numpy_array[i][2] = *res_states[i].vx;
        res_numpy_array[i][3] = *res_states[i].y;
        res_numpy_array[i][4] = *res_states[i].vy;
        res_numpy_array[i][5] = *res_states[i].z;
        res_numpy_array[i][6] = *res_states[i].vz;
        res_numpy_array[i][7] = *res_states[i].r;
        res_numpy_array[i][8] = *res_states[i].vr;
        res_numpy_array[i][9] = *res_states[i].p;
        res_numpy_array[i][10] = *res_states[i].vp;
        res_numpy_array[i][11] = *res_states[i].yaw;
        res_numpy_array[i][12] = *res_states[i].vyaw;
        res_numpy_array[i][13] = *res_states[i].om_fl;
        res_numpy_array[i][14] = *res_states[i].om_fr;
        res_numpy_array[i][15] = *res_states[i].om_rl;
        res_numpy_array[i][16] = *res_states[i].om_rr;
        // additional state
        res_numpy_array[i][17] = *my_helpers[i].ax;
        res_numpy_array[i][18] = *my_helpers[i].ay;
        res_numpy_array[i][19] = *my_helpers[i].sr1;
        res_numpy_array[i][20] = *my_helpers[i].sr2;
        res_numpy_array[i][21] = *my_helpers[i].sr3;
        res_numpy_array[i][22] = *my_helpers[i].sr4;
        res_numpy_array[i][23] = *my_helpers[i].sa1;
        res_numpy_array[i][24] = *my_helpers[i].sa2;
        res_numpy_array[i][25] = *my_helpers[i].sa3;
        res_numpy_array[i][26] = *my_helpers[i].sa4;
    }
    return res_numpy_array;
}

/**
*   \fn        cpp_version
*   \brief     Get the C++ code compilation date and time as a python string
*/
boost::python::object cpp_version()
{
    time_t rawtime;
    struct tm * timeinfo;
    char buffer[80];

    time (&rawtime);
    timeinfo = localtime(&rawtime);

    strftime(buffer,sizeof(buffer),"%d-%m-%Y %H:%M:%S",timeinfo);
    std::string str(buffer);

    return boost::python::str("Compiled on " + str);
}

/**
*   BOOST_PYTHON_MODULE
*   \brief     Expose the c++ functions to python
*   Important: the name provided to the macro (i.e. vehiclesim) must match the name of the .so generated file
*   Important: initialize boost before defining the functions exposed
*/
BOOST_PYTHON_MODULE(vehiclesim)
        {
                Py_Initialize();
        boost::python::numpy::initialize();
        def("run", run);
        def("cpp_version", cpp_version);
        }
