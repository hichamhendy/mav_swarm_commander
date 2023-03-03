# pragma once

struct SystemConstants
{
    double m = 2.0;
    Eigen::Vector3d g = Eigen::Vector3d(0, 0, 9.8066);
    Eigen::Vector3d drag_coefficients = Eigen::Vector3d(0.010000 , 0.010000, 0.0000);
    double psi_gain = 2.8;
    double roll_gain = 20;
    double pitch_gain = 20;
    double maxmin_angle = 0.7853981633974483;
    double max_thrust = m * g[3] * 1.5;
    double min_thrust = m * g[3] * 0.5;
};

/* 
Summarizing the system states in one vector to calculate
velocities, accelerations, and roll and pitch's angular velocity from
second order diffrential equations
Phi to roll around x, theta to pitch around y and Psi to yaw around z.
regarding the documentation https://docs.px4.io/master/en/config/flight_controller_orientation.html#calculating-orientation
Z is positiv downward, so the body pitches upward with a positiv change
 */

class NonlinearStateSpace_ODE
{
    private:
        Eigen::VectorXd x_;
        Eigen::VectorXd u_;
        Eigen::VectorXd dxdt_;
        SystemConstants sys_constants;
        Eigen::MatrixXf r_ = Eigen::MatrixXf::Zero(3, 3); 

    public:
        NonlinearStateSpace_ODE(const Eigen::VectorXd& x, const Eigen::VectorXd& u) : x_(x), u_(u)
        {
            r_(0,0) = cos(x_[6]) * cos(x_[8]);
            r_(0,1) = cos(x_[6]) * sin(x_[8]) * sin(x_[7]) - cos(x_[7]) * sin(x_[6]);
            r_(0,2) = CppAD::cos(x_[6]) * CppAD::sin(x_[8]) * CppAD::cos(x_[7]) + CppAD::sin(x_[6]) * CppAD::sin(x_[7]);
            r_(1,0) = cos(x_[8]) * sin(x_[6]);
            r_(1,1) = cos(x_[6]) * cos(x_[7]) + sin(x_[6]) * sin(x_[8]) * sin(x_[7]);
            r_(1,2) = CppAD::sin(x_[6]) * CppAD::sin(x_[8]) * CppAD::cos(x_[7]) - CppAD::cos(x_[6]) * CppAD::sin(x_[7]);
            r_(2,0) = - sin(x_[8]);
            r_(2,1) = cos(x_[8]) * sin(x_[7]);
            r_(2,2) = CppAD::cos(x_[8]) * CppAD::cos(x_[7]);
        }

        Eigen::VectorXd systemDynamics()
        {
            dxdt_[0] = x_[4];
            dxdt_[1] = x_[5];
            dxdt_[2] = x_[6];
            dxdt_[3] = r_(0 , 2) * (1/sys_constants.m) * u_[3] - sys_constants.drag_coefficients[0] *  x_[4];
            dxdt_[4] = r_(1 , 2) * (1/sys_constants.m) * u_[3] - sys_constants.drag_coefficients[1] *  x_[5];
            dxdt_[5] = sys_constants.g[2] - r_(3 , 2) * (1/sys_constants.m) * u_[3];
            dxdt_[6] =  (sys_constants.psi_gain * u_[3] - x_[6]);
            dxdt_[7] =  (sys_constants.roll_gain * u_[1] - x_[7]);
            dxdt_[8] =  (sys_constants.pitch_gain * u_[2] - x_[8]);

            return dxdt_;
        }
};


class RungeKutta
{
    Eigen::VectorXd k1;
    Eigen::VectorXd k2;
    Eigen::VectorXd k3;
    Eigen::VectorXd k4;
    Eigen::VectorXd x1_;
    Eigen::VectorXd u1_;
    double dt_;
    NonlinearStateSpace_ODE ode1_;


    RungeKutta(const Eigen::VectorXd& x, const Eigen::VectorXd& u, double& dt) : x1_(x), u1_(u), ode1_(x, u), dt_(dt)
    {
        k1 = ode1_.systemDynamics();
        NonlinearStateSpace_ODE ode2_( x1_ + (dt_/2) * k1, u1_);
        k2 = ode2_.systemDynamics();
        NonlinearStateSpace_ODE ode3_( x1_ + (dt_/2) * k2, u1_);
        k3 = ode3_.systemDynamics();
        NonlinearStateSpace_ODE ode4_( x1_ + (dt_) * k3, u1_);
        k4 = ode4_.systemDynamics();
    }

    Eigen::VectorXd propagate()
    {
        return (x1_ + (dt_/6.0) * (k1 + 2 * k2 + 2 * k3 + k4));
    }
};

class ExplicitEuler
{
    Eigen::VectorXd x1_;
    Eigen::VectorXd u1_;
    double dt_;
    NonlinearStateSpace_ODE ode1_;

    ExplicitEuler(const Eigen::VectorXd& x, const Eigen::VectorXd& u, double& dt) : x1_(x), u1_(u), ode1_(x, u), dt_(dt)
    {}

    Eigen::VectorXd propagate()
    {
        return (x1_ + (dt_) * ode1_.systemDynamics());
    }
};