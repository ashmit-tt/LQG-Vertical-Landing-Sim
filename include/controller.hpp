#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP
#include "config.hpp"

class PIDController {
public:
    PIDController(double kp, double ki, double kd);
    double compute(double target_z, double current_z, double current_v, double dt);

private:
    double Kp;
    double Ki;
    double Kd;
    double integral;
    double prev_error;
};

class LQRController {
    public:
        LQRController(double k1, double k2);
        double compute(const StateVector& state);
    private:
        double K_1;
        double K_2;
};

#endif
