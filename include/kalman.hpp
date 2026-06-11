#ifndef KALMAN_HPP
#define KALMAN_HPP
#include "config.hpp"

class KalmanFilter {
public:
    KalmanFilter(double initial_z, double initial_v, double init_var, double r_measure, double q_process);
    StateVector update(double z_measured, double thrust, double dt);

private:
    double z_est;
    double v_est;
    double P00;
    double P01;
    double P11;
    double R; //sensor
    double Q; //phys
};

#endif