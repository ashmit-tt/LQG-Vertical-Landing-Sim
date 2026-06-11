#include "../include/kalman.hpp"

KalmanFilter::KalmanFilter(double initial_z, double initial_v, double initial_variance, double r_measure, double q_process)
    : z_est{initial_z}, v_est{initial_v}, P00{initial_variance}, P01{0.0}, P11{initial_variance}, R{r_measure}, Q{q_process} {}

StateVector KalmanFilter::update(double z_measured, double thrust, double dt) {
    double accel{(thrust / ROCKET_MASS) - GRAVITY};
    double z_pred{z_est + (v_est * dt) + (0.5 * accel * dt * dt)};
    double v_pred{v_est + (accel * dt)};


    // P = F * P * F^T + Q
    double P00_pred{P00 + (2.0 * dt * P01) + (dt * dt * P11) + Q};
    double P01_pred{P01 + (dt * P11)};
    double P11_pred{P11 + Q};

    double S{P00_pred + R};  // total uncertainty
    double K0{P00_pred / S}; // altitude gain
    double K1{P01_pred / S}; // vel gain

    double y{z_measured - z_pred};

    z_est = z_pred + (K0 * y);
    v_est = v_pred + (K1 * y);

    // P = (I - K * H) * P
    P00 = (1.0 - K0) * P00_pred;
    P01 = (1.0 - K0) * P01_pred;
    P11 = P11_pred - (K1 * P01_pred);


    return {z_est, v_est};
}
