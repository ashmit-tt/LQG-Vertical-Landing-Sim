#include "../include/controller.hpp"

PIDController::PIDController(double kp, double ki, double kd) 
    : Kp{kp}, Ki{ki}, Kd{kd}, integral{0.0}, prev_error{0.0} {}

double PIDController::compute(double target_z, double current_z, double current_v, double dt) {

    double error{target_z - current_z};
    double P{Kp * error};
    integral += error * dt;
    double I{Ki * integral};
    double D{Kd * (0.0- current_v)};

    return P + I + D + ROCKET_MASS * GRAVITY; //acc for hover thrust

}

LQRController::LQRController(double k1, double k2)
    : K_1{k1}, K_2{k2} {}

double LQRController::compute(const StateVector& state) {

    double control_input{-(K_1 * state[0] + K_2 * state[1])};

    return control_input + ROCKET_MASS * GRAVITY; //same here
}
