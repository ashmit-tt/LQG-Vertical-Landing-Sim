#include <iostream>
#include <fstream>
#include <iomanip>
#include <algorithm>
#include "../include/config.hpp"
#include "../include/simulation.hpp"
#include "../include/controller.hpp"
#include "../include/noise.hpp"
#include "../include/dynamics.hpp"
#include "../include/kalman.hpp"

int main() {
    StateVector state{100.0, 0.0}; //true actual state of the rocket
    double time{0.0};
    double t_final{25.0};
    double current_thrust{ROCKET_MASS * GRAVITY};

    
    PIDController pid{3.0, 0.05, 8.0};

    //calculated new LQR values using Riccati eq
    LQRController lqr{0.5, 2.69};

    NoiseGenerator altimeter_noise{0.0, 0.5};
    //init z, init v, init var, measurement noise R, process noise Q
    KalmanFilter kf{100.0, 0.0, 10.0, 0.25, 0.0005};

    bool use_lqr{true};

    std::ofstream telemetry{"flight_data.csv"};
    telemetry << "time,z_true,v_true,z_measured,z_est,v_est,thrust\n";

    std::cout << "IGNITION, Running " << (use_lqr ? "LQR Controller" : "PID Controller") << " with KF \n";
    

    while (time < t_final) {
        double z_true{state[0]};
        double v_true{state[1]};

        if (z_true <= 0.0) {
            std::cout << "Touched the ground after " << std::fixed << std::setprecision(2) << time << "s\n";
            break;
        }

        double z_measured{z_true + altimeter_noise.get_sample()}; //adds static to simulate real world scenarios in 1D
        StateVector est_state = kf.update(z_measured, current_thrust, DT);
        double z_est{est_state[0]};
        double v_est{est_state[1]};

        if (use_lqr) {
            StateVector error_state{z_est - TARGET_ALTITUDE, v_est - 0.0};
            current_thrust = lqr.compute(error_state);
        } else {
            current_thrust = pid.compute(TARGET_ALTITUDE, z_est, v_est, DT);
        }
        current_thrust = std::clamp(current_thrust, 0.0, 20.0);
        state = rk4steps(rocket_dynamics, state, current_thrust, DT);

        telemetry << time << ","
                  << z_true << ","
                  << v_true << ","
                  << z_measured << ","
                  << z_est << ","
                  << v_est << ","
                  << current_thrust << "\n";
        time += DT;
    }

    telemetry.close();
    std::cout << "Telemtry successfully saved in flight_data.csv\n";
    return 0;
}