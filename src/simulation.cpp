#include "../include/simulation.hpp"


//error to dt^5
StateVector rk4steps(DynamicsFunction f, const StateVector& state, double u, double dt) {

    StateVector k1{f(state, u)};

    StateVector state_k2{{ state[0] + 0.5 * dt * k1[0], state[1] + 0.5 * dt * k1[1] }};
    StateVector k2{f(state_k2, u)};

    StateVector state_k3{{ state[0] + 0.5 * dt * k2[0], state[1] + 0.5 * dt * k2[1] }};
    StateVector k3{f(state_k3, u)};

    StateVector state_k4{{ state[0] + dt * k3[0], state[1] + dt * k3[1] }};
    StateVector k4{f(state_k4, u)};

    StateVector next_state{{
        state[0] + (dt / 6.0) * (k1[0] + 2.0 * k2[0] + 2.0 * k3[0] + k4[0]),
        state[1] + (dt / 6.0) * (k1[1] + 2.0 * k2[1] + 2.0 * k3[1] + k4[1])
    }};

    return next_state;
}