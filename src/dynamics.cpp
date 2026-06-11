#include "../include/dynamics.hpp"

#include <array>
#include <algorithm>

StateVector rocket_dynamics(const StateVector& state, double thrust) {

    double v{state[1]};
    double actual_thrust{std::clamp(thrust, 0.0, MAX_THRUST)};
    double dzdt{v};
    double a{(actual_thrust / ROCKET_MASS) - GRAVITY};

    return {dzdt, a};
}






