#ifndef SIMULATION_HPP
#define SIMULATION_HPP
#include "config.hpp"

using DynamicsFunction = StateVector(*)(const StateVector&, double);
StateVector rk4steps(DynamicsFunction f, const StateVector& state, double u, double dt);

#endif