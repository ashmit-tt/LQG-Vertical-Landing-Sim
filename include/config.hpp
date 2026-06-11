#ifndef CONFIG_HPP
#define CONFIG_HPP
#include <array>


using StateVector = std::array<double, 2>;

inline constexpr double GRAVITY = 9.81;
inline constexpr double DT = 0.01;
inline constexpr double TARGET_ALTITUDE = 0.0;
inline constexpr double ROCKET_MASS = 1.0;
inline constexpr double MAX_THRUST = 2.0 * ROCKET_MASS * GRAVITY;

#endif
