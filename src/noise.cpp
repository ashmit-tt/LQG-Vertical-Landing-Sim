#include "../include/noise.hpp"

NoiseGenerator::NoiseGenerator(double mean, double std_dev)
    : generator{std::random_device{}()}, distribution{mean, std_dev} {}

double NoiseGenerator::get_sample() {
    return distribution(generator);
}