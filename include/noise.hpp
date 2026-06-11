#ifndef NOISE_HPP
#define NOISE_HPP
#include <random>

class NoiseGenerator {
public:
    NoiseGenerator(double mean, double std_dev);
    double get_sample();

private:
    std::mt19937 generator;
    std::normal_distribution<double> distribution;
};

#endif