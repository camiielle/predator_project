#ifndef STATS_HPP
#define STATS_HPP
#include "flock.hpp"
#include <iomanip>
#include <iostream>

struct Result
{
  double mean;
  double std_dev;
};

Result mean_dist(std::vector<Boid> const& state);

Result mean_speed(std::vector<Boid> const& state);

void print_state(std::vector<Boid> const& state);

void write_data(std::vector<std::vector<Boid>> const& states);

#endif