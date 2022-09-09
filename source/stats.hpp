#ifndef STATS_HPP
#define STATS_HPP
#include "flock.hpp"
#include <iomanip>
#include <iostream>

void write_counter(std::array<double, simulations> const& preys_eaten,
                   int const seek_type);

#endif