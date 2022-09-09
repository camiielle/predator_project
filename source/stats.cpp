#include "stats.hpp"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <numeric>
#include <sstream>

// defines functions for analyzing, printing and saving data

void write_counter(std::array<double, simulations> const& preys_eaten,
                   int const seek_type)
{
  std::ofstream os{"preys_eaten_counter.txt"}; // opens file for writing
  if (!os) {
    throw std::ios_base::failure{
        "ERROR: Cannot open file preys_eaten_counter.txt\n"};
  }

  assert(seek_type == 0 || seek_type == 1 || seek_type == 2);
  switch (seek_type) {
  case 0:
    os << "attack nearest prey\n";
    break;
  case 1:
    os << " attack most isolated prey\n";
    break;
  case 2:
    os << "attack center of mass\n";
    break;
  default:
    break;
  }

  for (auto const& count : preys_eaten) {
    os << count << '\n';
  }
  std::cout << "\nSUCCESS! Data have been saved to file preys_eaten_counter.txt "
               "in current directory\n";
}