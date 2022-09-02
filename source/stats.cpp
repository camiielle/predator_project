#include "stats.hpp"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <numeric>
#include <sstream>

// defines functions for analyzing, printing and saving data

// auxiliary function, calculates sum of pair distances between boid and all
// others after it in the vector
double sum_distances(Boid const& boid, std::vector<Boid> const& state, int N)
{
  return std::transform_reduce(
      &boid, &(*state.begin()) + N, 0., std::plus<>{},
      [&](Boid const& other) { return distance(boid, other); });
};

// same as above but squares each pair distance
double sum_sq_distances(Boid const& boid, std::vector<Boid> const& state, int N)
{
  return std::transform_reduce(
      &boid, &(*state.begin()) + N, 0., std::plus<>{}, [&](Boid const& other) {
        return distance(boid, other) * distance(boid, other);
      });
};

// returns mean distance between boids with its std_dev
Result mean_dist(std::vector<Boid> const& state)
{
  int N{static_cast<int>(state.size())};
  assert(N > 1);

  // applies sum_distances to each boid in state and sums the results
  double sum_dist{std::transform_reduce(
      state.begin(), state.end(), 0., std::plus<>{},
      [&](Boid const& boid) { return sum_distances(boid, state, N); })};

  // same as above but with sum_sq_distances
  double sum_sq_dist{std::transform_reduce(
      state.begin(), state.end(), 0., std::plus<>{},
      [&](Boid const& boid) { return sum_sq_distances(boid, state, N); })};

  // number of pair distances, calculated as sum of first N-1 numbers
  int n{((N * (N - 1)) / 2)};

  double mean_dist{sum_dist / n};
  double mean_sq_dist{sum_sq_dist / n};
  double std_dev{
      std::sqrt(n / (n - 1.) * (mean_sq_dist - mean_dist * mean_dist))};

  return {mean_dist, std_dev};
}

// returns mean speed of boids with its std_dev
Result mean_speed(std::vector<Boid> const& state)
{
  int N{static_cast<int>(state.size())};
  assert(N > 1);

  auto sum_speed{std::transform_reduce(
      state.begin(), state.end(), 0., std::plus<>{},
      [](Boid const& boid) { return norm(boid.velocity()); })};

  auto sum_sq_speed{std::transform_reduce(
      state.begin(), state.end(), 0., std::plus<>{}, [](Boid const& boid) {
        return (norm(boid.velocity()) * norm(boid.velocity()));
      })};

  double mean_speed{sum_speed / N};
  double mean_sq_speed{sum_sq_speed / N};
  double std_dev{
      std::sqrt(N / (N - 1.) * (mean_sq_speed - mean_speed * mean_speed))};

  return {mean_speed, std_dev};
}

// prints calculated data to standard output
void print_state(std::vector<Boid> const& state)
{
  Result distance{mean_dist(state)};
  Result speed{mean_speed(state)};

  std::cout << std::setprecision(3) << std::fixed << std::setw(8)
            << distance.mean << " \u00b1 " << std::setw(7) << distance.std_dev
            << std::setw(8) << '|' << std::setw(13) << speed.mean << " \u00b1 "
            << std::setw(7) << speed.std_dev << '\n';
}

// writes data obtained from the analysis to file indicated by user
void write_data(std::vector<std::vector<Boid>> const& states)
{
  std::cout << "\nPlease write name of file data will be saved in, then "
               "press ENTER to continue. (.txt "
               "extension added automatically).\n";
  std::string filename;
  std::getline(std::cin, filename);    // reads until the first newline
  std::ofstream os{filename + ".txt"}; // opens file for writing
  if (!os) {
    throw std::ios_base::failure{"ERROR: Cannot open file " + filename
                                 + ".txt\n"};
  }
  std::ostringstream data;
  for (auto const& state : states) {
    Result distance{mean_dist(state)};
    Result speed{mean_speed(state)};
    data << std::setprecision(3) << std::fixed << std::setw(9) << distance.mean
         << std::setw(9) << distance.std_dev << std::setw(9) << speed.mean
         << std::setw(9) << speed.std_dev << std::endl;
  }
  os << data.str();
  std::cout << "SUCCESS! Data have been saved to file: " + filename
                   + ".txt in current directory\n";
}
