#ifndef PARSER_HPP
#define PARSER_HPP

#include "parameters.hpp"
#include <lyra/lyra.hpp>
#include <iomanip>
#include <iostream>

inline auto get_parser(double& angle, double& d, double& d_s, double& s,
                       double& c, double& a, double& max_speed,
                       double& min_speed_fraction, double& duration, int& steps,
                       int& prescale, int& N_boids, int& N_preds,
                       bool& show_help, int& seek_type)
{
  return lyra::cli{
      lyra::help(show_help)
      | lyra::opt(angle, "angle-of-view")["-A"]["--angle_of_view"](
          "Set angle of view - must be in range (0.,360.)  [Default value is "
          "300.]")
      | lyra::opt(d, "neighbour-distance")["-D"]["--neighbour_distance"](
          "Set neighbour distance - must be in range + (0.,100.) [Default "
          "value "
          "is 35.]")
      | lyra::opt(d_s, "separation-distance")["-d"]["--separation_distance"](
          "Set separation distance - must be in range "
          "(0.,neighbour-distance/2)  [Default value is 3.5]")
      | lyra::opt(s, "separation-factor")["-s"]["--separation_factor"](
          "Set separation factor - must be in range (0.,5.)  [Default value is "
          ".7]")
      | lyra::opt(c, "cohesion-factor")["-c"]["--cohesion_factor"](
          "Set cohesion factor - must be in range (0.,5.)  [Default value is "
          "0.045]")
      | lyra::opt(a, "alignment-factor")["-a"]["--alignment_factor"](
          "Set alignment factor - must be in range (0.,5.)  [Default value is "
          ".8]")
      | lyra::opt(max_speed, "maximum-speed")["-V"]["--maximum_speed"](
          "Set maximum speed - must be greater than 0.  [Default value is 80.]")
      | lyra::opt(min_speed_fraction,
                  "minimum-speed-fraction")["-v"]["--minimum_speed_fraction"](
          "Set minimum speed as a fraction of maximum speed - must be in range "
          "(0.,1.)  [Default value is 0.05]")
      | lyra::opt(duration, "duration-of-simulation{s}")["-t"]["--duration"](
          "Set duration of the simulation{s} - must be greater than 0.  "
          "[Default value is 30.]")
      | lyra::opt(steps, "number-of-evolutions")["-S"]["--steps"](
          "Set number of steps to perform evolution - must be greater than 1  "
          "[Default value is 3000]")
      | lyra::opt(prescale, "prescale")["-p"]["--prescale"](
          "Print data every [prescale] steps - must be in range (0, steps)  "
          "[Default value is 40]")
      | lyra::opt(N_boids, "number-of-boids")["-b"]["--boids"](
          "Set number of boids  - must be greater than 1  [Default value is "
          "120]")
      | lyra::opt(N_preds, "number-of-predators")["-P"]["--preds"](
          "Set number of predators  - must be greater than 0  [Default value "
          "is "
          "1]")
      | lyra::opt(seek_type, "seek-type")["--seek-type"](
          "Set the seek type  [Default value is "
          "0]")};
}

// prints summary of values of parameters used in the simulation
inline void print_parameters(Parameters const& pars)
{
  std::cout << std::setfill(' ');
  std::cout << std::setprecision(3) << std::fixed << std::setw(15)
            << "angle A:  " << std::setw(7) << pars.get_angle() << std::setw(20)
            << "distance d: " << std::setw(10) << pars.get_d() << '\n'
            << std::setw(15) << "sep-dist d_s:  " << std::setw(7)
            << pars.get_d_s() << std::setw(20)
            << "sep-fact s: " << std::setw(10) << pars.get_s() << '\n'
            << std::setw(15) << "coe-fact c:  " << std::setw(7) << pars.get_c()
            << std::setw(20) << "align-fact a: " << std::setw(10)
            << pars.get_a() << '\n'
            << std::setw(15) << "max-sp V:  " << std::setw(7)
            << pars.get_max_speed() << std::setw(20)
            << "min-sp-fr v: " << std::setw(10) << std::setprecision(6)
            << pars.get_min_speed() / pars.get_max_speed() << '\n'
            << std::setprecision(3) << std::setw(15)
            << "duration t: " << std::setw(7) << pars.get_duration()
            << std::setw(20) << "steps S: " << std::setw(10) << pars.get_steps()
            << '\n'
            << std::setw(15) << "presc p:  " << std::setw(7)
            << pars.get_prescale() << std::setw(20)
            << "N_boids N: " << std::setw(10) << pars.get_N_boids() << '\n'
            << std::setw(15) << "N_preds P:  " << std::setw(7)
            << pars.get_N_preds() << std::setw(20)
            << "seek-type: " << std::setw(10) << pars.get_seek_type() << "\n\n";
}

#endif