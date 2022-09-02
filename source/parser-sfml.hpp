#ifndef PARSERSFML_HPP
#define PARSERSFML_HPP

#include <lyra/lyra.hpp>
#include <string>

inline auto get_parser(double& angle, double& d, double& d_s, double& s,
                       double& c, double& a, double& max_speed,
                       double& min_speed_fraction, int& delta_t, int& fps,
                       int& N_boids, double const display_width,
                       double const display_height, bool& show_help)
{
  return lyra::cli{
      lyra::help(show_help)
      | lyra::opt(angle, "angle-of-view")["-A"]["--angle_of_view"](
          "Set angle of view - must be in range (0.,360.)  [Default value is "
          "310.]")
      | lyra::opt(d, "neighbour-distance")["-D"]["--neighbour_distance"](
          "Set neighbour distance - must be in range + (0.,"
          + std::to_string(std::min(display_width, display_height))
          + ")  [Default "
            "value "
            "is 55.]")
      | lyra::opt(d_s, "separation-distance")["-d"]["--separation_distance"](
          "Set separation distance - must be in range "
          "(0.,neighbour-distance/2)  [Default value is 10.]")
      | lyra::opt(s, "separation-factor")["-s"]["--separation_factor"](
          "Set separation factor - must be in range (0.,5.)  [Default value is "
          "4.9]")
      | lyra::opt(c, "cohesion-factor")["-c"]["--cohesion_factor"](
          "Set cohesion factor - must be in range (0.,5.)  [Default value is "
          "0.0015]")
      | lyra::opt(a, "alignment-factor")["-a"]["--alignment_factor"](
          "Set alignment factor - must be in range (0.,5.)  [Default value is "
          ".1]")
      | lyra::opt(max_speed, "maximum-speed")["-V"]["--maximum_speed"](
          "Set maximum speed - must be greater than 0.  [Default value is "
          "500.]")
      | lyra::opt(min_speed_fraction,
                  "minimum-speed-fraction")["-v"]["--minimum_speed_fraction"](
          "Set minimum speed as a fraction of maximum speed - must be in range "
          "(0.,1.)  [Default value is 0.5]")
      | lyra::opt(delta_t, "time-interval-of-evolution{ms}")["-t"]["--delta_t"](
          "Set time interval for each evolution {ms} - must be a positive "
          "integer in range (0,125) "
          "[Default value is 1]")
      | lyra::opt(fps, "fps")["-f"]["--fps"](
          "Set frames per second - must be greater than 0 and less than "
          "1000/delta_t  "
          "[Default value is 60]")
      | lyra::opt(N_boids, "number-of-boids")["-b"]["--boids"](
          "Set number of boids  - must be greater than 1  [Default value is "
          "80]")};
}

#endif