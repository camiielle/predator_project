#ifndef PARAMETERS_HPP
#define PARAMETERS_HPP

#include <SFML/Graphics.hpp>
#include <algorithm>
#include <cassert>
#include <stdexcept>
#include <string>

// defines class Parameters, whose constructor validates input, and its
// exception handling

class Invalid_Parameter : public std::runtime_error
{
  using std::runtime_error::runtime_error;
};

template<class T>
void is_in_range(T val, T val_min, T val_max, std::string par_name)
{
  if (val <= val_min || val >= val_max) {
    throw Invalid_Parameter{"Parameter " + par_name
                            + " is not in the required range"};
  }
}

template<class T>
void is_greater_than(T val, T val_min, std::string par_name)
{
  if (val <= val_min) {
    throw Invalid_Parameter{"Parameter " + par_name + " must be greater than "
                            + std::to_string(val_min)};
  }
}

class Parameters
{
  // values depending on user input:
  double angle_; // boids' angle of view
  double d_;     // neighbour distance
  double d_s_;   // separation distance
  double s_;     // separation factor
  double c_;     // cohesion factor
  double a_;     // alignment factor
  double max_speed_;
  double min_speed_;
  double duration_; // duration of the simulation{s}
  //(or time interval for each evolve, if GRAPHICS is defined)
  int steps_;           // evolve flock for [steps] times
  int prescale_or_fps_; // print flock state every [prescale] steps
  //(or frames per second, if GRAPHICS is defined)
  int prescale_or_fps_limit_;
  int N_boids_;
  double d_s_pred_; // separation distance for predators
  double s_pred_;   // separation factor for predators

  // values set by developer:
  double x_min_{0.};
  double y_min_{0.};
#ifdef GRAPHICS
  double x_max_{.7 * sf::VideoMode::getDesktopMode().width};
  double y_max_{.7 * sf::VideoMode::getDesktopMode().height};
#endif
#ifndef GRAPHICS
  double x_max_{100.};
  double y_max_{100.};
#endif

  bool invariant()
  {
    return (angle_ > 0. && angle_ < 360.)
        && (d_ > 0. && d_ < std::min(x_max_, y_max_))
        && (d_s_ > 0. && d_s_ < .5 * d_) && (s_ > 0. && s_ < 5.)
        && (c_ > 0. && c_ < 5.) && (a_ > 0. && a_ < 5.) && (max_speed_ > 0.)
        && (min_speed_ > 0. && min_speed_ < max_speed_) && (duration_ > 0.)
        && (steps_ >= 1)
        && (prescale_or_fps_ > 0 && prescale_or_fps_ < prescale_or_fps_limit_)
        && (N_boids_ > 1);
  }

 public:
  explicit Parameters(double angle, double d, double d_s, double s, double c,
                      double a, double max_speed, double min_speed_fraction,
                      double duration, int steps, int prescale_or_fps,
                      int prescale_or_fps_limit, int N_boids)
      : angle_{angle}
      , d_{d}
      , d_s_{d_s}
      , s_{s}
      , c_{c}
      , a_{a}
      , max_speed_{max_speed}
      , min_speed_{max_speed * min_speed_fraction}
      , duration_{duration}
      , steps_{steps}
      , prescale_or_fps_{prescale_or_fps}
      , prescale_or_fps_limit_{prescale_or_fps_limit}
      , N_boids_{N_boids}
      , d_s_pred_{7. * d_s} // boids' separation rule from predators has larger
      , s_pred_{10.5 * s}   // separation distance and highest separation factor

  {
    is_in_range(angle_, 0., 360., "angle-of-view");
    is_in_range(d_, 0., std::min(x_max_, y_max_), "neighbour-distance");
    // d_s has to be significantly less than d for the flock to form
    is_in_range(d_s_, 0., 0.5 * d_, "separation-distance");
    is_in_range(s_, 0., 5., "separation-factor");
    is_in_range(c_, 0., 5., "cohesion-factor");
    is_in_range(a_, 0., 5., "alignment-factor");
    is_greater_than(max_speed_, 0., "maximum-speed");
    is_in_range(min_speed_, 0., max_speed_, "minimum-speed");
#ifndef GRAPHICS
    is_greater_than(duration_, 0., "duration-of-simulation{s}");
    is_greater_than(steps_, 1, "number-of-evolutions");
    // guarantees that at least two flock's states are printed
    is_in_range(prescale_or_fps_, 0, prescale_or_fps_limit_, "prescale");
#endif
#ifdef GRAPHICS
    is_in_range(prescale_or_fps_, 0, prescale_or_fps_limit_,
                "frames-per-second");
// Steps and duration now aren't input pars, so input validation isn't
// performed. Valid values of fps (validated above) and delta_t (validated in
// main) result in valid values of duration and steps: therefore, value-checking
// of both is done through assertions.
#endif
    is_greater_than(N_boids_, 1, "number-of-boids");

    assert(invariant());
  }

  // clang-format off
  double get_angle() const{return angle_;}
  double get_d() const{return d_;}
  double get_d_s() const{return d_s_;}
  double get_s() const{return s_;}
  double get_c() const{return c_;}
  double get_a() const{return a_;}
  double get_max_speed() const{return max_speed_;}
  double get_min_speed() const{return min_speed_;}
  double get_duration() const{return duration_;}
  int get_steps() const{return steps_;}
  #ifndef GRAPHICS
  int get_prescale() const{return prescale_or_fps_;}
  #endif
  #ifdef GRAPHICS
  int get_fps() const{return prescale_or_fps_;}
  #endif
  int get_N_boids() const{return N_boids_;}
  double get_x_min() const{return x_min_;}
  double get_x_max() const{return x_max_;}
  double get_y_min() const{return y_min_;}
  double get_y_max() const{return y_max_;}
  double& set_x_max(){return x_max_;}
  double& set_y_max(){return y_max_;}
  double get_d_s_pred() const{return d_s_pred_;}
  double get_s_pred() const{return s_pred_;}
  // clang-format on
};

#endif
