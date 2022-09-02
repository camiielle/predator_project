#include "boids.hpp"

// defines Vector2D's operators, function normalize, Boid's constructors and
// auxiliary functions of the main flying rules taking one or more boids as
// arguments

// Vector2D's overloaded operators
Vector2D& Vector2D::operator+=(Vector2D const& other)
{
  first += other.first;
  second += other.second;
  return *this;
}
Vector2D& Vector2D::operator-=(Vector2D const& other)
{
  first -= other.first;
  second -= other.second;
  return *this;
}

Vector2D& Vector2D::operator/=(double scalar)
{
  assert(scalar != 0.);
  first /= scalar;
  second /= scalar;
  return *this;
}

Vector2D& Vector2D::operator*=(double scalar)
{
  assert(scalar != 0.);
  first *= scalar;
  second *= scalar;
  return *this;
}

// keeping speed in the allowed limits (speed modified, direction unaltered)
Velocity& normalize(Velocity& v, double min_speed, double max_speed)
{
  if (norm(v) >= max_speed) { // sets new speed a little below the max
    v *= (0.95 * max_speed / norm(v));
  }
  if (norm(v) == 0.) { // sets direction, sets new speed a little above the min
    v = Velocity{1., 1.} * (1.05 * min_speed / sqrt2);
  }
  if (norm(v) <= min_speed) { // sets new speed a little above the min
    v *= 1.05 * min_speed / norm(v);
  }
  assert(norm(v) > min_speed && norm(v) < max_speed);
  return v;
}

// Boid constructor overloading: 1st one will be used to construct predators,
// 2nd for regular boids
Boid::Boid(Position p, Velocity v, bool is_pred)
    : p_{p}
    , v_{v}
    , is_pred_{is_pred}
{
  assert(is_pred_);
}
Boid::Boid(Position p, Velocity v)
    : p_{p}
    , v_{v}
{
  assert(!is_pred_);
}

double distance(Boid const& b1, Boid const& b2)
{
  double xdiff{b1.position().x() - b2.position().x()};
  double ydiff{b1.position().y() - b2.position().y()};
  return std::sqrt(xdiff * xdiff + ydiff * ydiff);
}

// returns true if boid 1 can see boid 2, false otherwise
bool is_seen(Boid const& b1, Boid const& b2, double angle_of_view)
{
  // if boids' positions coincide, they always see each other (must be handled
  // separately, since angle between a null vector and a vector is undefined)
  if (b1.position() == b2.position()) {
    return true;
  }
  // calculates angle in range [0 , π] between b1's velocity b1 and difference
  // of positions between b2 and b1
  auto pos_diff{b2.position() - b1.position()};
  double scalar_prod{pos_diff.x() * b1.velocity().x()
                     + pos_diff.y() * b1.velocity().y()};
  double cos{(scalar_prod / (norm(b1.velocity()) * norm(pos_diff)))};
  if (cos
      >= std::cos(pi * angle_of_view
                  / 360.)) // converting half the angle-of-view into radiants
  {
    return true;
  } else {
    return false;
  }
}

// auxiliary function returning true if boid is in one of the 4 corners
bool in_corner(Boid const& boid, double x_max, double y_max)
{
  return (boid.position().x() < .1 * x_max && boid.position().y() < .1 * y_max)
      || (boid.position().x() < .1 * x_max && boid.position().y() > .9 * y_max)
      || (boid.position().x() > .9 * x_max && boid.position().y() > .9 * y_max)
      || (boid.position().x() > .9 * x_max && boid.position().y() < .1 * y_max);
}

// auxiliary function called at the end of bound_position. Sets predator's
// velocity away from the corners, since they represent regular birds' refuge
void leave_corner(Boid& boid, double x_min, double x_max, double y_min,
                  double y_max)
{
  assert(boid.is_pred());

  if (boid.position().x() < x_min + .045 * x_max
      && boid.position().y() < y_min + .045 * y_max) {
    boid.velocity() = Velocity{1., 1.} * 2.5 * norm(boid.velocity()) / sqrt2;
  }
  if (boid.position().x() < x_min + .045 * x_max
      && boid.position().y() > y_max - .045 * y_max) {
    boid.velocity() = Velocity{1., -1.} * 2.5 * norm(boid.velocity()) / sqrt2;
  }
  if (boid.position().x() > x_max - .045 * x_max
      && boid.position().y() > y_max - .045 * y_max) {
    boid.velocity() = Velocity{-1., -1.} * 2.5 * norm(boid.velocity()) / sqrt2;
  }
  if (boid.position().x() > x_max - .045 * x_max
      && boid.position().y() < y_min + .045 * y_max) {
    boid.velocity() = Velocity{-1., -1.} * 2.5 * norm(boid.velocity()) / sqrt2;
  }
}

// encourages the boid to stay within rough boundaries in order to keep the
// flock on screen. Comes into play when boid either crosses border or comes
// really close to it
Velocity& bound_position(Boid& boid, double x_min, double x_max, double y_min,
                         double y_max)
{
  double norm_v{norm(boid.velocity())};
  // ifs are not mutually exclusive: a boid could have crossed both the x and
  // y border
  if (boid.position().x() < x_min + 0.015 * x_max) {
    boid.velocity().x() += norm_v * 1.5;
  }
  if (boid.position().x() > x_max - 0.015 * x_max) {
    boid.velocity().x() -= norm_v * 1.5;
  }
  if (boid.position().y() < y_min + 0.015 * y_max) {
    boid.velocity().y() += norm_v * 1.5;
  }
  if (boid.position().y() > y_max - 0.015 * y_max) {
    boid.velocity().y() -= norm_v * 1.5;
  }
  if (boid.is_pred()) {
    leave_corner(boid, x_min, x_max, y_min, y_max);
  }
  return boid.velocity();
}
