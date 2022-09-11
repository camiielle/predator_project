#ifndef BOIDS_HPP
#define BOIDS_HPP

#include <cassert>
#include <cmath>
#include <utility>

// defines Vector2D, Position, Velocity and Boid (user-defined types)

constexpr double pi{3.14159265358979323846};
constexpr double sqrt2{1.41421356237309504880};
constexpr int simulations{100};

// Vector2D, representing the algebraic entity 'vector' in 2D Euclidean space

struct Vector2D : public std::pair<double, double>
{
  // clang-format off
  using std::pair<double, double>::pair;
  Vector2D& operator+=(Vector2D const& other);
  Vector2D& operator-=(Vector2D const& other);
  Vector2D& operator/=(double scalar);
  Vector2D& operator*=(double scalar);
  double x() const{return first;}
  double y() const{return second;}
  double& x() {return first;}
  double& y() {return second;}
};
// clang-format on
inline double norm(Vector2D const& vector)
{
  return std::sqrt(vector.x() * vector.x() + vector.y() * vector.y());
}

template<class T>
T operator+(T const& v1, T const& v2)
{
  static_assert(std::is_base_of<Vector2D, T>::value);
  return T{v1.first + v2.first, v1.second + v2.second};
}
template<class T>
T operator-(T const& v1, T const& v2)
{
  static_assert(std::is_base_of<Vector2D, T>::value);
  return T{v1.first - v2.first, v1.second - v2.second};
}
template<class T>
T operator*(T const& v, double scalar)
{
  static_assert(std::is_base_of<Vector2D, T>::value);
  assert(scalar != 0.);
  return T{v.first * scalar, v.second * scalar};
}
template<class T>
T operator/(T const& v, double scalar)
{
  static_assert(std::is_base_of<Vector2D, T>::value);
  assert(scalar != 0.);
  return T{v.first / scalar, v.second / scalar};
}

// Distinguishing between vectors with different physical meanings (i.e. vector
// position and vector velocity)
struct Position : public Vector2D
{
  using Vector2D::Vector2D;
};
struct Velocity : public Vector2D
{
  using Vector2D::Vector2D;
};

Velocity& normalize(Velocity& v, double min_speed, double max_speed);

class Boid
{
  Position p_;
  Velocity v_;
  bool is_pred_{false};
  bool is_eaten_{false};

 public:
  explicit Boid(Position p, Velocity v, bool is_pred);
  explicit Boid(Position p, Velocity v);
  // clang-format off
  Position position() const{return p_;}
  Position& position(){return p_;}
  Velocity velocity() const{return v_;}
  Velocity& velocity(){return v_;}
  // only const method defined for is_pred_ since predatory nature of a boid
  // is not meant to be modified after its creation
  bool is_pred() const{return is_pred_;}
  bool is_eaten() const{return is_eaten_;}
  bool& is_eaten(){return is_eaten_;}
};
// clang-format on

double distance(Boid const& b1, Boid const& b2);

bool is_seen(Boid const& b1, Boid const& b2, double angle_of_view);

bool in_corner(Boid const& boid, double x_max, double y_max);

void leave_corner(Boid& boid, double x_min, double x_max, double y_min,
                  double y_max);

Velocity& bound_position(Boid& b, double x_min, double x_max, double y_min,
                         double y_max);

#endif
