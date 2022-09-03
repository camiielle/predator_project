#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "boids.hpp"
#include "doctest.h"

TEST_CASE("testing Vector2D")
{
  Vector2D v1{2., 4.};
  Vector2D v1_bis = v1;
  Vector2D v2{-5., 4.};
  Vector2D v3{};
  Vector2D v4{-2., -4.};
  double s1 = 2.5;
  double s2 = -2;
  double s3 = 1.;

  SUBCASE("testing modifying-value operators")
  {
    SUBCASE("operator +=")
    {
      CHECK((v1 += v2) == Vector2D{-3., 8.}); // regular vectors
      CHECK((v2 += v3) == v2);                // null vector
      CHECK((v2 += v2) == (v2 * 2.));         // equal vectors
      CHECK((v3 += v3) == v3);                // null vectors
      CHECK((v1_bis += v4) == v3);            // opposite vectors
    }
    SUBCASE("operator -=")
    {
      CHECK((v2 -= v1) == Vector2D{-7., 0.}); // regular vectors
      CHECK((v1 -= v3) == v1);                // null vector
      CHECK((v1 -= v1) == v3);                // equal vectors
      CHECK((v1 -= v3) == v3);                // null vectors
      CHECK((v1_bis -= v4) == (v1_bis * 2.)); // opposite vectors
      CHECK((v4 -= v1_bis) == (v4 * 3.));     // parallel vectors
    }
    SUBCASE("operator /=")
    {
      CHECK((v1 /= s1) == Vector2D{.8, 1.6});  // positive value
      CHECK((v1 /= s2) == Vector2D{-.4, -.8}); // negative value
      CHECK((v2 /= s3) == v2);                 // division by 1
    }

    SUBCASE("operator *=")
    {
      CHECK((v1 *= s1) == Vector2D{5., 10.});  // positive value
      CHECK((v2 *= s2) == Vector2D{10., -8.}); // negative value
      CHECK((v2 *= s3) == v2);                 // multiplication by 1
    }
  }

  SUBCASE("testing non-modifying-value operators")
  {
    CHECK((v1 + v2) == Vector2D{-3., 8.});
    // no need for v1_bis now since v1 is not getting modified:
    CHECK((v1 + v4) == v3);
    CHECK((v1 -= v3) == v1);
    CHECK((v1 - v1) == v3);
    CHECK((v1 - v4) == (v1 * 2.));
  }

  SUBCASE("testing norm")
  {
    CHECK(norm(v2) == doctest::Approx(std::sqrt(41.)));  // regular vector
    CHECK(norm(v4) == doctest::Approx(std::sqrt(20.)));  // regular vector
    CHECK(norm(v3) == doctest::Approx(.0));              // null vector
    CHECK((norm(v1) - norm(v4)) == doctest::Approx(.0)); // opposite vectors
  }
}

// functions defined here and not exported anywhere, only to test
// that overloading is performed correctly

// clang-format off
int test_overload(Vector2D) {return 0;}
int test_overload(Position) {return 1;}
int test_overload(Velocity) {return 2;}
int testing_overload(Vector2D) {return 3;}
// clang-format on

TEST_CASE("Testing Position and Velocity")
{
  Velocity v1{4., 3.};  // oblique vector, speed=5
  Velocity v2{-3., 0.}; // horizontal vector, speed=3
  Velocity v3{0., -7.}; // vertical vector, speed=7
  Velocity v4{};        // null speed

  SUBCASE("testing overloading")
  {
    Vector2D vec{7., -3};
    Position pos{1., 1.};
    CHECK(test_overload(vec) == 0);
    CHECK(test_overload(pos) == 1);
    CHECK(test_overload(v1) == 2);
    CHECK(testing_overload(vec) == 3);
    CHECK(testing_overload(pos) == 3);
    CHECK(testing_overload(v1) == 3);
  }
  SUBCASE("testing normalize for upper limit")
  {
    CHECK(norm(normalize(v1, .5, 4.)) // speed greater than max_speed
          == doctest::Approx(3.8));
    CHECK(norm(normalize(v2, .5, 3.)) // speed equal to max_speed
          == doctest::Approx(2.85));
    CHECK(normalize(v3, .5, 7.1) == v3); // speed in range
  }
  SUBCASE("testing normalize for null velocity")
  {
    // testing with different min and max speed:
    // norm set properly
    CHECK(norm(normalize(v4, .7, 18)) == doctest::Approx(.735));
    CHECK(norm(normalize(v4, 3., 20.)) == doctest::Approx(3.15));
    // direction set properly as well
    CHECK(normalize(v4, 6., 25.) == Velocity{1., 1.} * (6. * 1.05 / sqrt2));
    CHECK(normalize(v4, 20., 40.) == Velocity{1., 1.} * (20. * 1.05 / sqrt2));
  }

  SUBCASE("testing normalize for lower limit")
  {
    CHECK(norm(normalize(v1, 6., 25.)) // speed smaller than min_speed
          == doctest::Approx(6.3));
    CHECK(norm(normalize(v2, 3., 20.)) // speed equal to min_speed
          == doctest::Approx(3.15));
    CHECK((normalize(v3, .7, 18)) == v3); // speed in range
  }
}

TEST_CASE("Testing Boid")
{
  Velocity v1{1., 2.};
  Position p1{2., 2.};
  Position p2{3., 0.};
  Position p3{0., 2.};
  Position p4{};
  Position p5 = p1 * (-1.);
  Position p6{2., -6};
  Boid b1{p1, v1};
  Boid b1_p{p1, v1, true};
  Boid b2{p2, v1};
  Boid b3{p3, v1};
  Boid b4{p4, v1};
  Boid b5{p5, v1, true};
  Boid b6{p6, v1, true};
  Boid b7{p4, Velocity{4., 4.}, true};

  SUBCASE("testing distance")
  {
    CHECK(distance(b1, b2) == doctest::Approx(std::sqrt(5.))); // regular
    CHECK(distance(b1, b4) == norm(b1.position()));            // null vector
    CHECK(distance(b1, b3) == doctest::Approx(2.)); // horizontally aligned
    CHECK(distance(b1, b6) == doctest::Approx(8.)); // vertically aligned
    CHECK(distance(b1, b1_p)
          == doctest::Approx(0.)); // same position, 1 normal boid & 1 predator
    CHECK(distance(b1, b5)
          == (2.) * norm(b1.position())); // opposite vectors, 1 normal boid & 1
                                          // predator
  }
  SUBCASE("testing angle")
  {
    // considering  other boids from b1 (regular boid) perspective
    // coincident positions, 1 regular & 1 predator
    CHECK(is_seen(b1, b1_p, 11.) == true);
    // boids in field of view
    CHECK(is_seen(b1, b2, 320.) == true);
    CHECK(is_seen(b1, b3, 320.) == true);
    CHECK(is_seen(b1, b6, 320.) == true);
    // boids out of field of view
    CHECK(is_seen(b1, b4, 320.) == false);
    CHECK(is_seen(b1, b5, 320.) == false);

    // considering  other boids from b7 (predator) perspective
    // coincident positions, 1 regular & 1 predator
    CHECK(is_seen(b7, b4, 1.) == true);
    // boids close to the limit of the viewing angle, from opposite sides
    CHECK(is_seen(b7, b2, 90.1) == true);
    CHECK(is_seen(b7, b3, 90.1) == true);
    // boids in field of view
    CHECK(is_seen(b7, b1, 90.) == true);
    CHECK(is_seen(b7, b1_p, 90.) == true);
    // boids out of field of view
    CHECK(is_seen(b7, b5, 90.) == false);
    CHECK(is_seen(b7, b6, 90.) == false);
  }
}
TEST_CASE("Testing behavior in corners' proximity")
{
  Velocity v1{3., 3.};
  Velocity v2{1., 2.};
  Velocity v3{-2., -1};
  Velocity v4{-1.5, -1.5};
  Velocity v5{-3.5, 0.};
  Velocity v6{0., 2.5};

  Position p1{11., 7.};
  Position p2{11., 3.};
  Position p3{3., -.5};
  Position p4{-1., -1.};
  Position p5{};
  Position p6{5., 3.};
  Position p7{-2., 3.};
  Position p8{2., 8.};

  Boid b1{p1, v1};
  Boid b2{p2, v2};
  Boid b3{p3, v3};
  Boid b4{p4, v4};
  Boid b5{p5, v1, true};
  Boid b6{p6, v2, true};
  Boid b7{p7, v5};
  Boid b8{p8, v6};

  double xmin{};
  double ymin{};
  double xmax{10.};
  double ymax{6.};

  SUBCASE("testing in_corner")
  {
    Boid b9{{9.5, .5}, v5};
    Boid b10{{.5, 9.5}, v4, true};
    CHECK(in_corner(b1, xmax, ymax));  // upper right corner, regular
    CHECK(in_corner(b10, xmax, ymax)); // upper left corner, predator
    CHECK(in_corner(b4, xmax, ymax));  // lower left corner, regular
    CHECK(in_corner(b5, xmax, ymax));  // lower left corner, predator
    CHECK(in_corner(b9, xmax, ymax));  // lower right corner, regular

    CHECK_FALSE(in_corner(b6, xmax, ymax)); // in the center
    CHECK_FALSE(in_corner(b2, xmax, ymax)); // crossed xmax but not in corner
    CHECK_FALSE(in_corner(b7, xmax, ymax)); // crossed xmin but not in corner
    CHECK_FALSE(in_corner(b8, xmax, ymax)); // crossed ymax but not in corner
    CHECK_FALSE(in_corner(b3, xmax, ymax)); // crossed yin but not in corner
  }

  SUBCASE("testing leave_corner")
  { // leave_corner internally asserts boid is a predator
    Boid b7_p{p7, v5, true};
    Boid b10{{.0, 5.9}, v4, true};
    leave_corner(b10, xmin, xmax, ymin, ymax); // upper left corner
    CHECK(b10.velocity().x()
          == doctest::Approx((Velocity{2.5, -2.5} * std::sqrt(2.25)).x()));
    CHECK(b10.velocity().y()
          == doctest::Approx((Velocity{2.5, -2.5} * std::sqrt(2.25)).y()));
    leave_corner(b5, xmin, xmax, ymin, ymax); // lower left corner
    CHECK(b5.velocity().x()
          == doctest::Approx((Velocity{5., 5.} * std::sqrt(2.25)).x()));
    CHECK(b5.velocity().y()
          == doctest::Approx((Velocity{5., 5.} * std::sqrt(2.25)).y()));
    leave_corner(b7_p, xmin, xmax, ymin, ymax); // crossed xmin but not in
                                                // corner
    CHECK(b7_p.velocity() == v5);
    leave_corner(b6, xmin, xmax, ymin, ymax); // in center
    CHECK(b6.velocity() == v2);
  }

  SUBCASE("testing bound position")
  {
    // using both predators and regular boids
    CHECK(
        ((bound_position(b1, xmin, xmax, ymin, ymax)).x()) // crossed 2 borders
        == doctest::Approx(3. - 1.5 * std::sqrt(18.)));
    // bound pos applied for 2nd time
    CHECK((bound_position(b1, xmin, xmax, ymin, ymax)).y()
          == doctest::Approx(-10.5));
    CHECK(
        ((bound_position(b4, xmin, xmax, ymin, ymax)).x()) // crossed 2 borders
        == doctest::Approx(-1.5 + 3 * 1.5 / sqrt2));
    CHECK(((bound_position(b4, xmin, xmax, ymin, ymax)).y())
          == doctest::Approx(1.6819805153 + 3.563818177).epsilon(0.01));
    CHECK((bound_position(b2, xmin, xmax, ymin, ymax).x()) // crossed xmax
          == doctest::Approx(1. - 1.5 * std::sqrt(5.)));
    CHECK((bound_position(b3, xmin, xmax, ymin, ymax).y()) // crossed ymin
          == doctest::Approx(-1. + 1.5 * std::sqrt(5.)));
    CHECK((bound_position(b7, xmin, xmax, ymin, ymax)).x() // crossed xmin
          == doctest::Approx(1.75));
    CHECK((bound_position(b8, xmin, xmax, ymin, ymax)).y() // crossed ymax
          == doctest::Approx(-1.25));
    CHECK((bound_position(b8, xmin, xmax, ymin, ymax)).x()
          == 0.); // checking v_x was left unchanged
    // positioned in one corner: testing leave corner is applied at the end of
    // funct, since b8 is a predator
    CHECK(bound_position(b5, xmin, xmax, ymin, ymax).x()
          == doctest::Approx(23.40990258));
    CHECK(bound_position(b6, xmin, xmax, ymin, ymax) // positioned in the center
          == v2);
  }
}