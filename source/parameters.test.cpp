#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "parameters.hpp"
#include "doctest.h"

TEST_CASE("testing Parameters")
{
  SUBCASE("acceptable values don't throw")
  {
    CHECK_NOTHROW(Parameters{1., 1., .1, 1., 1., 1., 1., .1, 1., 2, 1, 2,
                             2}); // values close to lower limit
    CHECK_NOTHROW(Parameters{150., 40., 8.5, 1., 1.5, 2., 155., .2, 85., 1000,
                             10, 1000, 500}); // values close to center of range
    CHECK_NOTHROW(Parameters{359., 99., 45., 4.8, 4.4, 4.7, 500., .9,
                             1000., // values close to higher limit
                             1000, 500, 1000, 1000});
  }

  SUBCASE("limit values throw") // range of definition is open: value cannot
                                // equal the limits of the interval.
  {
    CHECK_THROWS_WITH((Parameters{360., 40., 8.5, 1., 1.5, 2., 155., .2, 85.,
                                  1000, 10, 1000, 500}),
                      ("Parameter angle-of-view is not in the required range"));
    CHECK_THROWS_WITH(
        (Parameters{150., 0., 8.5, 1., 1.5, 2., 155., .2, 85., 1000, 10, 1000,
                    500}),
        ("Parameter neighbour-distance is not in the required range"));
    CHECK_THROWS_WITH(
        (Parameters{150., 40., 40., 1., 1.5, 2., 155., .2, 85., 1000, 10, 1000,
                    500}),
        ("Parameter separation-distance is not in the required range"));
    CHECK_THROWS_WITH(
        (Parameters{150., 40., 8.5, 0., 1.5, 2., 155., .2, 85., 1000, 10, 1000,
                    500}),
        ("Parameter separation-factor is not in the required range"));
    CHECK_THROWS_WITH(
        (Parameters{150., 40., 8.5, 1., 5., 2., 155., .2, 85., 1000, 10, 1000,
                    500}),
        ("Parameter cohesion-factor is not in the required range"));
    CHECK_THROWS_WITH(
        (Parameters{150., 40., 8.5, 1., 1.5, 0., 155., .2, 85., 1000, 10, 1000,
                    500}),
        ("Parameter alignment-factor is not in the required range"));
    CHECK_THROWS_WITH(
        (Parameters{150., 40., 8.5, 1., 1.5, 2., 0., .2, 85., 1000, 10, 1000,
                    500}),
        ("Parameter maximum-speed must be greater than 0.000000"));
    CHECK_THROWS_WITH((Parameters{150., 40., 8.5, 1., 1.5, 2., 155., 0., 85.,
                                  1000, 10, 1000, 500}),
                      ("Parameter minimum-speed is not in the required range"));
    CHECK_THROWS_WITH(
        (Parameters{150., 40., 8.5, 1., 1.5, 2., 155., .2, 0., 1000, 10, 1000,
                    500}),
        ("Parameter duration-of-simulation{s} must be greater than 0.000000"));
    CHECK_THROWS_WITH(
        (Parameters{150., 40., 8.5, 1., 1.5, 2., 155., .2, 85., 1, 10, 1, 500}),
        ("Parameter number-of-evolutions must be greater than 1"));
    CHECK_THROWS_WITH((Parameters{150., 40., 8.5, 1., 1.5, 2., 155., .2, 85.,
                                  1000, 1200, 1000, 500}),
                      ("Parameter prescale is not in the required range"));
    CHECK_THROWS_WITH((Parameters{150., 40., 8.5, 1., 1.5, 2., 155., .2, 85.,
                                  1000, 10, 1000, 1}),
                      ("Parameter number-of-boids must be greater than 1"));
  }

  SUBCASE("out of range values throw")
  {
    CHECK_THROWS_AS(
        (Parameters{-1., 1., .1, 1., 1., 1., 1., .1, 1., 2, 1, 2, 2}),
        Invalid_Parameter); // one negative value
    CHECK_THROWS_AS(
        (Parameters{1., 1., 11, 1., 1., 1., 1., .1, 1., 2, 1, 2, 2}),
        Invalid_Parameter); // one value greater than superior limit

    // mixing negative, limit and greater-than-superior-limit values, checking
    // that constructor's statements are executed in order, as expected
    CHECK_THROWS_WITH((Parameters{150., 40., 8.5, -9.5, 1.5, 2., 155., .2, -10.,
                                  1000, 1200, 1000, 1}),
                      ("Parameter separation-factor is not in the required "
                       "range"));
    CHECK_THROWS_WITH(
        (Parameters{150., 40., 8.5, 1., 1.5, 2., 155., .2, 0., -5, 10, -5, 0}),
        ("Parameter duration-of-simulation{s} must be greater than 0.000000"));

    // all negative values
    CHECK_THROWS_WITH((Parameters{-1., -11., -17., -4.8, -.5, -.7, -500., -.9,
                                  -19., -22, -5, -22, -20}),
                      ("Parameter angle-of-view is not in the required range"));
  }
}
