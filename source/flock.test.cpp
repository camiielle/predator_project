#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "flock.hpp"
#include "doctest.h"
#include "parameters.hpp"
#include <random>

TEST_CASE("testing rules' auxiliary functions")
{
  Velocity v1{0., 1.};
  Velocity v2{-3., 6.};
  Position p1{};
  Position p2{1., 1.};
  Position p3{5., 0.};

  Boid b1{p1, v1};               //[from b1's perspective]:
  Boid b2{p2, v2};               // in distance and angle, regular
  Boid b2_p{p2, v2, true};       // in distance and angle, predator
  Boid b3{p2 * -1., v2};         // in distance but not angle
  Boid b4{p2 * -4., v2};         // not distance, not angle
  Boid b4_p{p2 * -4., v2, true}; // not distance, not angle
  Boid b5{p2 * 5., v1};          // not distance but angle
  Boid b5_p{p2 * 5., v1, true};  // not distance but angle
  Boid b6{p1, v2};               // position coincides, regular
  Boid b6_p{p1, v2, true};       // position coincides, predator
  Boid b7{p3, v1};               // distance coinciding with d

  std::vector<Boid> boids{b1, b2, b2_p, b3, b4, b4_p, b5, b5_p, b6, b6_p, b7};
  Flock flock{boids};

  SUBCASE("testing neighbors")
  { // d is 2.
    std::vector<Boid> nbrs;
    CHECK((neighbours(b1, flock, nbrs, 180., 2.)).size()
          == 3u);                               // all regular neighbours copied
    CHECK(nbrs[0].velocity() == b1.velocity()); // b1 itself was copied
    CHECK(nbrs[0].position() == b1.position());
    CHECK_FALSE(nbrs[0].is_pred());
    CHECK(nbrs[1].velocity()
          == b2.velocity()); // regular got copied, predator didn't
    CHECK(nbrs[1].position() == b2.position());
    CHECK_FALSE(nbrs[0].is_pred());
    CHECK(nbrs[2].velocity()
          == b6.velocity()); // regular got copied, predator didn't
    CHECK(nbrs[2].position() == b6.position());
    CHECK_FALSE((nbrs[0].is_pred()));
    nbrs.clear();
    CHECK((neighbours(b7, flock, nbrs, 180., 2.)).size()
          == 1u); // regular with no neighbours
  }

  SUBCASE("testing predators")
  { // d_s_pred is 2.
    std::vector<Boid> preds;
    CHECK((predators(b1, flock, preds, 180., 2.)).size()
          == 2u);             // all predators copied
    CHECK(preds[0].velocity() // predator was copied, regular didn't
          == b2_p.velocity());
    CHECK(preds[0].position() == b2_p.position());
    CHECK(preds[0].is_pred());
    CHECK(preds[1].velocity()
          == b6_p.velocity()); // predator was copied, regular didn't
    CHECK(preds[1].position() == b6_p.position());
    CHECK((preds[1].is_pred()));
    preds.clear();
    CHECK((predators(b7, flock, preds, 180., .5)).size()
          == 0u); // regular with no predators
  }

  SUBCASE("testing competitors")
  { // d_s is 6.
    std::vector<Boid> comps;
    CHECK((competitors(b6_p, flock, comps, 240., 6.)).size()
          == 3u); // all competitors were copied
    CHECK(comps[0].velocity()
          == b2_p.velocity()); // predator got copied, regular didn't
    CHECK(comps[0].position() == b2_p.position());
    CHECK(comps[0].is_pred());
    CHECK(comps[1].velocity()
          == b4_p.velocity()); // predator got copied, regular didn't
    CHECK(comps[1].position() == b4_p.position());
    CHECK(comps[1].is_pred());
    CHECK(comps[2].velocity() == b6_p.velocity()); // b6_p itself got copied
    CHECK(comps[2].position() == b6_p.position());
    CHECK(comps[2].is_pred());
    comps.clear();
    CHECK((competitors(b4_p, flock, comps, 240., .5)).size()
          == 1u); // predator with no competitors
  }

  SUBCASE("testing find_prey")
  {
    boids.erase((boids.end()) - 3);   // erasing b6
    boids.erase((boids.begin()) + 6); // erasing b5
    Flock flock1{boids};
    Boid prey{
        find_prey(b6_p, flock1, 270.)}; // predator with coincident regular boid
    CHECK_FALSE(prey.is_pred());
    CHECK(prey.position() == b1.position());
    CHECK(prey.velocity() == b1.velocity());

    Boid prey1{find_prey(b5_p, flock1, 180.)}; // no boids at all in sight
    CHECK(prey1.is_pred());
    CHECK(prey1.position() == b5_p.position());
    CHECK(prey1.velocity() == b5_p.velocity());

    Boid b8_p{p2 * 2., Velocity{1., 1.}, true};
    flock1.push_back(b8_p);
    Boid prey2{find_prey(b8_p, flock1, 90.)}; // only predators in sight
    CHECK(prey2.is_pred());
    CHECK(prey2.position() == b8_p.position());
    CHECK(prey2.velocity() == b8_p.velocity());

    Boid b1_p{p1, v1, true};
    boids[0] = b1_p; // replaced b1 with b1_p
    Flock flock2{boids};
    Boid prey3{find_prey(
        b1_p, flock2,
        185.)}; // predators and regular in sight, one pred and reg coincide
    CHECK_FALSE(prey3.is_pred()); // between coinciding regular and pred,
                                  // regular was picked
    CHECK(prey3.position() == b2.position());
    CHECK(prey3.velocity() == b2.velocity());

    Boid b9_p{Position{.5, -.5}, v1 * (-1.), true};
    flock2.push_back(b9_p);
    Boid prey4{find_prey(b9_p, flock2,
                         220.)}; // predators and regulars in sight, closer
                                 // regular NOT coinciding with any predator
    CHECK_FALSE(prey4.is_pred());
    CHECK(prey4.position() == b3.position());
    CHECK(prey4.velocity() == b3.velocity());
  }
}

TEST_CASE("Testing flying rules")
{
  Parameters const pars{190.,    5.,  2.,   1., 1.,   1., 100,
                        .000005, 30., 3000, 60, 3000, 120};
  // d_s_pred is 7 times greater than d_s
  Boid b1_p{{10., 12.}, {2., 4.}, true};
  Boid b2{{-8., -12.}, {-2., -4.}};
  Boid b3_p{{8., 10.}, {2., 2.}, true};
  Boid b4_p{{6., 8.}, {4., 2.}, true};
  Boid b5{{8., 8.}, {0., 2.}};
  Boid b6_p{{-4., -10.}, {-4., -2.}, true};
  Boid b7{{8., 7.}, {0., 2.}};
  Boid b8{{6.5, 7.}, {0., -6.}};
  Boid b9_p{{10., 11.}, {1., 1.}, true};
  Flock flock{std::vector<Boid>{b1_p, b2, b3_p, b4_p, b5, b6_p, b7, b8, b9_p}};

  SUBCASE("testing separation")
  {
    // pred with no competitors nor preys
    CHECK(separation(b1_p, flock, pars) == Velocity{0., 0.});
    // regular with no predators nor close neighbours
    CHECK(separation(b2, flock, pars) == Velocity{0., 0.});
    // pred with no prey, no competitors
    CHECK(separation(b3_p, flock, pars) == Velocity{0., 0.});
    // pred with prey, no competitor
    CHECK(separation(b4_p, flock, pars) == Velocity{0., 0.});
    // regular with 4 predators, no close neighbours
    CHECK(separation(b5, flock, pars)
          == Velocity{2., 9.} * (-pars.get_s_pred()));
    // pred with prey, no competitor
    CHECK(separation(b6_p, flock, pars) == Velocity{0., 0.});
    // regular with 3 predators, 2 close neighbours
    CHECK(separation(b7, flock, pars)
          == Velocity{2., 13.} * (-pars.get_s_pred())
                 + Velocity{-1.5, 1.} * (-pars.get_s()));
    // regular with no preds, 1 close neighbour
    CHECK(separation(b8, flock, pars) == Velocity{1.5, 0.} * (-pars.get_s()));
    // pred with no prey, 1 competitor
    CHECK(separation(b9_p, flock, pars) == Velocity{0., 1.} * (-pars.get_s()));
  }

  SUBCASE("testing cohesion")
  {
    Boid b10{{7.5, 6.5}, {-1., 1.}};
    Boid b11{{-7., -10.5}, {-3., 0.}};
    flock.push_back(b10);
    flock.push_back(b11);
    // regular, no neighbours
    CHECK((cohesion(b2, flock, pars)) == Velocity{0., 0.});
    // regular, no neighbours (the 4 predators are not affecting cohesion)
    CHECK((cohesion(b5, flock, pars)) == Velocity{0., 0.});
    // regular, 2 neighbours
    CHECK((cohesion(b8, flock, pars)) == Velocity{1.25, -.25});
    // regular, 2 neighbours
    CHECK((cohesion(b7, flock, pars)) == Velocity{-.75, .5});
    // regular, 3 neighbours
    CHECK((cohesion(b10, flock, pars)).x() == 0.);
    // regular, 3 neighbours
    CHECK((cohesion(b10, flock, pars)).y() == doctest::Approx(5. / 6.));
    // regular, 1 neighbour
    CHECK((cohesion(b11, flock, pars)) == Velocity{-1., -1.5});
  }

  SUBCASE("testing seek")
  {
    CHECK(seek(b1_p, flock, pars) == Velocity{0., 0.}); // predator with no prey
    CHECK(seek(b3_p, flock, pars) == Velocity{0., 0.}); // predator with no prey
    // pred with prey (3 reg in sight) but prey in corner
    CHECK(seek(b4_p, flock, pars) == Velocity{0., 0.});
    // pred with prey (only 1 regular in sight), but prey in corner
    CHECK(seek(b6_p, flock, pars) == Velocity{0., 0.});
    Boid b10{{12., 13.}, {-2., -2}};
    flock.push_back(b10);
    // predator with prey (not in corner) 1 reg in sight, case of norm(vel)==0.
    CHECK(seek(b9_p, flock, pars) == Velocity{0., 0.});
    // predator with prey (close to, but not in corner) 1 reg in sight
    Boid b11{{11., 12.}, {-2., -2}};
    flock.push_back(b11);
    CHECK(seek(b9_p, flock, pars).x()
          == doctest::Approx(-1 / std::sqrt(5000.)));
    CHECK(seek(b9_p, flock, pars).y()
          == doctest::Approx(-1 / std::sqrt(5000.)));
    // pred with one prey (not in corner) and no competitors
    Boid b11_p{{50., 50.}, {2., 0.}, true};
    Boid b12{{52., 52.}, {0., 1.}};
    flock.push_back(b11_p);
    flock.push_back(b12);
    CHECK(
        seek(b11_p, flock, pars).x()
        == doctest::Approx((Velocity{2., 3.} / std::sqrt((13. * 312.5))).x()));
    CHECK(
        seek(b11_p, flock, pars).y()
        == doctest::Approx((Velocity{2., 3.} / std::sqrt((13. * 312.5))).y()));
  }
}

TEST_CASE("Testing evolve")
{
  Parameters const pars{300.,    3.,  1.,   2., .5,   1., 100.,
                        .000005, 30., 3000, 60, 3000, 100};
  Boid b1{{}, {1., 1.}};
  Boid b2{{8., 10.}, {1., 2.}};
  Boid b3{{13., 3.}, {-1., 3.}};
  Boid b4{{2., 5.}, {0., 3.}};
  Boid b5_p{{5., 2.}, {.5, 1.}, true};
  Boid b6_p{{6., 8.}, {2., -1.}, true};
  Boid b7_p{{4., 16.}, {1.5, 1.5}, true};
  double d_t{pars.get_duration() / pars.get_steps()};

  SUBCASE("4 regulars (no neighbours), all valid")
  {
    std::vector<Boid> boids{b1, b2, b3, b4};
    Flock flock{boids};
    flock.evolve(pars);
    CHECK(flock.state().size() == 4); // size left unchanged
    // is_pred attribute left unchanged
    CHECK_FALSE(flock.state()[0].is_pred());
    CHECK_FALSE(flock.state()[1].is_pred());
    CHECK_FALSE(flock.state()[2].is_pred());
    CHECK_FALSE(flock.state()[3].is_pred());
    // order unchanged, positions modified only by the motion of
    // boid itself, velocity only changed by bound_position
    CHECK(flock.state()[0].position().x()
          == b1.position().x() + b1.velocity().x() * d_t);
    CHECK(flock.state()[0].position().y()
          == b1.position().y() + b1.velocity().y() * d_t);
    // bound_position applied to b1
    CHECK(flock.state()[0].velocity().x() == doctest::Approx(1.5 * sqrt2 + 1));
    CHECK(flock.state()[0].velocity().y() == doctest::Approx(1.5 * sqrt2 + 1));

    CHECK(flock.state()[1].position().x()
          == b2.position().x() + b2.velocity().x() * d_t);
    CHECK(flock.state()[1].position().y()
          == b2.position().y() + b2.velocity().y() * d_t);
    CHECK(flock.state()[1].velocity() == b2.velocity());

    CHECK(flock.state()[2].position().x()
          == b3.position().x() + b3.velocity().x() * d_t);
    CHECK(flock.state()[2].position().y()
          == b3.position().y() + b3.velocity().y() * d_t);
    CHECK(flock.state()[2].velocity() == b3.velocity());

    CHECK(flock.state()[3].position().x()
          == b4.position().x() + b4.velocity().x() * d_t);
    CHECK(flock.state()[3].position().y()
          == b4.position().y() + b4.velocity().y() * d_t);
    CHECK(flock.state()[3].velocity() == b4.velocity());
  }

  SUBCASE("5 regulars (no neighbours), 2 go out of grid after evolution")
  { // testing that bound_position is applied within solve when evolve is called
    b1.velocity() = {-1., 1.}; // b1 will cross x_min and get too close to y_min
    Boid b5{{4., .1}, {0., -15.}}; // b5 will cross y_min
    std::vector<Boid> boids{b1, b2, b3, b4, b5};
    Flock flock{boids};
    flock.evolve(pars);

    CHECK(flock.state()[0].velocity().x() == b1.velocity().x() + 1.5 * sqrt2);
    CHECK(flock.state()[0].velocity().y() == b1.velocity().y() + 1.5 * sqrt2);

    CHECK(flock.state()[4].velocity().y() == b5.velocity().y() + 22.5);
    CHECK(flock.state()[4].velocity().x()
          == b5.velocity().x()); // v_x unaltered

    // velocities were modified in accordance to bound_position's formula
  }

  SUBCASE("4 regulars (no neighbours), one crosses x_min and its speed breaks "
          "limits")
  { // testing that normalize is applied, after bound_position, within solve
    // when evolve is called
    b4.position() = {.001, 5.};
    b4.velocity() = {-3., 4.};
    Parameters const pars1{270.,    3.,  1.,   2., .5,   1., 5.5,
                           .000005, 30., 3000, 60, 3000, 100};
    std::vector<Boid> boids{b1, b2, b3, b4};
    Flock flock{boids};
    flock.evolve(pars1);
    CHECK(flock.state()[3].velocity().x()
          == doctest::Approx(24.75 * .95 / std::sqrt(36.25)));
    CHECK(flock.state()[3].velocity().y()
          == doctest::Approx(22. * .95 / std::sqrt(36.25)));
  }

  SUBCASE("6 regulars (no close neighbours), some have neighbours")
  {
    Boid b5{{2., 3.}, {1., 2.}};
    Boid b6{{1., 1.}, {0., 3.}};
    Boid b7{{2.5, 4.}, {-1., 1.}};
    std::vector<Boid> boids{b1, b2, b3, b4, b5, b6, b7};
    Flock flock{boids};
    flock.evolve(pars);
    CHECK(flock.state().size() == 7); // size left unchanged
    // is_pred attribute left unchanged
    CHECK_FALSE(flock.state()[0].is_pred());
    CHECK_FALSE(flock.state()[1].is_pred());
    CHECK_FALSE(flock.state()[2].is_pred());
    CHECK_FALSE(flock.state()[3].is_pred());
    CHECK_FALSE(flock.state()[4].is_pred());
    CHECK_FALSE(flock.state()[5].is_pred());
    CHECK_FALSE(flock.state()[6].is_pred());
    // order unchanged, positions and velocities modified accordingly to flying
    // rules
    CHECK(
        flock.state()[0].position().x() // b1 has 1 neighbour (+ bound_position)
        == b1.position().x() + b1.velocity().x() * d_t);
    CHECK(flock.state()[0].position().y()
          == b1.position().y() + b1.velocity().y() * d_t);
    CHECK(flock.state()[0].velocity().x()
          == doctest::Approx(.5 + 1.5 * std::sqrt(12.5)));
    CHECK(flock.state()[0].velocity().y()
          == doctest::Approx(3.5 + 1.5 * std::sqrt(12.5)));

    CHECK(flock.state()[1].position().x() // b2 has no neighbours
          == b2.position().x() + b2.velocity().x() * d_t);
    CHECK(flock.state()[1].position().y()
          == b2.position().y() + b2.velocity().y() * d_t);
    CHECK(flock.state()[1].velocity() == b2.velocity());

    CHECK(flock.state()[2].position().x() // b3 has no neighbours
          == b3.position().x() + b3.velocity().x() * d_t);
    CHECK(flock.state()[2].position().y()
          == b3.position().y() + b3.velocity().y() * d_t);
    CHECK(flock.state()[2].velocity() == b3.velocity());

    CHECK(flock.state()[3].position().x() // b4 has no neighbours
          == b4.position().x() + b4.velocity().x() * d_t);
    CHECK(flock.state()[3].position().y()
          == b4.position().y() + b4.velocity().y() * d_t);
    CHECK(flock.state()[3].velocity() == b4.velocity());

    CHECK(flock.state()[4].position().x() // b5 has 2 neighbours
          == b5.position().x() + b5.velocity().x() * d_t);
    CHECK(flock.state()[4].position().y()
          == b5.position().y() + b5.velocity().y() * d_t);
    CHECK(flock.state()[4].velocity()
          == (b5.velocity() + Velocity{-1.375, .75}));

    CHECK(flock.state()[5]
              .position()
              .x() // b6 has 2 neighbours (+ bound_position)
          == b6.position().x() + b6.velocity().x() * d_t);
    CHECK(flock.state()[5].position().y()
          == b6.position().y() + b6.velocity().y() * d_t);
    CHECK(flock.state()[5].velocity().x()
          == doctest::Approx(1. + 1.5 * std::sqrt(4.0625)));
    CHECK(flock.state()[5].velocity().y()
          == doctest::Approx(1.75 + 1.5 * std::sqrt(4.0625)));

    CHECK(flock.state()[6].position().x() // b7 has 2 neighbours
          == b7.position().x() + b7.velocity().x() * d_t);
    CHECK(flock.state()[6].position().y()
          == b7.position().y() + b7.velocity().y() * d_t);
    CHECK(flock.state()[6].velocity() == (b7.velocity() + Velocity{1.25, 1.5}));
  }
  SUBCASE("3 predators, no competitors")
  {
    std::vector<Boid> boids{b5_p, b6_p, b7_p};
    Flock flock{boids};
    flock.evolve(pars);
    CHECK(flock.state().size() == 3); // size left unchanged
    // is_pred attribute left unchanged
    CHECK(flock.state()[0].is_pred());
    CHECK(flock.state()[1].is_pred());
    CHECK(flock.state()[2].is_pred());
    // order and velocity unchanged, positions modified only by the motion of
    // boid itself
    CHECK(flock.state()[0].position().x()
          == b5_p.position().x() + b5_p.velocity().x() * d_t);
    CHECK(flock.state()[0].position().y()
          == b5_p.position().y() + b5_p.velocity().y() * d_t);
    CHECK(flock.state()[0].velocity() == b5_p.velocity());

    CHECK(flock.state()[1].position().x()
          == b6_p.position().x() + b6_p.velocity().x() * d_t);
    CHECK(flock.state()[1].position().y()
          == b6_p.position().y() + b6_p.velocity().y() * d_t);
    CHECK(flock.state()[1].velocity() == b6_p.velocity());

    CHECK(flock.state()[2].position().x()
          == b7_p.position().x() + b7_p.velocity().x() * d_t);
    CHECK(flock.state()[2].position().y()
          == b7_p.position().y() + b7_p.velocity().y() * d_t);
    CHECK(flock.state()[2].velocity() == b7_p.velocity());
  }

  SUBCASE("5 predators, some with competitors")
  {
    Boid b8_p{{4., 15.5}, {0., 2.}, true};
    Boid b9_p{{3.5, 16.}, {1., 0.}, true};
    std::vector<Boid> boids{b5_p, b6_p, b7_p, b8_p, b9_p};
    Flock flock{boids};
    flock.evolve(pars);
    CHECK(flock.state().size() == 5); // size left unchanged
    // is_pred attribute left unchanged
    CHECK(flock.state()[0].is_pred());
    CHECK(flock.state()[1].is_pred());
    CHECK(flock.state()[2].is_pred());
    CHECK(flock.state()[3].is_pred());
    CHECK(flock.state()[4].is_pred());
    // order unchanged, positions and velocities modified accordingly to flying
    // rules
    CHECK(flock.state()[0].position().x() // b5_p has no competitors
          == b5_p.position().x() + b5_p.velocity().x() * d_t);
    CHECK(flock.state()[0].position().y()
          == b5_p.position().y() + b5_p.velocity().y() * d_t);
    CHECK(flock.state()[0].velocity() == b5_p.velocity());

    CHECK(flock.state()[1].position().x() // b6_p has no competitors
          == b6_p.position().x() + b6_p.velocity().x() * d_t);
    CHECK(flock.state()[1].position().y()
          == b6_p.position().y() + b6_p.velocity().y() * d_t);
    CHECK(flock.state()[1].velocity() == b6_p.velocity());

    CHECK(flock.state()[2].position().x() // b7_p has 2 competitors
          == b7_p.position().x() + b7_p.velocity().x() * d_t);
    CHECK(flock.state()[2].position().y()
          == b7_p.position().y() + b7_p.velocity().y() * d_t);
    CHECK(flock.state()[2].velocity() == b7_p.velocity() + Velocity{1., 1.});

    CHECK(flock.state()[3].position().x() // b8_p has 2 competitors
          == b8_p.position().x() + b8_p.velocity().x() * d_t);
    CHECK(flock.state()[3].position().y()
          == b8_p.position().y() + b8_p.velocity().y() * d_t);
    CHECK(flock.state()[3].velocity() == b8_p.velocity() + Velocity{1., -2.});

    CHECK(flock.state()[4].position().x() // b9_p has 2 competitors
          == b9_p.position().x() + b9_p.velocity().x() * d_t);
    CHECK(flock.state()[4].position().y()
          == b9_p.position().y() + b9_p.velocity().y() * d_t);
    CHECK(flock.state()[4].velocity() == b9_p.velocity() + Velocity{-2., 1.});
  }
  SUBCASE("one regular and one predator")
  { // d_s_pred is 2.5 times greater than d_s
    std::vector<Boid> boids{b2, b6_p};
    Flock flock{boids};
    flock.evolve(pars);
    CHECK(flock.state().size() == 2); // size left unchanged
    // is_pred attribute left unchanged
    CHECK_FALSE(flock.state()[0].is_pred());
    CHECK(flock.state()[1].is_pred());
    // order unchanged, positions and velocities modified accordingly to flying
    // rules

    // b2 can't see b6_p (not in angle of view) so he doesn't feel separation
    CHECK(flock.state()[0].position().x()
          == b2.position().x() + b2.velocity().x() * d_t);
    CHECK(flock.state()[0].position().y()
          == b2.position().y() + b2.velocity().y() * d_t);
    CHECK(flock.state()[0].velocity() == b2.velocity());

    // b6_p feels seek drive towards b2
    CHECK(flock.state()[1].position().x()
          == b6_p.position().x() + b6_p.velocity().x() * d_t);
    CHECK(flock.state()[1].position().y()
          == b6_p.position().y() + b6_p.velocity().y() * d_t);
    CHECK(flock.state()[1].velocity().x()
          == doctest::Approx(
                 (b6_p.velocity() + (Velocity{2., 3.}) * (0.008770580193)).x())
                 .epsilon(0.01));
    CHECK(flock.state()[1].velocity().y()
          == doctest::Approx(
                 (b6_p.velocity() + (Velocity{2., 3.}) / std::sqrt(13000.)).y())
                 .epsilon(0.1));
  }

  SUBCASE("one regular and one close predator")
  { // d_s_pred is 2.5 times greater than d_s
    b6_p.position() = {9., 9.};
    std::vector<Boid> boids{b2, b6_p};
    Flock flock{boids};
    flock.evolve(pars);
    CHECK(flock.state().size() == 2); // size left unchanged
    // is_pred attribute left unchanged
    CHECK_FALSE(flock.state()[0].is_pred());
    CHECK(flock.state()[1].is_pred());
    // order unchanged, positions and velocities modified accordingly to flying
    // rules

    // b2 feels (strong) separation from b6_p
    CHECK(flock.state()[0].position().x()
          == b2.position().x() + b2.velocity().x() * d_t);
    CHECK(flock.state()[0].position().y()
          == b2.position().y() + b2.velocity().y() * d_t);
    CHECK(flock.state()[0].velocity()
          == b2.velocity() + Velocity{-1., 1.} * pars.get_s_pred());

    // b6_p feels seek drive towards b2
    CHECK(flock.state()[1].position().x()
          == b6_p.position().x() + b6_p.velocity().x() * d_t);
    CHECK(flock.state()[1].position().y()
          == b6_p.position().y() + b6_p.velocity().y() * d_t);
    CHECK(flock.state()[1].velocity().x()
          == doctest::Approx(
                 (b6_p.velocity() + (Velocity{2., 3.}) * (0.008770580193)).x())
                 .epsilon(0.01));
    CHECK(flock.state()[1].velocity().y()
          == doctest::Approx(
                 (b6_p.velocity() + (Velocity{2., 3.}) / std::sqrt(13000.)).y())
                 .epsilon(0.1));
  }

  SUBCASE(
      "Generalization: 6 regulars and 3 predators, competitors, neighbours and "
      "close neighbours, two consecutive evolutions: one regular going out "
      "of grid after first evolution,")
  {
    b6_p.position() = {9.5, 9.5};
    b4.velocity()   = {0., -1.};
    Boid b5{{2., 2.1}, {.5, 1.}};
    Boid b8{{.5, 0.}, {1., -1.}}; // will cross y_min
    Boid b9_p{{10., 10.}, {1., 3.}, true};
    std::vector<Boid> boids{b1, b2, b3, b4, b5, b6_p, b7_p, b8, b9_p};
    Flock flock{boids};

    // first evolution
    flock.evolve(pars);
    CHECK(flock.state().size() == 9); // size left unchanged
    // is_pred attribute left unchanged
    CHECK_FALSE(flock.state()[0].is_pred());
    CHECK_FALSE(flock.state()[1].is_pred());
    CHECK_FALSE(flock.state()[2].is_pred());
    CHECK_FALSE(flock.state()[3].is_pred());
    CHECK_FALSE(flock.state()[4].is_pred());
    CHECK(flock.state()[5].is_pred());
    CHECK(flock.state()[6].is_pred());
    CHECK_FALSE(flock.state()[7].is_pred());
    CHECK(flock.state()[8].is_pred());
    // order unchanged, positions and velocities modified accordingly to flying
    // rules
    // b1 has 1 close neighbour, 1 neighbour, no predators (+ bound_position)
    CHECK(flock.state()[0].position().x()
          == b1.position().x() + b1.velocity().x() * d_t);
    CHECK(flock.state()[0].position().y()
          == b1.position().y() + b1.velocity().y() * d_t);
    CHECK(flock.state()[0].velocity().x()
          == doctest::Approx(0.375 + 1.5 * std::sqrt(0.41625)));
    CHECK(flock.state()[0].velocity().y()
          == doctest::Approx(0.525 + 1.5 * std::sqrt(0.41625)));

    // b2 has 2 predators, no neighbours
    CHECK(flock.state()[1].position().x()
          == b2.position().x() + b2.velocity().x() * d_t);
    CHECK(flock.state()[1].position().y()
          == b2.position().y() + b2.velocity().y() * d_t);
    CHECK(flock.state()[1].velocity()
          == b2.velocity() + (Velocity{-3.5, .5} * pars.get_s_pred()));

    // b3 has no predators nor neighbours
    CHECK(flock.state()[2].position().x()
          == b3.position().x() + b3.velocity().x() * d_t);
    CHECK(flock.state()[2].position().y()
          == b3.position().y() + b3.velocity().y() * d_t);
    CHECK(flock.state()[2].velocity() == b3.velocity());

    // b4 has no predators, 1 neighbour
    CHECK(flock.state()[3].position().x()
          == b4.position().x() + b4.velocity().x() * d_t);
    CHECK(flock.state()[3].position().y()
          == b4.position().y() + b4.velocity().y() * d_t);
    CHECK(flock.state()[3].velocity() == b4.velocity() + Velocity{.5, .55});

    // b5 has only b4 as neighbour (no close, no preds). Its d_v
    // is expected to be the opposite as b4
    CHECK(flock.state()[4].position().x()
          == b5.position().x() + b5.velocity().x() * d_t);
    CHECK(flock.state()[4].position().y()
          == b5.position().y() + b5.velocity().y() * d_t);
    CHECK(flock.state()[4].velocity() == (b5.velocity() + Velocity{-.5, -.55}));

    // b6_p has 1 prey and 1 competitor
    CHECK(flock.state()[5].position().x()
          == b6_p.position().x() + b6_p.velocity().x() * d_t);
    CHECK(flock.state()[5].position().y()
          == b6_p.position().y() + b6_p.velocity().y() * d_t);
    CHECK(flock.state()[5].velocity()
          == b6_p.velocity() + Velocity{-1., -1.}
                 + Velocity{2.5, -3.5} * (std::sqrt(545. / 370000.)));

    // b7_p has a prey and no competitors
    CHECK(flock.state()[6].position().x()
          == b7_p.position().x() + b7_p.velocity().x() * d_t);
    CHECK(flock.state()[6].position().y()
          == b7_p.position().y() + b7_p.velocity().y() * d_t);
    CHECK(flock.state()[6].velocity()
          == b7_p.velocity()
                 + Velocity{5., -4.} * .03 * (std::sqrt(26. / 41.)));

    // b8 has 1 close neighbour, 1 neighbour, no predators
    CHECK(flock.state()[7].position().x()
          == b8.position().x() + b8.velocity().x() * d_t);
    CHECK(flock.state()[7].position().y()
          == b8.position().y() + b8.velocity().y() * d_t);
    // v_x and v_y modified according to flocking
    // behavior and bound_position
    CHECK(flock.state()[7].velocity().x()
          == doctest::Approx(2. + 1.5 * std::sqrt(6.325625)));
    CHECK(flock.state()[7].velocity().y()
          == doctest::Approx(1.525 + 1.5 * std::sqrt(6.325625)));

    // b9_p has a prey and no competitors
    CHECK(flock.state()[8].position().x()
          == b9_p.position().x() + b9_p.velocity().x() * d_t);
    CHECK(flock.state()[8].position().y()
          == b9_p.position().y() + b9_p.velocity().y() * d_t);
    CHECK(flock.state()[8].velocity()
          == b9_p.velocity() + Velocity{-1., 2.} / (std::sqrt(1250.)));

    // second evolution
    flock.evolve(pars);
    CHECK(flock.state().size() == 9); // size left unchanged
    // is_pred attribute left unchanged
    CHECK_FALSE(flock.state()[0].is_pred());
    CHECK_FALSE(flock.state()[1].is_pred());
    CHECK_FALSE(flock.state()[2].is_pred());
    CHECK_FALSE(flock.state()[3].is_pred());
    CHECK_FALSE(flock.state()[4].is_pred());
    CHECK(flock.state()[5].is_pred());
    CHECK(flock.state()[6].is_pred());
    CHECK_FALSE(flock.state()[7].is_pred());
    CHECK(flock.state()[8].is_pred());
    // order unchanged, positions and velocities modified accordingly to flying
    // rules

    // b1 has 1 close neighbour, 1 neighbour, no predators (+ bound_position)
    CHECK(flock.state()[0].position().x() == doctest::Approx(0.02342761593));
    CHECK(flock.state()[0].position().y() == doctest::Approx(0.02492761593));
    CHECK(flock.state()[0].velocity().x() == doctest::Approx(8.890135));
    CHECK(flock.state()[0].velocity().y() == doctest::Approx(9.813886226));

    // b2 has (now) no predators, no neighbours
    CHECK(flock.state()[1].position().x() == doctest::Approx(7.285));
    CHECK(flock.state()[1].position().y() == doctest::Approx(10.145));
    // b2's velocity is unaltered since after last evolutions he hasn't got
    // predators anymore
    CHECK(flock.state()[1].velocity()
          == b2.velocity() + (Velocity{-3.5, .5} * pars.get_s_pred()));

    // b3 has no predators nor neighbours
    CHECK(flock.state()[2].position().x() == doctest::Approx(12.98));
    CHECK(flock.state()[2].position().y() == doctest::Approx(3.06));
    CHECK(flock.state()[2].velocity() == b3.velocity());

    // b4 has no predators, 1 neighbour
    CHECK(flock.state()[3].position().x() == doctest::Approx(2.005));
    CHECK(flock.state()[3].position().y() == doctest::Approx(4.9855));
    CHECK(flock.state()[3].velocity().x() == doctest::Approx(.0025));
    CHECK(flock.state()[3].velocity().y() == doctest::Approx(-0.99));

    /// b5 has b4, b1, b8 as neighbours
    CHECK(flock.state()[4].position().x() == doctest::Approx(2.005));
    CHECK(flock.state()[4].position().y() == doctest::Approx(2.1145));
    CHECK(flock.state()[4].velocity().x() == doctest::Approx(1.95596029));
    CHECK(flock.state()[4].velocity().y() == doctest::Approx(1.890127));

    // b6_p has 1 prey (b2 instead of b3 as before)and 1 competitor
    CHECK(flock.state()[5].position().x() == doctest::Approx(9.530959483));
    CHECK(flock.state()[5].position().y() == doctest::Approx(9.468656723));
    CHECK(flock.state()[5].velocity().x()
          == doctest::Approx(0.0804003).epsilon(0.01));
    CHECK(flock.state()[5].velocity().y()
          == doctest::Approx(-3.199816697).epsilon(0.01));

    // b7_p has a prey and no competitors
    CHECK(flock.state()[6].position().x() == doctest::Approx(4.0311945));
    CHECK(flock.state()[6].position().y() == doctest::Approx(16.029044));
    CHECK(flock.state()[6].velocity().x()
          == doctest::Approx(1.49411633).epsilon(0.1));
    CHECK(flock.state()[6].velocity().y()
          == doctest::Approx(1.313303322).epsilon(0.1));

    // b8 has 1 close neighbour, 1 neighbour, no predators
    CHECK(flock.state()[7].position().x() == doctest::Approx(0.56772619));
    // note that thanks to bound_position, b8 has re-entered the grid in the
    // following evolution
    CHECK(flock.state()[7].position().y() == doctest::Approx(0.042976));
    CHECK(flock.state()[7].velocity().x() == doctest::Approx(5.544167713));
    CHECK(flock.state()[7].velocity().y() == doctest::Approx(5.090417713));

    // b9_p has a prey (the same as before) and no competitors
    CHECK(flock.state()[8].position().x() == doctest::Approx(10.01971716));
    CHECK(flock.state()[8].position().y() == doctest::Approx(10.06056569));
    CHECK(flock.state()[8].velocity().x()
          == doctest::Approx(0.9159133389).epsilon(0.01));
    CHECK(flock.state()[8].velocity().y()
          == doctest::Approx(3.088206071).epsilon(0.01));

    // Note that this way of writing the test:
    // CHECK(flock.state()[0].position()
    // == b1.position() + b1.velocity() * d_t); wouldn't have worked for 2nd
    // evolution since boids in flock_ got updated, but the variables b1,b2
    // etc. didn't
  }
}

TEST_CASE("Testing simulate")
{
  Parameters const pars{90.,     5.,  2., 1., 1., 1., 100,
                        .000005, 10., 10, 2,  10, 2};
  // one regular and one predator, can't see each other
  Boid b1{{5., 2.}, {1., 0.}};
  Boid b2_p{{30., 2.}, {0., 1.}, true};
  Flock flock{std::vector<Boid>{b1, b2_p}};
  std::vector<std::vector<Boid>> states{};
  simulate(flock, pars);

  // checking flock evolved 10 times by confronting final positions
  CHECK(flock.state()[0].position() == Position{15., 2.});
  CHECK(flock.state()[1].position() == Position{30., 12.});
  // checking state was saved for 5 times
  CHECK(states.size() == 5u);
}

TEST_CASE("Testing fill")
{
  std::random_device rd;
  auto const seed{rd()};
  std::vector<Boid> boids{};

  SUBCASE("testing size and space limits")
  {
    Parameters const pars{90.,     5.,  2., 1., 1., 1., 100,
                          .000005, 10., 10, 2,  10, 40};

    Flock flock{fill(boids, pars, seed)};

    // checking fill inserted exactly N_boids
    CHECK(flock.size() == 40);
    // checking limits of space were respected
    CHECK(std::all_of(flock.state().begin(), flock.state().end(),
                      [&](Boid const& b) {
                        return b.position().x() >= pars.get_x_min()
                            && b.position().x() <= pars.get_x_max()
                            && b.position().y() >= pars.get_y_min()
                            && b.position().y() <= pars.get_y_max();
                      }));
  }

  SUBCASE("testing with small N_boids")
  {
    Parameters const pars{90.,     5.,  2., 1., 1., 1., 100,
                          .000005, 10., 10, 2,  10, 2};
    Flock flock{fill(boids, pars, seed)};
    CHECK(flock.size() == 2);
    CHECK(std::all_of(flock.state().begin(), flock.state().end(),
                      [&](Boid const& b) {
                        return b.position().x() >= pars.get_x_min()
                            && b.position().x() <= pars.get_x_max()
                            && b.position().y() >= pars.get_y_min()
                            && b.position().y() <= pars.get_y_max();
                      }));
  }

  SUBCASE("testing with large N_boids")
  {
    Parameters const pars{90.,     5.,  2., 1., 1., 1.,    100,
                          .000005, 10., 10, 2,  10, 100000};
    Flock flock{fill(boids, pars, seed)};
    CHECK(flock.size() == 100000);
    CHECK(std::all_of(flock.state().begin(), flock.state().end(),
                      [&](Boid const& b) {
                        return b.position().x() >= pars.get_x_min()
                            && b.position().x() <= pars.get_x_max()
                            && b.position().y() >= pars.get_y_min()
                            && b.position().y() <= pars.get_y_max();
                      }));
  }
}