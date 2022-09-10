#ifndef FLOCK_HPP
#define FLOCK_HPP
#include "boids.hpp"
#include "parameters.hpp"
#include <vector>

// defining class Flock, declaring flocks' flying rules, declaring functions
// fill and simulate

class Flock
{
  std::vector<Boid> flock_;
  Boid solve(Boid const& boid, Parameters const& pars) const;
  int counter_{0};

 public:
  explicit Flock(std::vector<Boid> const& flock)
      : flock_(flock)
  {
    // parameter N_boids was verified by the constructor of Parameters to be > 1
    assert(flock_.size() > 1);
  }
  // clang-format off
  bool empty() const{ return flock_.empty(); }
  //NB not risking narrowing with int as return type since parameter N_boids is an int
  int size() const { return flock_.size(); }
  std::vector<Boid> const& state() const { return flock_; }
  std::vector<Boid>& state() { return flock_; }
  int counter() const {return counter_;}
  int& counter() {return counter_;}
  void push_back(Boid const& boid) 
  {
    assert (!empty());
    flock_.push_back(boid);
  }
  void evolve(Parameters const& pars);
  // clang-format on
};

// flying rules' auxiliary functions
std::vector<Boid>& neighbours(Boid const& boid, Flock const& flock,
                              std::vector<Boid>& nbrs, double angle, double d);
std::vector<Boid>& predators(Boid const& boid, Flock const& flock,
                             std::vector<Boid>& preds, double angle,
                             double d_s_pred);
std::vector<Boid>& competitors(Boid const& boid, Flock const& flock,
                               std::vector<Boid>& competitors, double angle,
                               double d_s);
Boid const& find_prey(Boid const& boid, Flock const& flock, double angle);
Boid find_prey_isolated(Boid const& boid, Flock const& flock, double angle,
                        double dist);
void set_victims(Boid const& boid, Flock& flock, Parameters const& pars);

// flying rules' functions
Velocity separation(Boid const& boid, Flock const& flock,
                    Parameters const& pars);
Velocity alignment(Boid const& boid, Flock const& flock,
                   Parameters const& pars);
Velocity cohesion(Boid const& boid, Flock const& flock, Parameters const& pars);
Velocity seek(Boid const& boid, Flock const& flock, Parameters const& pars);

std::vector<Boid>& fill(std::vector<Boid>& boids, Parameters const& pars,
                        unsigned int seed);
void add_predators(Flock& flock, Parameters const& pars, unsigned int seed);
void simulate(Flock& flock, Parameters const& pars);

#endif
