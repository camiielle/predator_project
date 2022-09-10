#include "boids.hpp"
#include "flock.hpp"
#include "parameters.hpp"
#include "parser.hpp"
#include "stats.hpp"

#include <fstream>
#include <random>

int main(int argc, char* argv[])
{
  try {
    // default values are modified only if an input value is specified
    double angle{300.};
    double d{35.};
    double d_s{3.5};
    double s{.7};
    double c{.045};
    double a{.8};
    double max_speed{80.};
    double min_speed_fraction{.05};
    double duration{200.};
    int steps{2000};
    int prescale{40};
    int N_boids{120};
    int N_preds{1};
    auto show_help{false};
    int seek_type{0};

    // Parser with multiple option arguments and help option
    auto parser = get_parser(angle, d, d_s, s, c, a, max_speed,
                             min_speed_fraction, duration, steps, prescale,
                             N_boids, N_preds, show_help, seek_type);

    // Parses the arguments
    auto result = parser.parse({argc, argv});

    // Checks that arguments were valid
    if (!result) {
      std::cerr << "Error occured in command line: " << result.message() << '\n'
                << parser << '\n';
      return EXIT_FAILURE;
    }

    // Shows the help if asked for
    if (show_help) {
      std::cout << parser << '\n';
      return EXIT_SUCCESS;
    }

    assert(result && (!show_help));

    int const prescale_limit{steps};

    Parameters const pars{angle,    d,       d_s,       s,
                          c,        a,       max_speed, min_speed_fraction,
                          duration, steps,   prescale,  prescale_limit,
                          N_boids,  N_preds, seek_type};

    std::array<double, simulations> preys_eaten;

    for (int i{0}; i != simulations; ++i) { // simulation loop
      // obtains seed to pass to random number engine
      std::random_device rd;
      auto const seed{rd()};
      // fills empty vector with N_boids randomly generated and uses it to
      // initialize flock
      std::vector<Boid> boids{};
      Flock flock{fill(boids, pars, seed)};
      // adds N_preds randomly generated
      add_predators(flock, pars, seed);
      // performs the simulation
      simulate(flock, pars);
      preys_eaten[i] = flock.counter();
    }

    write_counter(preys_eaten, seek_type); // write count to file for analysis

    // printing summary of the parameters used
    std::cout << '\n' << std::setfill('=') << std::setw(53);
    std::cout << '\n' << "    SUMMARY: Parameters used in the simulation\n\n";
    print_parameters(pars);

  } catch (Invalid_Parameter const& par_err) {
    std::cerr << "Invalid Parameter: " << par_err.what() << '\n';
    std::cerr << "use flags -? , -h or --help for input parameters' "
                 "instructions"
              << '\n';
    return EXIT_FAILURE;
  } catch (const std::ios_base::failure& file_err) {
    std::cout << file_err.what() << '\n';
    return EXIT_FAILURE;
  } catch (std::exception const& err) {
    std::cerr << "An error occurred: " << err.what() << '\n';
    return EXIT_FAILURE;
  } catch (...) {
    std::cerr << "An unknown error occured\n";
    return EXIT_FAILURE;
  }
}