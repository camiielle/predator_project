#include "boids.hpp"
#include "flock.hpp"
#include "graphics.hpp"
#include "parameters.hpp"
#include "parser-sfml.hpp"
#include "stats.hpp"

#include <SFML/Graphics.hpp>
#include <SFML/System/Time.hpp>
#include <iostream>
#include <random>

int main(int argc, char* argv[])
{
  try {
    // default values are modified only if an input value is specified
    double angle{310.};
    double d{55.};
    double d_s{10.};
    double s{4.9};
    double c{.0015};
    double a{.1};
    double max_speed{500};
    double min_speed_fraction{.5};
    int delta_t{1};
    int fps{60};
    int N_boids{80};
    auto show_help{false};

    // display width and height
    double const display_width{.7 * sf::VideoMode::getDesktopMode().width};
    double const display_height{.7 * sf::VideoMode::getDesktopMode().height};

    // Parser with multiple option arguments and help option
    auto parser = get_parser(angle, d, d_s, s, c, a, max_speed,
                             min_speed_fraction, delta_t, fps, N_boids,
                             display_width, display_height, show_help);

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

    // delta_t is the only input par not passed directly to Parameters'
    // constructor, so its input is validated before
    if (delta_t <= 0 || delta_t >= 125) {
      throw Invalid_Parameter{"Parameter delta_t is not in the required range"};
    }

    double const duration{sf::milliseconds(delta_t).asSeconds()};
    int const steps{1000 / (delta_t * fps)}; // steps per evolution
    int const fps_limit{1000 / delta_t};

    Parameters pars{angle,    d,     d_s,       s,
                    c,        a,     max_speed, min_speed_fraction,
                    duration, steps, fps,       fps_limit,
                    N_boids};

    // obtains seed to pass to random number engine
    std::random_device rd;
    auto const seed{rd()};
    // fills empty vector with N_boids randomly generated and uses it to
    // initialize flock
    std::vector<Boid> boids{};
    Flock flock{fill(boids, pars, seed)};

    // graphics
    sf::RenderWindow window(sf::VideoMode(display_width, display_height),
                            "Flock simulation");
    game_loop(window, flock, pars, seed);

  } catch (Invalid_Parameter const& par_err) {
    std::cerr << "Invalid Parameter: " << par_err.what() << '\n';
    std::cerr << "use flags -? , -h or --help for input parameters' "
                 "instructions"
              << '\n';
    return EXIT_FAILURE;
  } catch (std::exception const& err) {
    std::cerr << "An error occurred: " << err.what() << '\n';
    return EXIT_FAILURE;
  } catch (...) {
    std::cerr << "An unknown error occured\n";
    return EXIT_FAILURE;
  }
}