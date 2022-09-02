#include "graphics.hpp"
#include <random>

// defines all functions responsible of graphics

// draws all boids in state, representing each boid as a triangle
void draw_state(sf::RenderWindow& window, std::vector<Boid> const& state)
{
  // creates an array of vertices defining a Triangles primitive (i.e. a set of
  // unconnected triangles)
  sf::VertexArray triangles(sf::Triangles);

  for (Boid const& boid : state) {
    // predators are represented as bigger triangles
    double const scale_fac{(boid.is_pred()) ? 1.5 : 1.};
    double constexpr half_base{3.5};   // half of the base of the triangle
    double constexpr half_height{5.5}; // half of the height of the triangle
    sf::Color grey{169, 169, 169, 255};
    sf::Color orange{255, 155, 0, 255};

    // each triangle is defined by three vertices
    sf::Vertex top;
    top.position = sf::Vector2f(boid.position().x(),
                                boid.position().y() + scale_fac * half_height);
    top.color    = (boid.is_pred()) ? sf::Color::Red : sf::Color::Black;

    sf::Vertex lr;
    lr.position = sf::Vector2f(boid.position().x() - scale_fac * half_base,
                               boid.position().y() - scale_fac * half_height);
    sf::Vertex ll;
    ll.position = sf::Vector2f(boid.position().x() + scale_fac * half_base,
                               boid.position().y() - scale_fac * half_height);
    lr.color = ll.color = (boid.is_pred()) ? orange : grey;

    // rotating triangle to match boid's velocity's direction:
    double angle{((boid.velocity().x() < 0) ? (1.) : (-1.)) * (180. / pi)
                 * std::acos(boid.velocity().y() / norm(boid.velocity()))};
    sf::Transform rotation;
    rotation.rotate(angle, boid.position().x(), boid.position().y());
    // rotates vertex of [angle] degrees (clockwise order)
    top.position = rotation.transformPoint(top.position);
    lr.position  = rotation.transformPoint(lr.position);
    ll.position  = rotation.transformPoint(ll.position);

    // inserting vertices in the array
    triangles.append(top);
    triangles.append(lr);
    triangles.append(ll);
  }

  window.draw(triangles);
}

auto evolve(Flock& flock, Parameters const& pars)
{
  for (int i{0}; i != pars.get_steps(); ++i) {
    flock.evolve(pars);
  }
  return flock.state();
}

// adds pred with position equal to mouse position (if in grid), random velocity
void add_predator(Position const& position, Flock& flock, Parameters& pars,
                  unsigned int seed)
{
  assert(position.x() >= pars.get_x_min() && position.x() <= pars.get_x_max());
  assert(position.y() >= pars.get_y_min() && position.y() <= pars.get_y_max());

  int init_size{flock.size()};
  std::default_random_engine eng(seed);
  std::uniform_real_distribution<double> unidist_v(
      -pars.get_max_speed() / sqrt2, pars.get_max_speed() / sqrt2);
  Boid boid{position, {unidist_v(eng), unidist_v(eng)}, true};
  normalize(boid.velocity(), pars.get_min_speed(), pars.get_max_speed());

  assert(boid.is_pred());
  flock.push_back(boid);
  assert(init_size + 1 == flock.size());
}

void game_loop(sf::RenderWindow& window, Flock& flock, Parameters& pars,
               unsigned int seed)
{
  std::vector<Boid> state{};
  window.setFramerateLimit(pars.get_fps());
  sf::Texture t;
  t.loadFromFile("bigsky.png");
  sf::Sprite s(t);

  while (window.isOpen()) {
    // #1 processing events:
    sf::Event event;
    while (window.pollEvent(event)) {
      switch (event.type) {
      // window closed
      case sf::Event::Closed:
        window.close();
        break;
      // Escape key pressed
      case sf::Event::KeyPressed:
        if (event.key.code == sf::Keyboard::Escape) {
          window.close();
        }
        break;
      // handle the resize events
      case sf::Event::Resized:
        // update the view to the new size of the window
        if (std::max(pars.get_d_s_pred(), pars.get_d())
            < std::min(static_cast<double>(event.size.height),
                       static_cast<double>(event.size.width))) {
          pars.set_x_max() = static_cast<double>(event.size.width);
          pars.set_y_max() = static_cast<double>(event.size.height);
          sf::FloatRect visibleArea(0, 0, pars.get_x_max(), pars.get_y_max());
          window.setView(sf::View(visibleArea));
        } else {
          // window will be resized only if new dimensions are compatible with
          // distance parameters
          window.setSize({static_cast<unsigned>(pars.get_x_max()),
                          static_cast<unsigned>(pars.get_y_max())});
        }
        break;
      default:
        break;
      }
    }

    // #2 evolving the scene:
    window.clear(sf::Color::White);
    window.draw(s);
    state = evolve(flock, pars);
    draw_state(window, state);
    // predator can be added by pressing left mouse button
    if (sf::Mouse::isButtonPressed(sf::Mouse::Left)) {
      auto mouse_position{sf::Mouse::getPosition(window)};
      // click is valid only if it's within the window
      if (mouse_position.x >= pars.get_x_min()
          && mouse_position.x <= pars.get_x_max()
          && mouse_position.y >= pars.get_y_min()
          && mouse_position.y <= pars.get_y_max()) {
        add_predator(Position{mouse_position.x, mouse_position.y}, flock, pars,
                     seed);
      }
    }

    // #3 displaying the evolved scene:
    window.display();
  }
}
