#include <SFML/Graphics.hpp>

#include "PhysicsWorld.h"

int main() {
  sf::RenderWindow window(sf::VideoMode({1200u, 600u}), "2D Physics");
  window.setFramerateLimit(60);

  PhysicsWorld world;
  world.reset();

  while (window.isOpen()) {
    while (const std::optional<sf::Event> event = window.pollEvent()) {
      if (event->is<sf::Event::Closed>())
        window.close();
      else
        world.handleEvent(*event);
    }

    world.update(1.f / 60.f);
    window.setTitle(world.getTitle());

    window.clear(sf::Color(22, 24, 30));
    world.draw(window);
    window.display();
  }
}
