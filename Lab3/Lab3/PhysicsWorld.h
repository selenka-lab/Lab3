#pragma once
#include <SFML/Graphics.hpp>
#include <string>
#include <vector>

#include "Vector2.h"

struct Body {
  Vec2 pos, vel;
  float r = 10.f;
  float mass = 1.f;
  float bounce = 0.85f;
  sf::Color color = sf::Color(80, 200, 255);
};

struct Wall {
  Vec2 min, max;
  sf::Color color = sf::Color(120, 125, 145);
};

class PhysicsWorld {
 public:
  int simulationMode = 1;
  bool comparisonMode = false;

  double stepTimeMs = 0.0;
  int substeps = 0;

  void reset();
  void update(float dt);
  void handleEvent(const sf::Event& e);
  void draw(sf::RenderWindow& win);
  std::string getTitle() const;

 private:
  std::vector<Wall> walls;
  std::vector<Body> normalBodies;
  std::vector<Body> fastBodies;

  void collideWalls(Body& b);
  void collideBodies(std::vector<Body>& list);
  void keepInWindow(Body& b);

  bool raycastWall(Vec2 start, Vec2 delta, const Wall& w, float r, float& tOut,
                   Vec2& nOut);
  void updateBodyCCD(Body& b, float dt);
};