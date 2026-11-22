#pragma once
#include <SFML/Graphics.hpp>
#include <string>
#include <vector>

#include "Vector2.h"

struct CircleBody {
  Vector2 position;
  Vector2 velocity;
  float radius = 10.0f;
  float mass = 1.0f;
  float restitution = 0.85f;
  sf::Color color = sf::Color::White;
};

struct Wall {
  Vector2 minBound;
  Vector2 maxBound;
  sf::Color color = sf::Color(220, 50, 50);
};

class PhysicsWorld {
 public:
  int simulationMode = 1;
  bool comparisonMode = false;

  double smoothedStepTimeMs = 0.0;
  double stepTimeMs = 0.0;
  int substepsCompleted = 0;

  void reset();
  void update(float deltaTime);
  void handleEvent(const sf::Event& event);
  void draw(sf::RenderWindow& window);
  std::string getWindowTitle() const;

 private:
  std::vector<Wall> walls;
  std::vector<CircleBody> normalBodies;
  std::vector<CircleBody> fastBodies;

  void resolveWallCollisions(CircleBody& body);
  void resolveCircleCollisions(std::vector<CircleBody>& bodies);
  void clampToWindow(CircleBody& body);

  bool raycastAgainstWall(Vector2 start, Vector2 delta, const Wall& wall,
                          float radius, float& hitFraction, Vector2& hitNormal);
  void updateBodyContinuous(CircleBody& body, float deltaTime);
};