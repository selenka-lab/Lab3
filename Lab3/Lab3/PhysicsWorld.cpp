#include "PhysicsWorld.h"

#include <algorithm>
#include <chrono>
#include <cmath>

void PhysicsWorld::reset() {
  walls.clear();
  normalBodies.clear();
  fastBodies.clear();

  walls.push_back({{600.f, 140.f}, {610.f, 460.f}, sf::Color(220, 40, 40)});

  walls.push_back({{300.f, 200.f}, {340.f, 240.f}, sf::Color(220, 40, 40)});
  walls.push_back({{800.f, 200.f}, {840.f, 240.f}, sf::Color(220, 40, 40)});
  walls.push_back({{300.f, 450.f}, {340.f, 490.f}, sf::Color(220, 40, 40)});
  walls.push_back({{800.f, 450.f}, {840.f, 490.f}, sf::Color(220, 40, 40)});

  for (int i = 0; i < 14; ++i) {
    float radius = 8.f + (i % 3) * 2.f;
    normalBodies.push_back(
        {{110.f + (i % 5) * 95.f, 80.f + (i / 5) * 110.f},
         {(i % 2 == 0) ? 100.f : -100.f, ((i / 2) % 2 == 0) ? 65.f : -65.f},
         radius,
         radius * radius / 100.f,
         0.85f,
         sf::Color::White});
  }

  fastBodies.push_back(
      {{40.f, 310.f}, {2000.f, 0.f}, 6.f, 0.5f, 0.9f, sf::Color(255, 215, 0)});

  fastBodies.push_back({{1150.f, 220.f},
                        {-2000.f, 0.f},
                        6.f,
                        0.5f,
                        0.9f,
                        sf::Color(192, 192, 192)});
}

void PhysicsWorld::resolveWallCollisions(CircleBody& body) {
  for (const auto& wall : walls) {
    float closestX =
        std::clamp(body.position.x, wall.minBound.x, wall.maxBound.x);
    float closestY =
        std::clamp(body.position.y, wall.minBound.y, wall.maxBound.y);

    Vector2 delta = body.position - Vector2{closestX, closestY};
    float distance = delta.length();

    if (distance < body.radius && distance > 0.001f) {
      Vector2 normal = delta * (1.f / distance);
      body.position += normal * (body.radius - distance);

      float velocityAlongNormal = body.velocity.dot(normal);
      if (velocityAlongNormal < 0.f) {
        body.velocity -=
            normal * ((1.f + body.restitution) * velocityAlongNormal);
      }
    }
  }
}

void PhysicsWorld::resolveCircleCollisions(std::vector<CircleBody>& bodies) {
  for (size_t i = 0; i < bodies.size(); ++i) {
    for (size_t j = i + 1; j < bodies.size(); ++j) {
      CircleBody& a = bodies[i];
      CircleBody& b = bodies[j];

      Vector2 delta = b.position - a.position;
      float distance = delta.length();
      float minDistance = a.radius + b.radius;

      if (distance < minDistance && distance > 0.001f) {
        Vector2 normal = delta * (1.f / distance);
        float correction = 0.5f * (minDistance - distance);

        a.position -= normal * correction;
        b.position += normal * correction;

        float relativeVelocity = (b.velocity - a.velocity).dot(normal);
        if (relativeVelocity < 0.f) {
          float e = std::min(a.restitution, b.restitution);
          float impulseMagnitude =
              -(1.f + e) * relativeVelocity / (1.f / a.mass + 1.f / b.mass);
          Vector2 impulse = normal * impulseMagnitude;

          a.velocity -= impulse * (1.f / a.mass);
          b.velocity += impulse * (1.f / b.mass);
        }
      }
    }
  }
}

void PhysicsWorld::resolveCollisionsBetween(std::vector<CircleBody>& groupA,
                                            std::vector<CircleBody>& groupB) {
  for (auto& a : groupA) {
    for (auto& b : groupB) {
      Vector2 delta = b.position - a.position;
      float distance = delta.length();
      float minDistance = a.radius + b.radius;

      if (distance < minDistance && distance > 0.001f) {
        Vector2 normal = delta * (1.f / distance);
        float correction = 0.5f * (minDistance - distance);

        a.position -= normal * correction;
        b.position += normal * correction;

        float relativeVelocity = (b.velocity - a.velocity).dot(normal);
        if (relativeVelocity < 0.f) {
          float e = std::min(a.restitution, b.restitution);
          float impulseMagnitude =
              -(1.f + e) * relativeVelocity / (1.f / a.mass + 1.f / b.mass);
          Vector2 impulse = normal * impulseMagnitude;

          a.velocity -= impulse * (1.f / a.mass);
          b.velocity += impulse * (1.f / b.mass);
        }
      }
    }
  }
}

void PhysicsWorld::clampToWindow(CircleBody& body) {
  const float width = 1200.f;
  const float height = 600.f;

  if (body.position.x < body.radius) {
    body.position.x = body.radius;
    if (body.velocity.x < 0.f)
      body.velocity.x = -body.velocity.x * body.restitution;
  }
  if (body.position.x > width - body.radius) {
    body.position.x = width - body.radius;
    if (body.velocity.x > 0.f)
      body.velocity.x = -body.velocity.x * body.restitution;
  }
  if (body.position.y < body.radius) {
    body.position.y = body.radius;
    if (body.velocity.y < 0.f)
      body.velocity.y = -body.velocity.y * body.restitution;
  }
  if (body.position.y > height - body.radius) {
    body.position.y = height - body.radius;
    if (body.velocity.y > 0.f)
      body.velocity.y = -body.velocity.y * body.restitution;
  }
}

bool PhysicsWorld::raycastAgainstWall(Vector2 start, Vector2 delta,
                                      const Wall& wall, float radius,
                                      float& hitFraction, Vector2& hitNormal) {
  Vector2 expandedMin{wall.minBound.x - radius, wall.minBound.y - radius};
  Vector2 expandedMax{wall.maxBound.x + radius, wall.maxBound.y + radius};

  float tNear = 0.f;
  float tFar = 1.f;
  Vector2 bestNormal;

  if (std::abs(delta.x) < 1e-8f) {
    if (start.x < expandedMin.x || start.x > expandedMax.x) return false;
  } else {
    float t1 = (expandedMin.x - start.x) / delta.x;
    float t2 = (expandedMax.x - start.x) / delta.x;
    Vector2 normal = {-1.f, 0.f};

    if (t1 > t2) {
      std::swap(t1, t2);
      normal = {1.f, 0.f};
    }

    if (t1 > tNear) {
      tNear = t1;
      bestNormal = normal;
    }
    if (t2 < tFar) tFar = t2;
    if (tNear > tFar) return false;
  }

  if (std::abs(delta.y) < 1e-8f) {
    if (start.y < expandedMin.y || start.y > expandedMax.y) return false;
  } else {
    float t1 = (expandedMin.y - start.y) / delta.y;
    float t2 = (expandedMax.y - start.y) / delta.y;
    Vector2 normal = {0.f, -1.f};

    if (t1 > t2) {
      std::swap(t1, t2);
      normal = {0.f, 1.f};
    }

    if (t1 > tNear) {
      tNear = t1;
      bestNormal = normal;
    }
    if (t2 < tFar) tFar = t2;
    if (tNear > tFar) return false;
  }

  if (tNear >= 0.f && tNear <= 1.f) {
    hitFraction = tNear;
    hitNormal = bestNormal;
    return true;
  }
  return false;
}

void PhysicsWorld::updateBodyContinuous(CircleBody& body, float deltaTime) {
  float remainingTime = deltaTime;

  for (int iteration = 0; iteration < 4 && remainingTime > 1e-6f; ++iteration) {
    float earliestHitTime = remainingTime;
    Vector2 hitNormal;
    bool hasHit = false;

    for (const auto& wall : walls) {
      float fraction = 0.f;
      Vector2 normal;

      if (raycastAgainstWall(body.position, body.velocity * remainingTime, wall,
                             body.radius, fraction, normal)) {
        float hitTime = fraction * remainingTime;
        if (hitTime < earliestHitTime) {
          earliestHitTime = hitTime;
          hitNormal = normal;
          hasHit = true;
        }
      }
    }

    if (hasHit) {
      body.position += body.velocity * earliestHitTime + hitNormal * 0.02f;

      float velocityAlongNormal = body.velocity.dot(hitNormal);
      if (velocityAlongNormal < 0.f) {
        body.velocity -=
            hitNormal * ((1.f + body.restitution) * velocityAlongNormal);
      }

      remainingTime -= earliestHitTime;
    } else {
      body.position += body.velocity * remainingTime;
      remainingTime = 0.f;
    }
  }

  clampToWindow(body);
}

void PhysicsWorld::update(float deltaTime) {
  auto startTime = std::chrono::high_resolution_clock::now();
  substepsCompleted = 0;

  for (auto& body : normalBodies) {
    body.position += body.velocity * deltaTime;
    resolveWallCollisions(body);
    clampToWindow(body);
  }
  resolveCircleCollisions(normalBodies);
  resolveCollisionsBetween(normalBodies, fastBodies);

  if (simulationMode == 1) {
    for (auto& body : fastBodies) {
      if (comparisonMode) {
        body.position += body.velocity * deltaTime;
        resolveWallCollisions(body);
        clampToWindow(body);
      } else {
        updateBodyContinuous(body, deltaTime);
      }
    }
    resolveCollisionsBetween(normalBodies, fastBodies);
    substepsCompleted = 1;
  } else {
    int steps = comparisonMode ? 1 : 4;
    float fastDeltaTime = deltaTime / static_cast<float>(steps);

    for (int s = 0; s < steps; ++s) {
      for (auto& body : fastBodies) {
        body.position += body.velocity * fastDeltaTime;
        resolveWallCollisions(body);
        clampToWindow(body);
      }
      resolveCollisionsBetween(normalBodies, fastBodies);
      ++substepsCompleted;
    }
  }

  auto endTime = std::chrono::high_resolution_clock::now();
  double currentMs =
      std::chrono::duration<double, std::milli>(endTime - startTime).count();

  const double smoothFactor = 0.08;
  smoothedStepTimeMs =
      smoothedStepTimeMs * (1.0 - smoothFactor) + currentMs * smoothFactor;
  stepTimeMs = smoothedStepTimeMs;
}

void PhysicsWorld::handleEvent(const sf::Event& event) {
  if (const auto* key = event.getIf<sf::Event::KeyPressed>()) {
    using Key = sf::Keyboard::Key;

    if (key->code == Key::Num1) {
      simulationMode = 1;
      comparisonMode = false;
      reset();
    } else if (key->code == Key::Num2) {
      simulationMode = 2;
      comparisonMode = false;
      reset();
    } else if (key->code == Key::T) {
      comparisonMode = !comparisonMode;
      reset();
    } else if (key->code == Key::R) {
      reset();
    }
  }
}

void PhysicsWorld::draw(sf::RenderWindow& window) {
  for (const auto& wall : walls) {
    sf::RectangleShape rect(
        {wall.maxBound.x - wall.minBound.x, wall.maxBound.y - wall.minBound.y});
    rect.setPosition({wall.minBound.x, wall.minBound.y});
    rect.setFillColor(wall.color);
    window.draw(rect);
  }

  auto drawBodies = [&](const std::vector<CircleBody>& bodies) {
    for (const auto& body : bodies) {
      sf::CircleShape circle(body.radius);
      circle.setOrigin({body.radius, body.radius});
      circle.setPosition({body.position.x, body.position.y});
      circle.setFillColor(body.color);
      window.draw(circle);
    }
  };

  drawBodies(normalBodies);
  drawBodies(fastBodies);
}

std::string PhysicsWorld::getWindowTitle() const {
  std::string modeName;

  if (simulationMode == 1) {
    modeName = comparisonMode ? "1: CCD [Discrete 60Hz]" : "1: CCD [Active]";
  } else {
    modeName = comparisonMode ? "2: Multi-rate [60Hz]"
                              : "2: Multi-rate [60Hz / 240Hz]";
  }

  return modeName + " | " +
         std::to_string(static_cast<int>(stepTimeMs * 1000.0) / 1000.0) +
         " ms" + " | substeps: " + std::to_string(substepsCompleted) +
         " | [1,2 Mode | T Compare | R Reset]";
}
