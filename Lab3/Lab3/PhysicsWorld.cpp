#include "PhysicsWorld.h"

#include <algorithm>
#include <chrono>
#include <cmath>

void PhysicsWorld::reset() {
  walls.clear();
  normalBodies.clear();
  fastBodies.clear();

  walls.push_back({{600.f, 140.f}, {610.f, 460.f}, {230, 80, 80}});

  walls.push_back({{300.f, 200.f}, {380.f, 280.f}});
  walls.push_back({{800.f, 300.f}, {880.f, 380.f}});
  walls.push_back({{500.f, 450.f}, {580.f, 530.f}});

  for (int i = 0; i < 14; ++i) {
    float r = 8.f + (i % 3) * 2.f;
    normalBodies.push_back(
        {{110.f + (i % 5) * 95.f, 80.f + (i / 5) * 110.f},
         {(i % 2 == 0) ? 100.f : -100.f, ((i / 2) % 2 == 0) ? 65.f : -65.f},
         r,
         r * r / 100.f,
         0.85f});
  }

  fastBodies.push_back({{40.f, 310.f},
                        {2350.f, 15.f},
                        6.f,
                        0.5f,
                        0.9f,
                        sf::Color(255, 220, 50)});

  fastBodies.push_back({{1150.f, 220.f},
                        {-2150.f, 180.f},
                        6.f,
                        0.5f,
                        0.9f,
                        sf::Color(255, 160, 40)});
}

void PhysicsWorld::collideWalls(Body& b) {
  for (auto& w : walls) {
    float cx = std::clamp(b.pos.x, w.min.x, w.max.x);
    float cy = std::clamp(b.pos.y, w.min.y, w.max.y);
    Vec2 d = b.pos - Vec2{cx, cy};
    float dist = d.length();

    if (dist < b.r && dist > 0.001f) {
      Vec2 n = d * (1.f / dist);
      b.pos += n * (b.r - dist);

      float vn = b.vel.dot(n);
      if (vn < 0.f) {
        b.vel -= n * ((1.f + b.bounce) * vn);
      }
    }
  }
}

void PhysicsWorld::collideBodies(std::vector<Body>& list) {
  for (size_t i = 0; i < list.size(); ++i) {
    for (size_t j = i + 1; j < list.size(); ++j) {
      Body& a = list[i];
      Body& b = list[j];

      Vec2 d = b.pos - a.pos;
      float dist = d.length();
      float minDist = a.r + b.r;

      if (dist < minDist && dist > 0.001f) {
        Vec2 n = d * (1.f / dist);
        float corr = 0.5f * (minDist - dist);
        a.pos -= n * corr;
        b.pos += n * corr;

        float vn = (b.vel - a.vel).dot(n);
        if (vn < 0.f) {
          float e = std::min(a.bounce, b.bounce);
          float jImp = -(1.f + e) * vn / (1.f / a.mass + 1.f / b.mass);
          Vec2 imp = n * jImp;
          a.vel -= imp * (1.f / a.mass);
          b.vel += imp * (1.f / b.mass);
        }
      }
    }
  }
}

void PhysicsWorld::keepInWindow(Body& b) {
  if (b.pos.x < b.r) {
    b.pos.x = b.r;
    if (b.vel.x < 0) b.vel.x = -b.vel.x * b.bounce;
  }
  if (b.pos.x > 1200.f - b.r) {
    b.pos.x = 1200.f - b.r;
    if (b.vel.x > 0) b.vel.x = -b.vel.x * b.bounce;
  }
  if (b.pos.y < b.r) {
    b.pos.y = b.r;
    if (b.vel.y < 0) b.vel.y = -b.vel.y * b.bounce;
  }
  if (b.pos.y > 600.f - b.r) {
    b.pos.y = 600.f - b.r;
    if (b.vel.y > 0) b.vel.y = -b.vel.y * b.bounce;
  }
}

bool PhysicsWorld::raycastWall(Vec2 start, Vec2 delta, const Wall& w, float r,
                               float& tOut, Vec2& nOut) {
  Vec2 bmin{w.min.x - r, w.min.y - r};
  Vec2 bmax{w.max.x + r, w.max.y + r};

  float tNear = 0.f;
  float tFar = 1.f;
  Vec2 nNear;

  if (std::abs(delta.x) < 1e-8f) {
    if (start.x < bmin.x || start.x > bmax.x) return false;
  } else {
    float t1 = (bmin.x - start.x) / delta.x;
    float t2 = (bmax.x - start.x) / delta.x;
    Vec2 n = {-1.f, 0.f};
    if (t1 > t2) {
      std::swap(t1, t2);
      n = {1.f, 0.f};
    }
    if (t1 > tNear) {
      tNear = t1;
      nNear = n;
    }
    if (t2 < tFar) tFar = t2;
    if (tNear > tFar) return false;
  }

  if (std::abs(delta.y) < 1e-8f) {
    if (start.y < bmin.y || start.y > bmax.y) return false;
  } else {
    float t1 = (bmin.y - start.y) / delta.y;
    float t2 = (bmax.y - start.y) / delta.y;
    Vec2 n = {0.f, -1.f};
    if (t1 > t2) {
      std::swap(t1, t2);
      n = {0.f, 1.f};
    }
    if (t1 > tNear) {
      tNear = t1;
      nNear = n;
    }
    if (t2 < tFar) tFar = t2;
    if (tNear > tFar) return false;
  }

  if (tNear >= 0.f && tNear <= 1.f) {
    tOut = tNear;
    nOut = nNear;
    return true;
  }
  return false;
}

void PhysicsWorld::updateBodyCCD(Body& b, float dt) {
  float remaining = dt;

  for (int iter = 0; iter < 4 && remaining > 1e-6f; ++iter) {
    float earliest = remaining;
    Vec2 hitN;
    bool hit = false;

    for (auto& w : walls) {
      float t = 0.f;
      Vec2 n;
      if (raycastWall(b.pos, b.vel * remaining, w, b.r, t, n)) {
        float hitTime = t * remaining;
        if (hitTime < earliest) {
          earliest = hitTime;
          hitN = n;
          hit = true;
        }
      }
    }

    if (hit) {
      b.pos += b.vel * earliest + hitN * 0.02f;
      float vn = b.vel.dot(hitN);
      if (vn < 0.f) {
        b.vel -= hitN * ((1.f + b.bounce) * vn);
      }
      remaining -= earliest;
    } else {
      b.pos += b.vel * remaining;
      remaining = 0.f;
    }
  }

  keepInWindow(b);
}

void PhysicsWorld::update(float dt) {
  auto t0 = std::chrono::high_resolution_clock::now();
  substeps = 0;

  for (auto& b : normalBodies) {
    b.pos += b.vel * dt;
    collideWalls(b);
    keepInWindow(b);
  }
  collideBodies(normalBodies);

  if (simulationMode == 1) {
    for (auto& b : fastBodies) {
      if (comparisonMode) {
        b.pos += b.vel * dt;
        collideWalls(b);
        keepInWindow(b);
      } else {
        updateBodyCCD(b, dt);
      }
    }
    substeps = 1;
  } else {
    int steps = comparisonMode ? 1 : 4;
    float dtFast = dt / static_cast<float>(steps);

    for (int s = 0; s < steps; ++s) {
      for (auto& b : fastBodies) {
        b.pos += b.vel * dtFast;
        collideWalls(b);
        keepInWindow(b);
      }
      ++substeps;
    }
  }

  auto t1 = std::chrono::high_resolution_clock::now();
  stepTimeMs = std::chrono::duration<double, std::milli>(t1 - t0).count();
}

void PhysicsWorld::handleEvent(const sf::Event& e) {
  if (const auto* key = e.getIf<sf::Event::KeyPressed>()) {
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

void PhysicsWorld::draw(sf::RenderWindow& win) {
  for (auto& w : walls) {
    sf::RectangleShape rect({w.max.x - w.min.x, w.max.y - w.min.y});
    rect.setPosition({w.min.x, w.min.y});
    rect.setFillColor(w.color);
    win.draw(rect);
  }

  auto drawList = [&](const std::vector<Body>& list) {
    for (auto& b : list) {
      sf::CircleShape c(b.r);
      c.setOrigin({b.r, b.r});
      c.setPosition({b.pos.x, b.pos.y});
      c.setFillColor(b.color);
      win.draw(c);
    }
  };

  drawList(normalBodies);
  drawList(fastBodies);
}

std::string PhysicsWorld::getTitle() const {
  std::string name;

  if (simulationMode == 1)
    name = comparisonMode ? "1: CCD [Discrete]" : "1: CCD [Active]";
  else
    name = comparisonMode ? "2: Multi-rate [60Hz]" : "2: Multi-rate [60/240Hz]";

  return name + " | " + std::to_string(stepTimeMs) + " ms" +
         " | sub: " + std::to_string(substeps) +
         " | [1,2 Mode | T Compare | R Reset]";
}