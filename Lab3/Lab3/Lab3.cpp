#include <SFML/Graphics.hpp>
#include <algorithm>
#include <cmath>
#include <vector>

struct Vec2 {
  float x = 0.f, y = 0.f;

  Vec2() = default;
  Vec2(float x, float y) : x(x), y(y) {}

  Vec2 operator+(const Vec2& o) const { return {x + o.x, y + o.y}; }
  Vec2 operator-(const Vec2& o) const { return {x - o.x, y - o.y}; }
  Vec2 operator*(float s) const { return {x * s, y * s}; }

  Vec2& operator+=(const Vec2& o) {
    x += o.x;
    y += o.y;
    return *this;
  }
  Vec2& operator-=(const Vec2& o) {
    x -= o.x;
    y -= o.y;
    return *this;
  }

  float dot(const Vec2& o) const { return x * o.x + y * o.y; }
  float length() const { return std::sqrt(x * x + y * y); }
};

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

std::vector<Wall> walls;
std::vector<Body> bodies;

void reset() {
  walls.clear();
  bodies.clear();

  walls.push_back({{600.f, 140.f}, {610.f, 460.f}, {230, 80, 80}});

  walls.push_back({{300.f, 200.f}, {380.f, 280.f}});
  walls.push_back({{800.f, 300.f}, {880.f, 380.f}});
  walls.push_back({{500.f, 450.f}, {580.f, 530.f}});

  for (int i = 0; i < 16; ++i) {
    float r = 8.f + (i % 3) * 2.f;
    bodies.push_back(
        {{120.f + (i % 4) * 110.f, 90.f + (i / 4) * 100.f},
         {(i % 2 == 0) ? 110.f : -110.f, ((i / 2) % 2 == 0) ? 70.f : -70.f},
         r,
         r * r / 100.f,
         0.85f});
  }
}

void collideWalls(Body& b) {
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

void collideBodies() {
  for (size_t i = 0; i < bodies.size(); ++i) {
    for (size_t j = i + 1; j < bodies.size(); ++j) {
      Body& a = bodies[i];
      Body& b = bodies[j];

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

void keepInWindow(Body& b) {
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

void update(float dt) {
  for (auto& b : bodies) {
    b.pos += b.vel * dt;
    collideWalls(b);
    keepInWindow(b);
  }
  collideBodies();
}

int main() {
  sf::RenderWindow window(sf::VideoMode({1200u, 600u}), "Дискретная физика");
  window.setFramerateLimit(60);
  reset();

  while (window.isOpen()) {
    while (const std::optional<sf::Event> event = window.pollEvent()) {
      if (event->is<sf::Event::Closed>()) window.close();
    }

    update(1.f / 60.f);

    window.clear(sf::Color(22, 24, 30));

    for (auto& w : walls) {
      sf::RectangleShape rect({w.max.x - w.min.x, w.max.y - w.min.y});
      rect.setPosition({w.min.x, w.min.y});
      rect.setFillColor(w.color);
      window.draw(rect);
    }

    for (auto& b : bodies) {
      sf::CircleShape c(b.r);
      c.setOrigin({b.r, b.r});
      c.setPosition({b.pos.x, b.pos.y});
      c.setFillColor(b.color);
      window.draw(c);
    }

    window.display();
  }
}
