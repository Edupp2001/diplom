// [автоматический разворот по полуокружности добавлен по клавише K]
#include <SFML/Graphics.hpp>
#include <cmath>
#include <iostream>

constexpr float PI = 3.14159265f;
constexpr float DEG_TO_RAD = PI / 180.0f;
constexpr float METERS_TO_PIXELS = 25.0f;

constexpr float TRACTOR_LENGTH_M = 3.9f;
constexpr float TRACTOR_WIDTH_M = 1.97f;
constexpr float TRACTOR_LENGTH = TRACTOR_LENGTH_M * METERS_TO_PIXELS;
constexpr float TRACTOR_WIDTH = TRACTOR_WIDTH_M * METERS_TO_PIXELS;

constexpr float TRAILER_WIDTH_M = 2.0f;
constexpr float TRAILER_WIDTH = TRAILER_WIDTH_M * METERS_TO_PIXELS;
constexpr float TRAILER_LENGTH = 1.8f * METERS_TO_PIXELS;
constexpr float HITCH_LENGTH = 1.2f * METERS_TO_PIXELS;

constexpr int WINDOW_WIDTH = 1200;
constexpr int WINDOW_HEIGHT = 800;
constexpr float MAX_SPEED = 3.33f * METERS_TO_PIXELS;
constexpr float MAX_TURN_RATE = 120.f;

class Tractor {
public:
    sf::Vector2f position;
    float angle = 0.f;
    float steeringAngle = 0.f;
    float speed = 0.f;
    bool cruiseControl = true;
    bool stopping = false;
    bool turningRight = false;
    float turnProgress = 0.f;
    float radius = 0.f;
    sf::Vector2f turnCenter;

    Tractor() {
        position = sf::Vector2f(WINDOW_WIDTH / 2, WINDOW_HEIGHT / 2);
    }

    void toggleCruise() { cruiseControl = !cruiseControl; }
    void stop() { stopping = true; }

    void startTurnRight() {
        if (turningRight) return;
        turningRight = true;
        turnProgress = 0.f;
        radius = TRACTOR_LENGTH * 1.5f;
        float angleRad = angle * DEG_TO_RAD;
        turnCenter = position + sf::Vector2f(-std::sin(angleRad), std::cos(angleRad)) * radius;
    }

    void update(float dt) {
        float wheelBase = 2.45f * METERS_TO_PIXELS;

        if (stopping) {
            float brakeDecel = 15.f * dt;
            if (speed > 0) speed = std::max(0.f, speed - brakeDecel);
            else if (speed < 0) speed = std::min(0.f, speed + brakeDecel);
            if (speed == 0) stopping = false;
        }

        if (turningRight) {
            // Принудительная установка скорости разворота
            float targetTurnSpeed = 3.0f * METERS_TO_PIXELS;
            if (speed < targetTurnSpeed) speed = std::min(speed + 20.f * dt, targetTurnSpeed);
            else if (speed > targetTurnSpeed) speed = std::max(speed - 20.f * dt, targetTurnSpeed);
            float turnSpeed = speed;
            float angleDelta = (turnSpeed * dt) / radius * 180.f / PI;
            turnProgress += angleDelta;

            if (turnProgress >= 180.f) {
                angle += angleDelta - (turnProgress - 180.f);
                turningRight = false;
                angle = std::fmod(angle + 360.f, 360.f);
            }
            else {
                angle += angleDelta;
            }

            float rad = (angle - 90.f) * DEG_TO_RAD;
            position = turnCenter + sf::Vector2f(std::cos(rad), std::sin(rad)) * radius;
        }
        else {
            float theta = angle * DEG_TO_RAD;
            float beta = atan(tan(steeringAngle * DEG_TO_RAD));
            position.x += speed * cos(theta + beta) * dt;
            position.y += speed * sin(theta + beta) * dt;
            angle += (speed / wheelBase) * tan(steeringAngle * DEG_TO_RAD) * dt * 180.0f / PI;
        }
    }

    void draw(sf::RenderWindow& window) {
        sf::RectangleShape body(sf::Vector2f(TRACTOR_LENGTH, TRACTOR_WIDTH));
        body.setOrigin(TRACTOR_LENGTH / 2, TRACTOR_WIDTH / 2);
        body.setPosition(position);
        body.setRotation(angle);
        body.setFillColor(cruiseControl ? sf::Color::Blue : sf::Color::Red);

        sf::RectangleShape stripe(sf::Vector2f(TRACTOR_WIDTH * 0.6f, TRACTOR_WIDTH * 0.1f));
        stripe.setFillColor(sf::Color::Yellow);
        stripe.setOrigin(stripe.getSize().x / 2, stripe.getSize().y / 2);
        stripe.setPosition(position);
        stripe.setRotation(angle);

        float offset = TRACTOR_LENGTH / 2 - stripe.getSize().y;
        float dx = std::cos(angle * DEG_TO_RAD) * offset;
        float dy = std::sin(angle * DEG_TO_RAD) * offset;
        stripe.move(dx, dy);

        window.draw(body);
        window.draw(stripe);
    }

    sf::Vector2f getHitchPosition() const {
        float theta = angle * DEG_TO_RAD;
        return position - sf::Vector2f(std::cos(theta), std::sin(theta)) * (TRACTOR_LENGTH / 2);
    }
};

class Trailer {
public:
    bool active = false;
    sf::Vector2f position;
    float angleDeg = 0.f;

    void toggle() { active = !active; }

    void update(const sf::Vector2f& hitch, float dt, sf::RenderTexture& terrainLayer) {
        sf::Vector2f toHitch = hitch - position;
        float dist = std::sqrt(toHitch.x * toHitch.x + toHitch.y * toHitch.y);

        if (dist > 0.001f) {
            sf::Vector2f dir = toHitch / dist;
            float targetAngle = std::atan2(dir.y, dir.x) * 180.f / PI;
            float delta = targetAngle - angleDeg;
            while (delta > 180.f) delta -= 360.f;
            while (delta < -180.f) delta += 360.f;
            float maxTurn = MAX_TURN_RATE * dt;
            delta = std::clamp(delta, -maxTurn, maxTurn);
            angleDeg += delta;
        }

        float angleRad = angleDeg * DEG_TO_RAD;
        position = hitch - sf::Vector2f(std::cos(angleRad), std::sin(angleRad)) * HITCH_LENGTH;

        if (active) {
            float w = TRAILER_WIDTH / 2.f;
            float backX = position.x - std::cos(angleRad) * TRAILER_LENGTH;
            float backY = position.y - std::sin(angleRad) * TRAILER_LENGTH;
            sf::Vector2f center(backX, backY);

            sf::Vector2f left = center + sf::Vector2f(std::cos(angleRad + PI / 2), std::sin(angleRad + PI / 2)) * w;
            sf::Vector2f right = center + sf::Vector2f(std::cos(angleRad - PI / 2), std::sin(angleRad - PI / 2)) * w;

            sf::VertexArray mark(sf::LinesStrip, 11);
            for (int i = 0; i <= 10; ++i) {
                float t = i / 10.f;
                sf::Vector2f point = left + t * (right - left);
                mark[i] = sf::Vertex(point, sf::Color(100, 100, 100));
            }
            terrainLayer.draw(mark);
        }
    }

    void draw(sf::RenderWindow& window) {
        float angleRad = angleDeg * DEG_TO_RAD;
        sf::ConvexShape triangle;
        triangle.setPointCount(3);
        triangle.setFillColor(active ? sf::Color::Green : sf::Color::Red);

        sf::Vector2f back = position - sf::Vector2f(std::cos(angleRad), std::sin(angleRad)) * TRAILER_LENGTH;
        sf::Vector2f left = back + sf::Vector2f(std::cos(angleRad + PI / 2), std::sin(angleRad + PI / 2)) * (TRAILER_WIDTH / 2.f);
        sf::Vector2f right = back + sf::Vector2f(std::cos(angleRad - PI / 2), std::sin(angleRad - PI / 2)) * (TRAILER_WIDTH / 2.f);

        triangle.setPoint(0, position);
        triangle.setPoint(1, left);
        triangle.setPoint(2, right);

        window.draw(triangle);
    }
};

void drawGrid(sf::RenderWindow& window) {
    sf::Color gridColor(200, 200, 200);
    float step = TRAILER_WIDTH;

    for (float x = 0; x < WINDOW_WIDTH; x += step) {
        sf::Vertex line[] = {
            sf::Vertex(sf::Vector2f(x, 0), gridColor),
            sf::Vertex(sf::Vector2f(x, WINDOW_HEIGHT), gridColor)
        };
        window.draw(line, 2, sf::Lines);
    }
    for (float y = 0; y < WINDOW_HEIGHT; y += step) {
        sf::Vertex line[] = {
            sf::Vertex(sf::Vector2f(0, y), gridColor),
            sf::Vertex(sf::Vector2f(WINDOW_WIDTH, y), gridColor)
        };
        window.draw(line, 2, sf::Lines);
    }
}

int main() {
    sf::RenderWindow window(sf::VideoMode(WINDOW_WIDTH, WINDOW_HEIGHT), "Tractor Simulation");
    window.setFramerateLimit(60);

    sf::RenderTexture terrainLayer;
    terrainLayer.create(WINDOW_WIDTH, WINDOW_HEIGHT);
    terrainLayer.clear(sf::Color::Transparent);

    Tractor tractor;
    Trailer trailer;
    trailer.position = tractor.getHitchPosition() - sf::Vector2f(std::cos(tractor.angle * DEG_TO_RAD), std::sin(tractor.angle * DEG_TO_RAD)) * HITCH_LENGTH;
    trailer.angleDeg = tractor.angle;

    sf::Clock clock;
    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed) window.close();
            if (event.type == sf::Event::KeyPressed) {
                if (event.key.code == sf::Keyboard::Space) tractor.toggleCruise();
                if (event.key.code == sf::Keyboard::G) tractor.stop();
                if (event.key.code == sf::Keyboard::T) trailer.toggle();
                if (event.key.code == sf::Keyboard::K) tractor.startTurnRight();
            }
        }

        float dt = clock.restart().asSeconds();

        if (!tractor.stopping && !tractor.turningRight) {
            if (sf::Keyboard::isKeyPressed(sf::Keyboard::W))
                tractor.speed += 20.f * dt;
            else if (sf::Keyboard::isKeyPressed(sf::Keyboard::S))
                tractor.speed -= 20.f * dt;
            else if (!tractor.cruiseControl)
                tractor.speed *= 0.98f;
        }

        if (tractor.speed > MAX_SPEED) tractor.speed = MAX_SPEED;
        if (tractor.speed < -MAX_SPEED) tractor.speed = -MAX_SPEED;

        if (!tractor.turningRight) {
            if (sf::Keyboard::isKeyPressed(sf::Keyboard::A))
                tractor.steeringAngle = std::max(tractor.steeringAngle - 60.f * dt, -30.f);
            else if (sf::Keyboard::isKeyPressed(sf::Keyboard::D))
                tractor.steeringAngle = std::min(tractor.steeringAngle + 60.f * dt, 30.f);
            else
                tractor.steeringAngle *= 0.9f;
        }

        tractor.update(dt);
        trailer.update(tractor.getHitchPosition(), dt, terrainLayer);

        window.clear(sf::Color::White);
        drawGrid(window);

        sf::Sprite terrainSprite(terrainLayer.getTexture());
        window.draw(terrainSprite);

        tractor.draw(window);
        trailer.draw(window);
        window.display();
    }
    return 0;
}
