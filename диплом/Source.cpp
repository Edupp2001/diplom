// main.cpp
#include <SFML/Graphics.hpp>
#include <cmath>
#include <iostream>
#include <vector>

constexpr float PI = 3.14159265f;
constexpr float DEG_TO_RAD = PI / 180.0f;
constexpr float METERS_TO_PIXELS = 50.0f; // Уменьшенный масштаб (в 10 раз)

// Размеры трактора (в метрах)
constexpr float TRACTOR_LENGTH_M = 3.9f;
constexpr float TRACTOR_WIDTH_M = 1.97f;

// Преобразованные размеры в пикселях
constexpr float TRACTOR_LENGTH = TRACTOR_LENGTH_M * METERS_TO_PIXELS;
constexpr float TRACTOR_WIDTH = TRACTOR_WIDTH_M * METERS_TO_PIXELS;

// Размер прицепа (ширина = шаг сетки)
constexpr float TRAILER_WIDTH_M = 2.0f;
constexpr float TRAILER_WIDTH = TRAILER_WIDTH_M * METERS_TO_PIXELS;
constexpr float TRAILER_LENGTH = 1.8f * METERS_TO_PIXELS;
constexpr float TRAILER_DISTANCE = 1.5f * METERS_TO_PIXELS; // расстояние от центра трактора до вершины прицепа

// Окно
constexpr int WINDOW_WIDTH = 1200;
constexpr int WINDOW_HEIGHT = 800;

// Ограничение скорости: 12 км/ч = 3.33 м/с
constexpr float MAX_SPEED = 3.33f * METERS_TO_PIXELS; // пикселей/сек

class Tractor {
public:
    sf::Vector2f position;
    float angle = 0.f; // Направление трактора (в градусах)
    float steeringAngle = 0.f; // Угол поворота колес (в градусах)
    float speed = 0.f; // Текущая скорость
    bool cruiseControl = true; // Круиз-контроль включён по умолчанию
    bool stopping = false; // Инициировано полное торможение

    Tractor() {
        position = sf::Vector2f(WINDOW_WIDTH / 2, WINDOW_HEIGHT / 2);
    }

    void toggleCruise() {
        cruiseControl = !cruiseControl;
        std::cout << "Cruise Control: " << (cruiseControl ? "ON" : "OFF") << std::endl;
    }

    void stop() {
        stopping = true;
    }

    void update(float dt) {
        float wheelBase = 2.45f * METERS_TO_PIXELS; // колёсная база

        if (stopping) {
            float brakeDecel = 15.f * dt;
            if (speed > 0) {
                speed -= brakeDecel;
                if (speed < 0) speed = 0;
            }
            else if (speed < 0) {
                speed += brakeDecel;
                if (speed > 0) speed = 0;
            }
            if (speed == 0) stopping = false;
        }

        float theta = angle * DEG_TO_RAD;
        float beta = atan(tan(steeringAngle * DEG_TO_RAD));
        position.x += speed * cos(theta + beta) * dt;
        position.y += speed * sin(theta + beta) * dt;

        angle += (speed / wheelBase) * tan(steeringAngle * DEG_TO_RAD) * dt * 180.0f / PI;
    }

    void draw(sf::RenderWindow& window) {
        sf::RectangleShape body(sf::Vector2f(TRACTOR_LENGTH, TRACTOR_WIDTH));
        body.setOrigin(TRACTOR_LENGTH / 2, TRACTOR_WIDTH / 2);
        body.setPosition(position);
        body.setRotation(angle);
        body.setFillColor(cruiseControl ? sf::Color::Blue : sf::Color::Red);

        sf::RectangleShape frontStripe(sf::Vector2f(TRACTOR_WIDTH * 0.6f, TRACTOR_WIDTH * 0.1f));
        frontStripe.setFillColor(sf::Color::Yellow);
        frontStripe.setOrigin(frontStripe.getSize().x / 2, frontStripe.getSize().y / 2);
        frontStripe.setPosition(position);
        frontStripe.setRotation(angle);

        float offset = TRACTOR_LENGTH / 2 - frontStripe.getSize().y;
        float dx = std::cos(angle * DEG_TO_RAD) * offset;
        float dy = std::sin(angle * DEG_TO_RAD) * offset;
        frontStripe.move(dx, dy);

        window.draw(body);
        window.draw(frontStripe);
    }
};

class Trailer {
public:
    bool active = false; // по умолчанию выключен
    sf::Vector2f axlePosition;
    float angle = 0.f;
    float angularVelocity = 0.f; // для инерции
    std::vector<sf::Vector2f> trail;

    void toggle() {
        active = !active;
        std::cout << "Trailer: " << (active ? "ACTIVE" : "INACTIVE") << std::endl;
    }

    void update(const sf::Vector2f& tractorPos, float tractorAngle, float tractorSpeed, float dt) {
        sf::Vector2f hitchPos = tractorPos - sf::Vector2f(std::cos(tractorAngle * DEG_TO_RAD), std::sin(tractorAngle * DEG_TO_RAD)) * TRACTOR_LENGTH / 2.f;

        sf::Vector2f direction = hitchPos - axlePosition;
        float distance = std::sqrt(direction.x * direction.x + direction.y * direction.y);

        if (distance > 0.001f) {
            sf::Vector2f norm = direction / distance;

            float speedFactor = std::max(tractorSpeed, 0.1f); // предотвращает нулевое смещение
            axlePosition += norm * speedFactor * dt;

            float desiredAngle = std::atan2(norm.y, norm.x) * 180.f / PI;
            float deltaAngle = desiredAngle - angle;
            while (deltaAngle > 180.f) deltaAngle -= 360.f;
            while (deltaAngle < -180.f) deltaAngle += 360.f;

            float inertia = 5.0f;
            angularVelocity += deltaAngle * dt * inertia;
            angularVelocity *= 0.9f; // затухание
            angle += angularVelocity * dt;
        }

        if (active) {
            float w = TRAILER_WIDTH / 2.f;
            float backX = axlePosition.x - std::cos(angle * DEG_TO_RAD) * TRAILER_LENGTH;
            float backY = axlePosition.y - std::sin(angle * DEG_TO_RAD) * TRAILER_LENGTH;
            sf::Vector2f center(backX, backY);

            sf::Vector2f left = center + sf::Vector2f(std::cos((angle + 90.f) * DEG_TO_RAD), std::sin((angle + 90.f) * DEG_TO_RAD)) * w;
            sf::Vector2f right = center + sf::Vector2f(std::cos((angle - 90.f) * DEG_TO_RAD), std::sin((angle - 90.f) * DEG_TO_RAD)) * w;

            for (int i = 0; i <= 10; ++i) {
                float t = i / 10.f;
                trail.push_back(left + t * (right - left));
            }
        }
    }

    void draw(sf::RenderWindow& window) {
        sf::ConvexShape triangle;
        triangle.setPointCount(3);
        triangle.setFillColor(active ? sf::Color::Green : sf::Color::Red);

        float rad = angle * DEG_TO_RAD;
        sf::Vector2f back = axlePosition - sf::Vector2f(cos(rad), sin(rad)) * TRAILER_LENGTH;
        sf::Vector2f left = back + sf::Vector2f(cos(rad + PI / 2), sin(rad + PI / 2)) * (TRAILER_WIDTH / 2.f);
        sf::Vector2f right = back + sf::Vector2f(cos(rad - PI / 2), sin(rad - PI / 2)) * (TRAILER_WIDTH / 2.f);

        triangle.setPoint(0, axlePosition);
        triangle.setPoint(1, left);
        triangle.setPoint(2, right);

        window.draw(triangle);

        for (auto& p : trail) {
            sf::CircleShape dot(1);
            dot.setPosition(p);
            dot.setFillColor(sf::Color(150, 150, 150));
            dot.setOrigin(1, 1);
            window.draw(dot);
        }
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

    Tractor tractor;
    Trailer trailer;
    trailer.axlePosition = tractor.position - sf::Vector2f(std::cos(tractor.angle * DEG_TO_RAD), std::sin(tractor.angle * DEG_TO_RAD)) * TRACTOR_LENGTH / 2.f;
    trailer.angle = tractor.angle;

    sf::Clock clock;
    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed)
                window.close();
            if (event.type == sf::Event::KeyPressed) {
                if (event.key.code == sf::Keyboard::Space)
                    tractor.toggleCruise();
                if (event.key.code == sf::Keyboard::G)
                    tractor.stop();
                if (event.key.code == sf::Keyboard::T)
                    trailer.toggle();
            }
        }

        float dt = clock.restart().asSeconds();

        if (!tractor.stopping) {
            if (sf::Keyboard::isKeyPressed(sf::Keyboard::W))
                tractor.speed += 20.f * dt;
            else if (sf::Keyboard::isKeyPressed(sf::Keyboard::S))
                tractor.speed -= 20.f * dt;
            else if (!tractor.cruiseControl)
                tractor.speed *= 0.98f;
        }

        if (tractor.speed > MAX_SPEED) tractor.speed = MAX_SPEED;
        if (tractor.speed < -MAX_SPEED) tractor.speed = -MAX_SPEED;

        if (sf::Keyboard::isKeyPressed(sf::Keyboard::A))
            tractor.steeringAngle = std::max(tractor.steeringAngle - 60.f * dt, -30.f);
        else if (sf::Keyboard::isKeyPressed(sf::Keyboard::D))
            tractor.steeringAngle = std::min(tractor.steeringAngle + 60.f * dt, 30.f);
        else
            tractor.steeringAngle *= 0.9f;

        tractor.update(dt);
        trailer.update(tractor.position, tractor.angle, tractor.speed, dt);

        window.clear(sf::Color::White);
        drawGrid(window);
        tractor.draw(window);
        trailer.draw(window);

        window.display();
    }

    return 0;
}