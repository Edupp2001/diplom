// [автоматический разворот по полуокружности добавлен по клавише K]
#include <SFML/Graphics.hpp>
#include <cmath>
#include <iostream>

constexpr float PI = 3.14159265f;
constexpr float DEG_TO_RAD = PI / 180.0f;
constexpr float METERS_TO_PIXELS = 15.0f;

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
float overlay = 0.f; // в метрах
struct canonline {
    float x1, y1;
    float dx, dy;//dx = x2 - x1, dy = y2 - y1
    // (x - x1) / dx = (y - y1) / dy
    float startangle;
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
            sf::ConvexShape line;
            line.setPointCount(4);
            line.setFillColor(sf::Color(128, 128, 0));

            sf::Vector2f back = position - sf::Vector2f(std::cos(angleRad), std::sin(angleRad)) * TRAILER_LENGTH;
            sf::Vector2f left = back + sf::Vector2f(std::cos(angleRad + PI / 2), std::sin(angleRad + PI / 2)) * w;
            sf::Vector2f right = back + sf::Vector2f(std::cos(angleRad - PI / 2), std::sin(angleRad - PI / 2)) * w;
            //center.y == 400

            line.setPoint(0, left + sf::Vector2f(0, 5 + 2 * (400 - left.y)));
            line.setPoint(1, left + sf::Vector2f(-5, 2 * (400 - left.y)));//fix
            line.setPoint(3, right + sf::Vector2f(0, -5 + 2 * (400 - right.y)));
            line.setPoint(2, right + sf::Vector2f(5, 2 * (400 - right.y)));


            terrainLayer.draw(line);
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
void drawCircle(sf::RenderWindow& window, float x, float y, float r) {
    sf::CircleShape circle(r);
    circle.setOrigin(r, r);          // Центр круга в его геометрическом центре
    circle.setPosition(x, y);         // Позиция круга — это теперь его центр
    circle.setFillColor(sf::Color::Transparent);
    circle.setOutlineColor(sf::Color::Red);
    circle.setOutlineThickness(1.f);

    window.draw(circle);
    //system("pause");
}
int checkdots(float x, float y, std::vector<sf::Vector2f>& fd, float theta) {
    if (fd.size() < 2) {
        return 1;
    }
    float newx1 = 0;
    float newy1 = 0;
    float xsh = fd[0].x - x;
    float ysh = fd[0].y - y;
    float newy2 = xsh * cos(theta) + ysh * sin(theta);
    float newx2 = -xsh * sin(theta) + ysh * cos(theta);
    xsh = fd[1].x - x;
    ysh = fd[1].y - y;
    float newy3 = xsh * cos(theta) + ysh * sin(theta);
    float newx3 = -xsh * sin(theta) + ysh * cos(theta);
    //чтобы точки считались "хорошими", нужно чтобы вторая точка относительно первой была в той же четверти, что и вторая относительно первой
    if (((newy3 > newy2) && (newy2 > newy1)) && (((newx1 < newx2) && (newx2 < newx3)) || ((newx1 > newx2) && (newx2 > newx3)))) //BAD CONDITION FIIXX
    {
        return 0;///FIIX

    }
    return 1;
}
class Tractor {
public:
    std::vector<sf::Vector2f> predictedPath;
    bool showPredictedPath = true;
    Trailer trailer;
    sf::Vector2f position;
    float angle = 0.f;//getdata from compass
    float steeringAngle = 0.f;
    float speed = 0.f;
    bool cruiseControl = true;
    bool stopping = false;
    float targetangle;
    //разворот
    std::string turningAuto = "";
    int turnPhase = 0; // 0 — не поворачиваем, 1 — выезд на дугу, 2 — выравнивание, TODO: 3 - стабилизирование(убираем погрешность)
    float traveled = 0.f;
    float targetTravel = 0.f;
    //forward
    bool movingForward = false;
    float forwardTarget = 0.f;
    float forwardTraveled = 0.f;
    //forward
    //fill
    //todo
    //mpc
    bool mpcMode = false;
    float replanTimer = 0.f;
    float replanInterval = 0.01f;
    //mpc
    
    void toggleCruise() { cruiseControl = !cruiseControl; }
    void stop() { stopping = true; }
    Tractor() {
        position = sf::Vector2f(WINDOW_WIDTH / 2, WINDOW_HEIGHT / 2);
    }
    void startTurn(std::string side) {
        if (turningAuto != "") return;
        targetangle = angle * DEG_TO_RAD;
        turningAuto = side;
        turnPhase = 1;
        traveled = 0.f;
        targetTravel = 5.4f * METERS_TO_PIXELS; // проехать 6 метра прямо влево
    }
    void forward(float meters = 25.0f) {
        if (!movingForward) {
            forwardTarget = meters * METERS_TO_PIXELS; // 25 метров
            forwardTraveled = 0.f;
            movingForward = true;
            cruiseControl = false; // отключим круиз, если активен
        }
    }

    void update(float dt) {
        float wheelBase = 2.45f * METERS_TO_PIXELS;
        if (stopping) {
            float brakeDecel = 15.f * dt;
            if (speed > 0) speed = std::max(0.f, speed - brakeDecel);
            else if (speed < 0) speed = std::min(0.f, speed + brakeDecel);
            if (speed == 0) stopping = false;
        }

        if (turningAuto != "") {//my method(bad)
            float targetSpeed = 3.0f * METERS_TO_PIXELS;
            if (speed < targetSpeed) speed = std::min(speed + 20.f * dt, targetSpeed);
            else if (speed > targetSpeed) speed = std::max(speed - 20.f * dt, targetSpeed);

            if (turnPhase == 1) {
                steeringAngle = 30.f * (turningAuto == "right" ? -1 : 1);
                float dx = speed * dt * std::cos(angle * DEG_TO_RAD);
                float dy = speed * dt * std::sin(angle * DEG_TO_RAD);
                traveled += std::sqrt(dx * dx + dy * dy);
                if (traveled >= targetTravel) {
                    turnPhase = 2;
                }
            }
            else if (turnPhase == 2) {
                steeringAngle = 30.f * (turningAuto == "left" ? -1 : 1);
                
                float a = angle * DEG_TO_RAD;
                
                if ((abs(cos(targetangle) + cos(a)) < 0.001)) { //       
                        turningAuto = "";
                        steeringAngle = 0.f;
                        speed = 0;//fix
                        this->forward(12.8);
                }    
            }//TODO turnPhase 3, -> exact angle
            

        }
        float theta = angle * DEG_TO_RAD;
        float beta = atan(tan(steeringAngle * DEG_TO_RAD));
        position.x += speed * cos(theta + beta) * dt;
        position.y += speed * sin(theta + beta) * dt;
        if (movingForward) {
            float dx = speed * dt * std::cos(angle * DEG_TO_RAD);
            float dy = speed * dt * std::sin(angle * DEG_TO_RAD);
            float distance = std::sqrt(dx * dx + dy * dy);
            forwardTraveled += distance;

            float targetSpeed = 3.0f * METERS_TO_PIXELS;
            if (speed < targetSpeed)
                speed = std::min(speed + 20.f * dt, targetSpeed);
            else if (speed > targetSpeed)
                speed = std::max(speed - 20.f * dt, targetSpeed);

            if (forwardTraveled >= forwardTarget) {
                speed = 0.f;
                movingForward = false;
                
            }
        }
        angle += (speed / wheelBase) * tan(steeringAngle * DEG_TO_RAD) * dt * 180.0f / PI;
        if (showPredictedPath) {
            predictPath();
        }
    }
    std::vector <sf::Vector2f> predictPathf(float tempAngle, float steeringAngle, sf::Vector2f tempPos, float tempSpeed, bool full = true) {
        std::vector <sf::Vector2f> Path;
        const float duration = 5.f;
        const float timestep = 0.1f;
        float wheelBase = 2.45f * METERS_TO_PIXELS;
        for (float t = 0.f; t < duration; t += timestep) {
            float theta = tempAngle * DEG_TO_RAD;
            float beta = atan(tan(steeringAngle * DEG_TO_RAD));
            tempPos.x += tempSpeed * cos(theta + beta) * timestep;
            tempPos.y += tempSpeed * sin(theta + beta) * timestep;
            tempAngle += (tempSpeed / wheelBase) * tan(steeringAngle * DEG_TO_RAD) * timestep * 180.0f / PI;
            Path.push_back(tempPos);
        }
        if (full)
            return Path;
        //
        std::vector <sf::Vector2f> thispp;
        sf::Vector2f tmp = position;
        for (int i = 0; i < Path.size() && thispp.size() < 5; ++i) {
            while (i < Path.size() &&
                sqrt(
                    (tmp.x - Path[i].x) * (tmp.x - Path[i].x)
                    +
                    (tmp.y - Path[i].y) * (tmp.y - Path[i].y)
                ) <= 5)
            {
                ++i;
            }
            if (i < Path.size()) {
                tmp = Path[i];
                thispp.push_back(tmp);
            }
        }
        return thispp;
    }
    void predictPath() {
        predictedPath.clear();
        float tempAngle = angle;
        sf::Vector2f tempPos = position;
        float tempSpeed = speed;
        predictedPath = predictPathf(angle, steeringAngle, position, speed);
        showPredictedPath = true;
    }
    void startMPC() {
        if (!mpcMode) {
            mpcMode = true;
            replanTimer = 0.f;
        }
    }
    void updateMPC(float dt, const std::vector<sf::Vector2f>& path, sf::RenderWindow& window) {
        if (!mpcMode || path.empty())
            return;

        checkReachTarget(path);

        replanTimer += dt;

        float speedRatio = speed / MAX_SPEED;
        replanInterval = (1.0f - speedRatio) * 0.09f + 0.01f;

        if (replanTimer >= replanInterval) {
            replanControl(path, window);
            replanTimer = 0.f;
        }
    }
    std::pair<float, float> findU(std::vector<sf::Vector2f> &fd, sf::RenderWindow& window) {
        float bestspeed = speed;
        float bestangle;
        float bestlen = 1000000;
        const float da = 30 * DEG_TO_RAD;
        float left = std::max(steeringAngle - da, -30 * DEG_TO_RAD);
        float right = std::min(steeringAngle + da, 30 * DEG_TO_RAD);

        std::vector <sf::Vector2f> IRC(3);
        std::vector <float> R(3), len(3);
        std::cout << "start" << std::endl;
        while (abs(right - left) > 0.0001 && abs(bestlen) > 0.01) {
            R[0] = R[1] = R[2] = 0;
            //tan(x) < 0.1 => (x < 5 degree)
            if (abs(tan(left)) >= 0.1)
                R[0] = TRACTOR_LENGTH / tan(left);
            if (abs(tan((left + right) / 2)) >= 0.1)
                R[1] = TRACTOR_LENGTH / tan((left + right) / 2);
            if (abs(tan(right)) >= 0.1)
                R[2] = TRACTOR_LENGTH / tan(right);
            
            std::cout << "{" << std::endl;
            for (int i = 0; i < 3; ++i) {
                if (R[i] != 0) {
                    IRC[i].x = position.x + R[i] * sin(angle * DEG_TO_RAD);
                    IRC[i].y = position.y - R[i] * cos(angle * DEG_TO_RAD);
                    //int dotsaregood = checkdots(position.x, position.y, fd, angle * DEG_TO_RAD);//0 if good, 1 if bad
                    len[i] = 0;
                    std::cout << IRC[i].x << " " << IRC[i].y << " " << R[i] << std::endl;
                    for (int j = 0; j < fd.size() - 1/*если точки плохие нужно искать минимум с одной*/; ++j) {
                        len[i] += sqrt((IRC[i].x - fd[j].x) * (IRC[i].x - fd[j].x) + (IRC[i].y - fd[j].y) * (IRC[i].y - fd[j].y)) - abs(R[i]);
                        std::cout << sqrt((IRC[i].x - fd[j].x) * (IRC[i].x - fd[j].x) + (IRC[i].y - fd[j].y) * (IRC[i].y - fd[j].y)) - abs(R[i]) << " " << fd[j].x << " " << fd[j].y << std::endl;
                    }
                    
                }
                else {
                    //line
                    len[i] = 0;

                    sf::Vector2f nextpos;
                    nextpos.x = position.x + cos(angle);
                    nextpos.y = position.y + sin(angle);
                    float A = position.x - nextpos.x;
                    float B = nextpos.y - position.y;
                    float C = nextpos.x * position.y - nextpos.y * position.x;
                    for (int j = 0; j < fd.size(); ++j) {
                        len[i] += (abs(A * fd[j].x + B * fd[j].y + C) / sqrt(A * A + B * B));
                        
                    }
                }
                if (abs(bestlen) > abs(len[i])) {
                    bestlen = len[i];
                    if (i == 0) {
                        bestangle = left;
                    }
                    else if (i == 1) {
                        bestangle = (left + right) / 2;
                    }
                    else {
                        bestangle = right;
                    }
                }
            }
            std::cout << "}" << std::endl;
            if (bestlen < 0) {//fix
                left = (left + right) / 2;
            }
            else {
                right = (left + right) / 2;
            }
            
            
            std::cout << bestlen << " " << bestangle << std::endl;
        }
        
        std::cout << "end" << std::endl;
        //std::cout << c << std::endl;
        return std::make_pair(bestangle, bestspeed);
    }
    void replanControl(const std::vector<sf::Vector2f>& path, sf::RenderWindow& window) {
        if (path.empty())
            return;
        
        sf::Vector2f target = path.front();
        sf::Vector2f toTarget = target - position;
        std::pair <float, float> bestU;//first is angle, second is v
        std::vector <sf::Vector2f> firstdots;
        for (int i = 0; i < path.size() && i < 2; ++i) {
            firstdots.push_back(path[i]);
        }
        
        bestU = findU(firstdots, window);
        
        //float desiredAngle = bestU.first;//std::atan2(toTarget.y, toTarget.x) * 180.f / PI;
        //float deltaAngle = desiredAngle - angle;
        //while (deltaAngle > 180.f) deltaAngle -= 360.f;
        //while (deltaAngle < -180.f) deltaAngle += 360.f;
        
        // Настройка руля
        steeringAngle = bestU.first;

        // Настройка скорости: быстрее если далеко, медленнее если близко
        float distance = std::sqrt(toTarget.x * toTarget.x + toTarget.y * toTarget.y);
        float dv = 2.0f * METERS_TO_PIXELS;
        if (distance > 2.0f * METERS_TO_PIXELS)
            speed = std::min(speed + dv, MAX_SPEED);
        else
            speed = bestU.second;//std::max(distance * 0.5f, 1.0f * METERS_TO_PIXELS);

        // Без заднего хода
        if (speed < 0.f) speed = 0.f;
    }

    void checkReachTarget(const std::vector<sf::Vector2f>& path) {
        if (path.empty())
            return;

        sf::Vector2f target = path.front();
        float dx = target.x - position.x;
        float dy = target.y - position.y;
        float distance = std::sqrt(dx * dx + dy * dy);

        if (distance < 0.5f * METERS_TO_PIXELS) {

            // Цель достигнута
            // Следующую точку должен удалить вызывающий код (main)
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
        if (showPredictedPath && predictedPath.size() > 1) {
            sf::VertexArray lines(sf::LineStrip, predictedPath.size());
            for (size_t i = 0; i < predictedPath.size(); ++i) {
                lines[i].position = predictedPath[i];
                lines[i].color = sf::Color::Magenta;
            }
            window.draw(lines);
        }
    }

    sf::Vector2f getHitchPosition() const {
        float theta = angle * DEG_TO_RAD;
        return position - sf::Vector2f(std::cos(theta), std::sin(theta)) * (TRACTOR_LENGTH / 2);
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
    window.setFramerateLimit(100);

    sf::RenderTexture terrainLayer;
    terrainLayer.create(WINDOW_WIDTH, WINDOW_HEIGHT);
    terrainLayer.clear(sf::Color::Transparent);

    Tractor tractor;
    tractor.trailer;
    tractor.trailer.position = tractor.getHitchPosition() - sf::Vector2f(std::cos(tractor.angle * DEG_TO_RAD), std::sin(tractor.angle * DEG_TO_RAD)) * HITCH_LENGTH;
    tractor.trailer.angleDeg = tractor.angle;

    sf::Clock clock;
    std::vector<sf::Vector2f> drawnPoints;
    bool isDrawing = false;
    while (window.isOpen()) {
        //std::cout << tractor.steeringAngle << std::endl;
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed) window.close();
            if (event.type == sf::Event::KeyPressed) {
                if (event.key.code == sf::Keyboard::Space) tractor.toggleCruise();
                if (event.key.code == sf::Keyboard::G) tractor.stop();
                if (event.key.code == sf::Keyboard::T) tractor.trailer.toggle();
                if (event.key.code == sf::Keyboard::K) {
                    tractor.startTurn("right");
                }
                if (event.key.code == sf::Keyboard::J) {
                    tractor.startTurn("left");
                }
                if (event.key.code == sf::Keyboard::F) {
                    tractor.forward();
                }
                if (event.key.code == sf::Keyboard::L) {
                    tractor.showPredictedPath = !tractor.showPredictedPath;
                }
                if (event.key.code == sf::Keyboard::M) {
                    tractor.startMPC();
                }

            }
            if (event.type == sf::Event::MouseButtonPressed && event.mouseButton.button == sf::Mouse::Left) {
                drawnPoints.clear();
                isDrawing = true;
            }
            if (event.type == sf::Event::MouseButtonReleased && event.mouseButton.button == sf::Mouse::Left) {
                isDrawing = false;
            }
        }
        if (isDrawing) {
            sf::Vector2f mousePos = window.mapPixelToCoords(sf::Mouse::getPosition(window));
            if (drawnPoints.empty() ||
                std::hypot(mousePos.x - drawnPoints.back().x, mousePos.y - drawnPoints.back().y) > 5.f) {
                drawnPoints.push_back(mousePos);
            }
        }
        float dt = clock.restart().asSeconds();

        if (!tractor.stopping && tractor.turningAuto == "") {
            if (sf::Keyboard::isKeyPressed(sf::Keyboard::W))
                tractor.speed += 20.f * dt;
            else if (sf::Keyboard::isKeyPressed(sf::Keyboard::S))
                tractor.speed -= 20.f * dt;
            //else if (!tractor.cruiseControl)
                //tractor.speed *= 0.98f;
        }

        if (tractor.speed > MAX_SPEED) tractor.speed = MAX_SPEED;//fix
        if (tractor.speed < -MAX_SPEED) tractor.speed = -MAX_SPEED;

        if (tractor.turningAuto == "") {
            if (sf::Keyboard::isKeyPressed(sf::Keyboard::A))
                tractor.steeringAngle = std::max(tractor.steeringAngle - 60.f * dt, -30.f);
            else if (sf::Keyboard::isKeyPressed(sf::Keyboard::D))
                tractor.steeringAngle = std::min(tractor.steeringAngle + 60.f * dt, 30.f);
            //else
                //tractor.steeringAngle *= 0.9f;
        }
        
        
        
        tractor.update(dt);
        tractor.updateMPC(dt, drawnPoints, window);
        if (!drawnPoints.empty()) {
            sf::Vector2f toTarget = drawnPoints.front() - tractor.position;
            if (std::sqrt(toTarget.x * toTarget.x + toTarget.y * toTarget.y) < 0.2f * METERS_TO_PIXELS) {
                drawnPoints.erase(drawnPoints.begin());
            }
        }
        tractor.trailer.update(tractor.getHitchPosition(), dt, terrainLayer);

        window.clear(sf::Color::White);
        drawGrid(window);
        //drawCircle(window, 600, 300, 100);
        sf::Sprite terrainSprite(terrainLayer.getTexture());
        window.draw(terrainSprite);

        tractor.draw(window);
        tractor.trailer.draw(window);

        if (drawnPoints.size() > 1) {
            sf::VertexArray line(sf::LineStrip, drawnPoints.size());
            for (size_t i = 0; i < drawnPoints.size(); ++i) {
                line[i].position = drawnPoints[i];
                line[i].color = sf::Color::Black;
            }
            window.draw(line);
        }

        window.display();
    }
    return 0;
}
