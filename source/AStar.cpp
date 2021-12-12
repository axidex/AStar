#include "AStar.hpp"
#include <algorithm>



inline bool Vec2i::operator == (const Vec2i& coordinates_)
{
    return (x == coordinates_.x && y == coordinates_.y);
}

inline Vec2i operator + (const Vec2i& left_, const Vec2i& right_)
{
    return{ left_.x + right_.x, left_.y + right_.y };
}

Node::Node(Vec2i coordinates_, Node* parent_) // Конструктор по умолчанию для узла
{
    parent = parent_;
    coordinates = coordinates_;
    G = H = 0;
}

inline unsigned int Node::getScore() // -_- геттер
{
    return G + H;
}

Map::Map() // Конструктор по умолчанию для карты
{
    setDiagonalMovement(false);
    setHeuristic(&Heuristic::manhattan);
    direction = {
        { 0, 1 }, { 1, 0 }, { 0, -1 }, { -1, 0 },
        { -1, -1 }, { 1, 1 }, { -1, 1 }, { 1, -1 }
    };
}

void Map::setWorldSize(Vec2i worldSize_) // сеттер для размеров
{
    worldSize = worldSize_;
}

inline void Map::setDiagonalMovement(bool enable_) // флаг для перемещения по диагонали
{
    directions = (enable_ ? 8 : 4);
}

void Map::setHeuristic(HeuristicFunction heuristic_) // сеттер для функции которая расчитывает расстояние
{
    heuristic = std::bind(heuristic_, std::placeholders::_1, std::placeholders::_2); // сеттер для функтора который расчитывает расстояния до нужной точки
                                                                                    // нужно 2 плейсхолдера тк функция принимает 2 значения
}

void Map::addCollision(Vec2i coordinates_) // добавить стену
{
    walls.push_back(coordinates_);
}

void Map::removeCollision(Vec2i coordinates_) // удалить определенную стену
{
    auto it = std::find(walls.begin(), walls.end(), coordinates_);
    if (it != walls.end()) {
        walls.erase(it);
    }
}

void Map::clearCollisions() // удалить все стены
{
    walls.clear();
}

CoordinateList Map::findPath(Vec2i source_, Vec2i target_)
{
    Node* current = nullptr;
    std::vector<Node*> openSet, closedSet; // в открытом храним узлы в которых мы будем проводить вычесления
                                           // в закрытых - нет
    openSet.reserve(100);
    closedSet.reserve(100);
    openSet.push_back(new Node(source_));

    while (!openSet.empty()) {
        auto current_it = openSet.begin();
        current = *current_it;

        for (auto it = openSet.begin(); it != openSet.end(); it++) {
            auto node = *it;
            if (node->getScore() <= current->getScore()) {
                current = node;
                current_it = it;
            }
        }

        if (current->coordinates == target_) // выход если дошли
        {
            break;
        }

        closedSet.push_back(current);
        openSet.erase(current_it);

        for (unsigned int i = 0; i < directions; ++i)  // Для каждого узла вокруг вычисляю G+H учитывая эвристику.
                                                       // Ниже пример если по диагонали ходить можно => вычисляем у всех 'x'     
                                               /*
                                                      x  x  x
                                                      x |*| x
                                                      x  x  x
                                               */
        {
            Vec2i newCoordinates(current->coordinates + direction[i]);
            if (detectCollision(newCoordinates) ||
                findNodeOnList(closedSet, newCoordinates)) {
                continue;
            }

            unsigned int totalCost = current->G + ((i < 4) ? 10 : 14); // если по диагоналям ходить можно следовательно добовлять будем sqrt(2) примерно 1.4
                                                                       // удобно это реализвать через тернарное выражение  

            Node* successor = findNodeOnList(openSet, newCoordinates);
            if (successor == nullptr)
            {
                successor = new Node(newCoordinates, current);
                successor->G = totalCost;
                successor->H = heuristic(successor->coordinates, target_);
                openSet.push_back(successor);
            }
            else if (totalCost < successor->G) // прокладываем путь указывая родителя
                                               /* пример для выкл диагоналей
                                                            x
                                                            |
                                                            ∨
                                                       x ->|*|<- x
                                                            ∧
                                                            |
                                                            x
                                               */
            {
                successor->parent = current;
                successor->G = totalCost;
            }
        }
    }

    CoordinateList path;
    while (current != nullptr)  // делаем вектор который содержит координаты пути.
    {
        path.push_back(current->coordinates);
        current = current->parent;
    }

    releaseNodes(openSet);
    releaseNodes(closedSet);

    return path;
}

Node* Map::findNodeOnList(std::vector<Node*>& nodes_, Vec2i coordinates_) // Найти узел по координатам если нет то NULL
{
    for (auto node : nodes_) {
        if (node->coordinates == coordinates_) {
            return node;
        }
    }
    return nullptr;
}

void Map::releaseNodes(std::vector<Node*>& nodes_) // чистим вектор узлов
{
    for (auto it = nodes_.begin(); it != nodes_.end();) {
        delete* it;
        it = nodes_.erase(it);
    }
}

bool Map::detectCollision(Vec2i coordinates_) // проверка на стену по координатам
{
    if (coordinates_.x < 0 || coordinates_.x >= worldSize.x ||
        coordinates_.y < 0 || coordinates_.y >= worldSize.y ||
        std::find(walls.begin(), walls.end(), coordinates_) != walls.end()) {
        return true;
    }
    return false;
}

inline Vec2i Heuristic::getDelta(Vec2i source_, Vec2i target_)
{
    return{ abs(source_.x - target_.x),  abs(source_.y - target_.y) };
}

// Эвристика (высчитывание расстояния)
unsigned int Heuristic::manhattan(Vec2i source_, Vec2i target_) // Манхеттан => вверх вниз влево вправо
{
    auto delta = std::move(getDelta(source_, target_)); // r-value
    return static_cast<unsigned int>(10 * (delta.x + delta.y));
}

unsigned int Heuristic::euclidean(Vec2i source_, Vec2i target_) // по диагонали расстояние как длина вектора
{
    auto delta = std::move(getDelta(source_, target_));
    return static_cast<unsigned int>(10 * sqrt(pow(delta.x, 2) + pow(delta.y, 2)));
}

unsigned int Heuristic::octagonal(Vec2i source_, Vec2i target_) // Расстояние Чебышёва
{
    auto delta = std::move(getDelta(source_, target_));
    return 10 * std::max(delta.x, delta.y);
}