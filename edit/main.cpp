#include <iostream>
#include "../source/AStar.hpp"

int main()
{
    Map map;
    map.setWorldSize({10, 10});
    map.setHeuristic(Heuristic::octagonal);
    map.setDiagonalMovement(true);
    map.addCollision({ 3, 6 });
    map.addCollision({ 6, 3 });
    map.addCollision({ 4, 6 });
    map.addCollision({ 6, 4 });
    map.addCollision({ 5, 4 });
    map.addCollision({ 4, 5 });
    map.addCollision({ 5, 5 });

    std::cout << "Generate path ... \n";
    auto path = map.findPath({0, 0}, {9, 9});


    for(auto& coordinate : path) {
        std::cout << coordinate.x << " " << coordinate.y << "\n";
    }
    system("pause");
}