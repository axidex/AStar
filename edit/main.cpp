#include <iostream>
#include <chrono>
#include "../source/AStar.hpp"

int main()
{
    auto start = std::chrono::steady_clock::now();
    Map map;
    map.setWorldSize({9, 9});
    map.setHeuristic(Heuristic::manhattan);
    map.setDiagonalMovement(false);
    
    map.addCollision({ 1, 7 });
    map.addCollision({ 2, 7 });
    map.addCollision({ 3, 7 });
    map.addCollision({ 4, 7 });
    map.addCollision({ 5, 7 });
    map.addCollision({ 6, 7 });
    map.addCollision({ 7, 7 });

    map.addCollision({ 2, 6 });
    map.addCollision({ 3, 5 });
    map.addCollision({ 4, 4 });
    map.addCollision({ 5, 3 });
    map.addCollision({ 6, 2 });
    map.addCollision({ 7, 1 });

    map.addCollision({ 7, 2 });
    map.addCollision({ 7, 3 });
    map.addCollision({ 7, 4 });
    map.addCollision({ 7, 5 });
    map.addCollision({ 7, 6 });

    std::cout << "Generate path ... \n";
    
    auto path = map.findPath({0, 0}, {8, 8});
    

    for(auto& coordinate : path) {
        std::cout << coordinate.x << " " << coordinate.y << "\n";
    }
    auto end = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
    std::cout << "elapsed time: " << elapsed_seconds.count() << "s\n";

    system("pause");
}