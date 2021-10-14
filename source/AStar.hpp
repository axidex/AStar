#ifndef __ASTAR__
#define __ASTAR__

#pragma once
#include <vector>
#include <functional>


    struct Vec2i
    {
        int x, y;

        bool operator == (const Vec2i& coordinates_);
    };

    using HeuristicFunction = std::function<unsigned int(Vec2i, Vec2i)>; // функтор для расчета расстояния 
    using CoordinateList = std::vector<Vec2i>;

    struct Node
    {
        unsigned int G, H;
        Vec2i coordinates;
        Node *parent;

        Node(Vec2i coord_, Node *parent_ = nullptr);
        unsigned int getScore();
    };

    class Map
    {
        bool detectCollision(Vec2i coordinates_);
        Node* findNodeOnList(std::vector<Node*>& nodes_, Vec2i coordinates_);
        void releaseNodes(std::vector<Node*>& nodes_);

    public:
        Map();
        void setWorldSize(Vec2i worldSize_);
        void setDiagonalMovement(bool enable_);
        void setHeuristic(HeuristicFunction heuristic_);
        CoordinateList findPath(Vec2i source_, Vec2i target_);
        void addCollision(Vec2i coordinates_);
        void removeCollision(Vec2i coordinates_);
        void clearCollisions();

    private:
        HeuristicFunction heuristic;
        CoordinateList direction, walls;
        Vec2i worldSize;
        unsigned int directions;
    };

    class Heuristic
    {
        static Vec2i getDelta(Vec2i source_, Vec2i target_);

    public:
        static unsigned int manhattan(Vec2i source_, Vec2i target_);
        static unsigned int euclidean(Vec2i source_, Vec2i target_);
        static unsigned int octagonal(Vec2i source_, Vec2i target_);
    };


#endif // __ASTAR__
