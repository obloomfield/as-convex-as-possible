#pragma once

#include "Eigen/Dense"
#include "geom/mesh.h"

int ITERATIONS = 10;
int DEPTH = 3;

struct Plane
{
    // TODO: implement...
};

struct Node
{
    Eigen::Vector3f position;
    int index;
    Node *next;

};

class MCTS {
public:

    static Plane cuttingPlane(const Mesh& mesh);

    static void clip();

private:
    static Plane mcts(const Mesh& mesh, int iterations, int depth);
    static std::tuple<std::vector<Plane>, int> TreePolicy(Node v, int depth);
    static std::tuple<std::vector<Plane>, int> DefaultPolicy(Node v, int depth);
};


