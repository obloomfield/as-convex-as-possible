#pragma once

#include "geom/mesh.h"

class MCTS
{
public:
    MCTS();

    static quickhull::Plane<double> cuttingPlane(const Mesh& cur_mesh);

    static quickhull::Plane<double> cuttingPlaneGreedy(const Mesh& cur_mesh);
};
