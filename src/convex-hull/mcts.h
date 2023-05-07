#pragma once

#include "geom/mesh.h"

class MCTS
{
public:
    MCTS();

    static quickhull::Plane<double> cuttingPlane(Mesh cur_mesh);
};
