#pragma once

#include "geom/mesh.h"

double K = 100.;

class Cost {
public:

    static float total_cost(const Mesh& m1, const Mesh& m2);
private:
    static float h_cost(const Mesh& m1, const Mesh& m2);
    static float v_cost(const Mesh& m1, const Mesh& m2);
};
