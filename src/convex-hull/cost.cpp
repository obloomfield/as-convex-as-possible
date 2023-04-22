#include "cost.h"

float Cost::v_cost(const Mesh& m1, const Mesh& m2) {

    float v1 = m1.volume(), v2 = m2.volume();

    return pow(3 * fabs(v1 - v2) / (4 * M_PI), 1.0 / 3.) * K;
}


float Cost::h_cost(const Mesh& m1, const Mesh& m2) {
    // TODO: figure out cutting cost

    // farthest away the closest thing is...

    return 0.;
}

// ~~~

float Cost::total_cost(const Mesh& m1, const Mesh& m2) {
    return fmax(v_cost(m1,m2), h_cost(m1,m2));
}
