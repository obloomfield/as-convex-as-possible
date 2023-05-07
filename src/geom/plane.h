#pragma once

#include "Eigen/Dense"
#include "plane.h"
#include "mesh.h"
#include "quickhull/Structs/Plane.hpp"
#include <vector>

class Mesh;

using namespace Eigen;

class Plane {
private:
    Vector3d p0;
    Vector3d p1;
    Vector3d p2;
    Vector3d p3;
public:

    Plane(Vector3d a, Vector3d b, Vector3d c, Vector3d d) {
        p0 = a, p1 = b, p2 = c, p3 = d;
        // ensure that all points make a plane
        double EPSILON = 1e-9;
        Eigen::Matrix<double, 3, 3> mat;
        mat << (p1 - p0), (p2 - p0), (p3 - p0);
        assert(mat.determinant() < EPSILON);
    }

    Plane(const quickhull::Plane<double>& p, const Mesh& m);

    std::tuple<Vector3d,Vector3d,Vector3d,Vector3d> bounds() {
        return std::make_tuple(p0,p1,p2,p3);
    }
};
