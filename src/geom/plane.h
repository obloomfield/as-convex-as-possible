#pragma once

#include <vector>

#include "Eigen/Dense"
#include "mesh.h"
#include "plane.h"
#include "quickhull/Structs/Plane.hpp"

class Mesh;

class Plane {
 private:
    Eigen::Vector3d p0;
    Eigen::Vector3d p1;
    Eigen::Vector3d p2;
    Eigen::Vector3d p3;

 public:
    Plane(Eigen::Vector3d a, Eigen::Vector3d b, Eigen::Vector3d c, Eigen::Vector3d d) {
        p0 = a, p1 = b, p2 = c, p3 = d;
        // ensure that all points make a plane
        double EPSILON = 1e-9;
        Eigen::Matrix<double, 3, 3> mat;
        mat << (p1 - p0), (p2 - p0), (p3 - p0);
        assert(mat.determinant() < EPSILON);
    }

    Plane(const quickhull::Plane<double>& p, const Mesh& m);

    std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d> bounds() {
        return std::make_tuple(p0, p1, p2, p3);
    }
};
