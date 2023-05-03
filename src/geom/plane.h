#pragma once

#include "Eigen/Dense"
#include <vector>

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
    }

    std::tuple<Vector3d,Vector3d,Vector3d,Vector3d> bounds() {
        return std::make_tuple(p0,p1,p2,p3);
    }

//    std::vector<double> vertices() {
//        std::vector<double> cutMeshVertices {
//            p0.x(), p0.y(), p0.z(),
//            p1.x(), p1.y(), p1.z(),
//            p2.x(), p2.y(), p2.z(),
//            p3.x(), p3.y(), p3.z()
//        };
//        return cutMeshVertices;
//    }

//    uint32_t numCutMeshVertices = 4;
//    uint32_t numCutMeshFaces = 2;
};
